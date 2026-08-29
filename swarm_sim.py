#!/usr/bin/env python3

import os
import sys
import time
import signal
import shutil
import subprocess
from pathlib import Path


# ============================================================
# CONFIGURATION
# ============================================================

PX4_DIR = Path.home() / "PX4-Autopilot"

PX4_BIN = (
    PX4_DIR
    / "build"
    / "px4_sitl_default"
    / "bin"
    / "px4"
)

SWARM_WS = Path.home() / "swarm_ws"

NUM_SLAVES = 5

XRCE_PORT = "8888"

SIM_SPEED = "1"

# Set to True to run Gazebo without the GUI window.
# Use this if the GUI client crashes/blanks out due to a
# broken OpenGL/GLX stack (common on Wayland + Intel/NVIDIA
# driver conflicts) - the physics server, PX4, and ROS 2
# swarm logic all still run normally without the window.
HEADLESS = os.environ.get("SWARM_HEADLESS", "0") == "1"

# How many seconds to wait before checking whether the
# commander process (and its GUI, if enabled) is still alive.
COMMANDER_STARTUP_WAIT = 15

# Commander
COMMANDER_MODEL = "gz_omnicopter"

# NOTE: 10019 does not trigger PX4's internal Gazebo auto-launch
# for this model - PX4 just waits forever on TCP port 4560 for
# an external simulator that never connects. 8011 is the model's
# own default and is confirmed working (spawns gz sim + GUI).
COMMANDER_AUTOSTART = "8011"

# X500
SLAVE_MODEL = "gz_x500"
SLAVE_AUTOSTART = "4001"

# Commander position
COMMANDER_POS = (0.0, 0.0, 5.0)

# X500 positions
SLAVE_POSITIONS = [
    (-10.0, 5.0, 0.0),
    (0.0, 10.0, 0.0),
    (10.0, 5.0, 0.0),
    (-5.0, -7.0, 0.0),
    (5.0, -7.0, 0.0),
]

# Base remote MAVLink port PX4 SITL opens per instance for GCS-type
# connections (instance N -> 14550 + N). We connect a lightweight
# heartbeat sender to each of these so every vehicle - commander AND
# all 5 slaves - sees a GCS datalink and clears the "No connection to
# the ground control station" health/arming check without needing a
# full QGroundControl GUI running.
GCS_BASE_PORT = 14550


# ============================================================
# COLORS
# ============================================================

RESET = "\033[0m"
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
WHITE = "\033[97m"


def log(text, color=WHITE):
    print(f"{color}{text}{RESET}", flush=True)


# ============================================================
# PROCESS LIST
# ============================================================

processes = []


# ============================================================
# CLEANUP
# ============================================================

def cleanup_old_simulation():

    log(
        "Cleaning old PX4/Gazebo processes...",
        YELLOW
    )

    commands = [
        ["pkill", "-9", "-f", "px4"],
        ["pkill", "-9", "-f", "gz sim"],
        ["pkill", "-9", "-f", "swarm_commander"],
        ["pkill", "-9", "-f", "MicroXRCEAgent"],
    ]

    for command in commands:

        try:
            subprocess.run(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception:
            pass

    time.sleep(3)

    log(
        "Old simulation cleaned.",
        GREEN
    )


def cleanup():

    log(
        "\nStopping swarm simulation...",
        YELLOW
    )

    # Stop our child processes first
    for process in reversed(processes):

        try:

            if process.poll() is None:
                process.terminate()

        except Exception:
            pass

    time.sleep(2)

    for process in reversed(processes):

        try:

            if process.poll() is None:
                process.kill()

        except Exception:
            pass

    # Also clean PX4/Gazebo processes
    try:
        subprocess.run(
            ["pkill", "-9", "-f", "px4"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass

    try:
        subprocess.run(
            ["pkill", "-9", "-f", "gz sim"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass

    try:
        subprocess.run(
            ["pkill", "-9", "-f", "swarm_commander"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass

    try:
        subprocess.run(
            ["pkill", "-9", "-f", "MicroXRCEAgent"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass

    log(
        "Simulation stopped.",
        GREEN
    )


def signal_handler(sig, frame):

    cleanup()
    sys.exit(0)


signal.signal(
    signal.SIGINT,
    signal_handler
)

signal.signal(
    signal.SIGTERM,
    signal_handler
)


# ============================================================
# ENVIRONMENT
# ============================================================

def check_environment():

    log(
        "\n[1/9] Checking PX4/Gazebo environment...",
        CYAN
    )

    log(
        f"DISPLAY={os.environ.get('DISPLAY')}  "
        f"XAUTHORITY={os.environ.get('XAUTHORITY')}  "
        f"XDG_RUNTIME_DIR={os.environ.get('XDG_RUNTIME_DIR')}  "
        f"XDG_SESSION_TYPE={os.environ.get('XDG_SESSION_TYPE')}",
        YELLOW
    )

    if not PX4_DIR.exists():

        log(
            f"PX4 directory missing: {PX4_DIR}",
            RED
        )

        sys.exit(1)

    if not PX4_BIN.exists():

        log(
            f"PX4 binary missing:\n{PX4_BIN}",
            RED
        )

        log(
            "Build PX4 first:",
            YELLOW
        )

        log(
            "cd ~/PX4-Autopilot && make px4_sitl",
            YELLOW
        )

        sys.exit(1)

    if shutil.which("gz") is None:

        log(
            "Gazebo 'gz' command not found.",
            RED
        )

        sys.exit(1)

    if shutil.which("ros2") is None:

        log(
            "ROS 2 command not found.",
            RED
        )

        sys.exit(1)

    if HEADLESS:

        log(
            "HEADLESS mode enabled - skipping GUI/DISPLAY check.",
            YELLOW
        )

    elif not os.environ.get("DISPLAY"):

        log(
            "DISPLAY is not set - no X session available for the Gazebo GUI.",
            RED
        )

        log(
            "If you're on SSH, reconnect with 'ssh -X' or run this on the local desktop session.",
            YELLOW
        )

        log(
            "Alternatively, rerun with SWARM_HEADLESS=1 to skip the GUI.",
            YELLOW
        )

        sys.exit(1)

    log(
        "PX4 OK",
        GREEN
    )

    log(
        "Gazebo OK",
        GREEN
    )

    log(
        "ROS 2 OK",
        GREEN
    )


# ============================================================
# PX4 MSG CHECK
# ============================================================

def check_px4_msgs():

    log(
        "\n[2/9] Checking px4_msgs...",
        CYAN
    )

    result = subprocess.run(
        [
            "bash",
            "-c",
            (
                "source /opt/ros/humble/setup.bash && "
                f"source {SWARM_WS}/install/setup.bash && "
                "ros2 pkg list"
            ),
        ],
        capture_output=True,
        text=True,
    )

    if "px4_msgs" not in result.stdout:

        log(
            "px4_msgs is not available.",
            RED
        )

        log(
            "Run colcon build in ~/swarm_ws first.",
            YELLOW
        )

        sys.exit(1)

    log(
        "px4_msgs OK",
        GREEN
    )


# ============================================================
# XRCE-DDS
# ============================================================

def start_xrce():

    log(
        "\n[3/9] Starting Micro XRCE-DDS Agent...",
        CYAN
    )

    agent = shutil.which(
        "MicroXRCEAgent"
    )

    if agent is None:

        log(
            "MicroXRCEAgent not found.",
            RED
        )

        log(
            "Install/build Micro XRCE-DDS Agent before continuing.",
            YELLOW
        )

        sys.exit(1)

    process = subprocess.Popen(
        [
            agent,
            "udp4",
            "-p",
            XRCE_PORT,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    processes.append(process)

    time.sleep(2)

    if process.poll() is not None:

        log(
            "MicroXRCEAgent failed to start.",
            RED
        )

        sys.exit(1)

    log(
        f"MicroXRCEAgent running on UDP {XRCE_PORT}",
        GREEN
    )


# ============================================================
# START COMMANDER + GAZEBO
# ============================================================

def start_commander():

    log(
        "\n[4/9] Starting Gazebo + Omnicopter commander...",
        CYAN
    )

    env = os.environ.copy()

    if HEADLESS:

        # Tell PX4/Gazebo to skip the GUI client entirely.
        env["HEADLESS"] = "1"

        log(
            "Running Gazebo headless (no GUI window).",
            YELLOW
        )

    else:

        # Force GUI mode (default behavior).
        env.pop("HEADLESS", None)

    env["PX4_SIM_MODEL"] = COMMANDER_MODEL

    env["PX4_SYS_AUTOSTART"] = COMMANDER_AUTOSTART

    env["PX4_GZ_MODEL_POSE"] = (
        f"{COMMANDER_POS[0]},"
        f"{COMMANDER_POS[1]},"
        f"{COMMANDER_POS[2]},"
        "0,0,0"
    )

    env["PX4_SIM_SPEED_FACTOR"] = SIM_SPEED

    env["PX4_UXRCE_DDS_NS"] = "commander"

    env["PX4_UXRCE_DDS_PORT"] = XRCE_PORT

    command = [
        "make",
        "px4_sitl",
        "gz_omnicopter",
    ]

    log(
        "Starting:",
        WHITE
    )

    log(
        "make px4_sitl gz_omnicopter",
        WHITE
    )

    commander_log_path = PX4_DIR / "commander_gz.log"

    commander_log_file = open(
        commander_log_path,
        "w"
    )

    log(
        f"Commander output logged to: {commander_log_path}",
        YELLOW
    )

    process = subprocess.Popen(
        command,
        cwd=PX4_DIR,
        env=env,
        start_new_session=True,
        stdout=commander_log_file,
        stderr=subprocess.STDOUT,
    )

    processes.append(process)

    log(
        "Waiting for Gazebo GUI and commander...",
        YELLOW
    )

    # Allow Gazebo + PX4 to initialize.
    time.sleep(15)

    if process.poll() is not None:

        log(
            "Omnicopter PX4 process exited.",
            RED
        )

        sys.exit(1)

    log(
        "Gazebo + Omnicopter commander started.",
        GREEN
    )


# ============================================================
# START X500
# ============================================================

def start_slave(instance, position):

    env = os.environ.copy()

    # Never start another Gazebo server.
    env["PX4_GZ_STANDALONE"] = "1"

    env["PX4_SYS_AUTOSTART"] = SLAVE_AUTOSTART

    # NOTE: PX4_SIM_MODEL is only used as a fallback alias for
    # PX4_GZ_MODEL when PX4_GZ_MODEL itself is unset - and PX4
    # explicitly does NOT honor PX4_GZ_MODEL_POSE through that
    # alias path. Since each slave needs a custom spawn pose,
    # PX4_GZ_MODEL must be set directly, or the spawn/pose can
    # silently misbehave and the X500 never fully attaches to
    # Gazebo - which means its uXRCE-DDS client never finishes
    # bringing up /px4_i/fmu/* topics, so the commander's ROS 2
    # node is publishing arm/offboard/goto into a vacuum.
    env["PX4_SIM_MODEL"] = SLAVE_MODEL
    env["PX4_GZ_MODEL"] = SLAVE_MODEL

    env["PX4_GZ_MODEL_POSE"] = (
        f"{position[0]},"
        f"{position[1]},"
        f"{position[2]},"
        "0,0,0"
    )

    env["PX4_SIM_SPEED_FACTOR"] = SIM_SPEED

    env["PX4_UXRCE_DDS_PORT"] = XRCE_PORT

    env["PX4_UXRCE_DDS_NS"] = (
        f"px4_{instance}"
    )

    # Each instance has its own working directory.
    instance_dir = (
        PX4_DIR
        / "build"
        / "px4_sitl_default"
        / f"instance_{instance}"
    )

    instance_dir.mkdir(
        parents=True,
        exist_ok=True
    )

    command = [
        str(PX4_BIN),
        "-i",
        str(instance),
    ]

    log(
        f"Starting X500 #{instance} "
        f"at {position}...",
        WHITE
    )

    slave_log_path = instance_dir / "px4.log"

    slave_log_file = open(
        slave_log_path,
        "w"
    )

    process = subprocess.Popen(
        command,
        cwd=instance_dir,
        env=env,
        stdout=slave_log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )

    processes.append(process)

    time.sleep(2)

    if process.poll() is not None:

        log(
            f"X500 #{instance} failed.",
            RED
        )

    else:

        log(
            f"X500 #{instance} started. "
            f"Log: {slave_log_path}",
            GREEN
        )


def start_all_slaves():

    log(
        "\n[5/9] Starting 5 slave drones...",
        CYAN
    )

    for i in range(
        1,
        NUM_SLAVES + 1
    ):

        start_slave(
            i,
            SLAVE_POSITIONS[i - 1]
        )


# ============================================================
# GCS HEARTBEAT (silences "No connection to the GCS" arming
# check on every vehicle without needing a full QGroundControl
# GUI - just a background pymavlink heartbeat per instance)
# ============================================================

def start_gcs_heartbeats():

    log(
        "\n[6/9] Starting lightweight GCS heartbeat links...",
        CYAN
    )

    try:
        import pymavlink  # noqa: F401
    except ImportError:

        log(
            "pymavlink not installed - installing now "
            "(pip install pymavlink)...",
            YELLOW
        )

        result = subprocess.run(
            [sys.executable, "-m", "pip", "install", "pymavlink"],
            capture_output=True,
            text=True,
        )

        if result.returncode != 0:

            log(
                "Failed to install pymavlink automatically. "
                "Install it manually (pip install pymavlink) "
                "or run QGroundControl instead to provide the "
                "GCS link.",
                RED
            )

            log(result.stderr, RED)

            return

    heartbeat_script = (
        "import sys, time\n"
        "from pymavlink import mavutil\n"
        "port = int(sys.argv[1])\n"
        "conn = mavutil.mavlink_connection(\n"
        "    'udpout:127.0.0.1:%d' % port,\n"
        "    source_system=255,\n"
        "    source_component=190,\n"
        ")\n"
        "while True:\n"
        "    conn.mav.heartbeat_send(\n"
        "        mavutil.mavlink.MAV_TYPE_GCS,\n"
        "        mavutil.mavlink.MAV_AUTOPILOT_INVALID,\n"
        "        0, 0, 0,\n"
        "    )\n"
        "    time.sleep(1)\n"
    )

    # One tiny heartbeat process per vehicle (instance 0 = commander,
    # instances 1..NUM_SLAVES = slaves). Each PX4 SITL instance N
    # exposes a GCS-facing remote MAVLink UDP port at 14550 + N.
    for instance in range(0, NUM_SLAVES + 1):

        port = GCS_BASE_PORT + instance

        process = subprocess.Popen(
            [sys.executable, "-c", heartbeat_script, str(port)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

        processes.append(process)

        label = "commander" if instance == 0 else f"slave {instance}"

        log(
            f"  Heartbeat -> 127.0.0.1:{port} ({label})",
            WHITE
        )

    log(
        "GCS heartbeat links running for all vehicles.",
        GREEN
    )


# ============================================================
# ROS COMMANDER
# ============================================================

def start_ros_commander():

    log(
        "\n[7/9] Starting ROS 2 swarm commander...",
        CYAN
    )

    command = [
        "bash",
        "-c",
        (
            "source /opt/ros/humble/setup.bash && "
            f"source {SWARM_WS}/install/setup.bash && "
            "ros2 run drone_swarm swarm_commander"
        ),
    ]

    process = subprocess.Popen(
        command,
        cwd=SWARM_WS,
        start_new_session=True,
    )

    processes.append(process)

    time.sleep(5)

    if process.poll() is not None:

        log(
            "ROS 2 swarm commander exited.",
            RED
        )

        sys.exit(1)

    log(
        "ROS 2 swarm commander running.",
        GREEN
    )


# ============================================================
# STATUS
# ============================================================

def show_status():

    log(
        "\n[8/9] SWARM STATUS",
        CYAN
    )

    print()

    log(
        "                 COMMANDER",
        CYAN
    )

    log(
        "              Omnicopter #0",
        WHITE
    )

    print()

    log(
        "                   SLAVES",
        CYAN
    )

    for i in range(
        1,
        NUM_SLAVES + 1
    ):

        position = SLAVE_POSITIONS[
            i - 1
        ]

        log(
            f"       X500 #{i}   {position}",
            WHITE
        )

    print()

    log(
        "Mission:",
        CYAN
    )

    log(
        "  Formation",
        WHITE
    )

    log(
        "  Takeoff",
        WHITE
    )

    log(
        "  Area search",
        WHITE
    )

    log(
        "  Victim/task detection",
        WHITE
    )

    log(
        "  Failure detection",
        WHITE
    )

    log(
        "  Task reassignment",
        WHITE
    )

    log(
        "  Return",
        WHITE
    )

    log(
        "  Landing",
        WHITE
    )


# ============================================================
# WAIT
# ============================================================

def monitor():

    log(
        "\n[9/9] Simulation running...",
        GREEN
    )

    log(
        "Gazebo GUI should now be visible.",
        GREEN
    )

    log(
        "Press CTRL+C to stop everything.",
        YELLOW
    )

    while True:

        time.sleep(2)

        # If ROS commander exits, stop.
        if processes:

            ros_process = processes[-1]

            if ros_process.poll() is not None:

                log(
                    "ROS swarm commander stopped.",
                    YELLOW
                )

                break


# ============================================================
# MAIN
# ============================================================

def main():

    print()

    log(
        "============================================================",
        CYAN
    )

    log(
        "             AUTONOMOUS DRONE SWARM",
        CYAN
    )

    log(
        "                 PX4 + GAZEBO + ROS 2",
        CYAN
    )

    log(
        "============================================================",
        CYAN
    )

    log(
        f"\nCommander : {COMMANDER_MODEL}",
        WHITE
    )

    log(
        f"Slaves    : {NUM_SLAVES} x {SLAVE_MODEL}",
        WHITE
    )

    log(
        f"PX4       : {PX4_DIR}",
        WHITE
    )

    try:

        # Important:
        # clean previous failed runs first.
        cleanup_old_simulation()

        check_environment()

        check_px4_msgs()

        start_xrce()

        # FIRST PX4 starts Gazebo GUI.
        start_commander()

        # Remaining PX4 instances connect
        # to the existing Gazebo.
        start_all_slaves()

        # Give Gazebo/PX4 time to settle.
        time.sleep(5)

        start_ros_commander()

        show_status()

        monitor()

    except KeyboardInterrupt:

        pass

    finally:

        cleanup()


if __name__ == "__main__":

    main()
