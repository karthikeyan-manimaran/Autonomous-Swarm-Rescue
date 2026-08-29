#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLocalPosition,
    VehicleStatus,
)


# ============================================================
# CONFIGURATION
# ============================================================

NUM_DRONES = 5

TAKEOFF_ALTITUDE = -5.0

CONTROL_RATE = 20.0

WAYPOINT_REACHED_DISTANCE = 1.5

# Search area
SEARCH_POINTS = [
    (0.0, 0.0),
    (10.0, 0.0),
    (10.0, 10.0),
    (0.0, 10.0),
]

# Initial positions of the 5 swarm drones
START_POSITIONS = [
    (0.0, 0.0),
    (5.0, 0.0),
    (10.0, 0.0),
    (0.0, 5.0),
    (10.0, 5.0),
]


# ============================================================
# DRONE AGENT
# ============================================================

class DroneAgent:

    def __init__(
        self,
        node,
        label,
        namespace,
        target_system
    ):

        self.node = node
        self.label = label

        # PX4 ROS 2 namespace, e.g. "/commander" or "/px4_1"
        self.namespace = namespace

        # MAV_SYS_ID this PX4 instance actually uses.
        self.target_system = target_system

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.armed = False
        self.active = True

        self.target_x = 0.0
        self.target_y = 0.0

        # Publisher QoS: TRANSIENT_LOCAL is fine here (a stronger
        # durability than PX4's subscribers request is compatible).
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Subscriber QoS: PX4 publishes fmu/out/* with BEST_EFFORT +
        # VOLATILE. A subscriber requesting TRANSIENT_LOCAL can never
        # match a VOLATILE publisher in DDS (the QoS is incompatible,
        # not just "not ideal") - this was silently blackholing every
        # position/status/ack subscription below for every vehicle,
        # commander included. Must mirror PX4's own QoS to receive
        # anything at all.
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # ----------------------------------------------------
        # Publishers
        # ----------------------------------------------------

        self.command_pub = node.create_publisher(
            VehicleCommand,
            f"{self.namespace}/fmu/in/vehicle_command",
            pub_qos
        )

        self.offboard_pub = node.create_publisher(
            OffboardControlMode,
            f"{self.namespace}/fmu/in/offboard_control_mode",
            pub_qos
        )

        self.trajectory_pub = node.create_publisher(
            TrajectorySetpoint,
            f"{self.namespace}/fmu/in/trajectory_setpoint",
            pub_qos
        )

        # ----------------------------------------------------
        # Subscribers
        # ----------------------------------------------------

        node.create_subscription(
            VehicleLocalPosition,
            f"{self.namespace}/fmu/out/vehicle_local_position",
            self.position_callback,
            sub_qos
        )

        node.create_subscription(
            VehicleStatus,
            f"{self.namespace}/fmu/out/vehicle_status",
            self.status_callback,
            sub_qos
        )

        node.create_subscription(
            VehicleCommandAck,
            f"{self.namespace}/fmu/out/vehicle_command_ack",
            self.ack_callback,
            sub_qos
        )

    # ========================================================
    # POSITION CALLBACK
    # ========================================================

    def position_callback(self, msg):

        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

    # ========================================================
    # STATUS CALLBACK
    # ========================================================

    def status_callback(self, msg):

        self.armed = (
            msg.arming_state ==
            VehicleStatus.ARMING_STATE_ARMED
        )

    # ========================================================
    # VEHICLE COMMAND ACK CALLBACK
    # ========================================================

    def ack_callback(self, msg):

        result_names = {
            0: "ACCEPTED",
            1: "TEMPORARILY_REJECTED",
            2: "DENIED",
            3: "UNSUPPORTED",
            4: "FAILED",
            5: "IN_PROGRESS",
            6: "CANCELLED",
        }

        result_text = result_names.get(
            msg.result,
            f"UNKNOWN({msg.result})"
        )

        self.node.get_logger().info(
            f"[{self.label}] ACK cmd={msg.command} "
            f"result={result_text}"
        )

    # ========================================================
    # SEND VEHICLE COMMAND
    # ========================================================

    def send_command(
        self,
        command,
        param1=0.0,
        param2=0.0
    ):

        msg = VehicleCommand()

        msg.command = command

        msg.param1 = float(param1)
        msg.param2 = float(param2)

        # target_system is passed in explicitly at
        # construction time - it must match this PX4
        # instance's actual MAV_SYS_ID, or PX4 silently
        # ignores every command.
        msg.target_system = self.target_system
        msg.target_component = 1

        # source_system must identify the SENDER (this ROS 2/GCS
        # node), never a vehicle's own MAV_SYS_ID. It was hardcoded
        # to 1, which collides with the commander's own sysid -
        # every command to every slave looked like it originated
        # from vehicle #1 instead of an external controller. Use
        # the conventional GCS/companion-computer id instead.
        msg.source_system = 255
        msg.source_component = 190

        msg.from_external = True

        msg.timestamp = (
            self.node.get_clock()
            .now()
            .nanoseconds // 1000
        )

        self.command_pub.publish(msg)

    # ========================================================
    # ARM
    # ========================================================

    def arm(self):

        self.node.get_logger().info(
            f"[{self.label}] ARM"
        )

        self.send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0
        )

    # ========================================================
    # OFFBOARD
    # ========================================================

    def set_offboard(self):

        self.node.get_logger().info(
            f"[{self.label}] OFFBOARD"
        )

        self.send_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0
        )

    # ========================================================
    # OFFBOARD CONTROL HEARTBEAT
    # ========================================================

    def publish_offboard_mode(self):

        msg = OffboardControlMode()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        msg.timestamp = (
            self.node.get_clock()
            .now()
            .nanoseconds // 1000
        )

        self.offboard_pub.publish(msg)

    # ========================================================
    # SET POSITION
    # ========================================================

    def goto(self, x, y, z=TAKEOFF_ALTITUDE):

        self.target_x = float(x)
        self.target_y = float(y)

        msg = TrajectorySetpoint()

        msg.position = [
            float(x),
            float(y),
            float(z)
        ]

        msg.yaw = 0.0

        msg.timestamp = (
            self.node.get_clock()
            .now()
            .nanoseconds // 1000
        )

        self.trajectory_pub.publish(msg)

    # ========================================================
    # TARGET DISTANCE
    # ========================================================

    def distance_to_target(self):

        dx = self.x - self.target_x
        dy = self.y - self.target_y

        return math.sqrt(
            dx * dx +
            dy * dy
        )


# ============================================================
# SWARM COMMANDER
# ============================================================

class SwarmCommander(Node):

    def __init__(self):

        super().__init__(
            "swarm_commander"
        )

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            "     AUTONOMOUS DRONE SWARM SYSTEM"
        )

        self.get_logger().info(
            f"     Number of drones: {NUM_DRONES}"
        )

        self.get_logger().info(
            "=========================================="
        )

        # ----------------------------------------------------
        # Create swarm
        #
        # NOTE: the commander (Omnicopter) is PX4 instance 0,
        # namespace "/commander", MAV_SYS_ID 1 by default -
        # it was previously never created as a DroneAgent at
        # all, so it never received arm/offboard/goto commands.
        # ----------------------------------------------------

        self.commander = DroneAgent(
            self,
            "CMD",
            "/commander",
            1
        )

        self.drones = []

        for i in range(1, NUM_DRONES + 1):

            # PX4 launched with `-i N` defaults to
            # MAV_SYS_ID = N + 1.
            drone = DroneAgent(
                self,
                f"D{i}",
                f"/px4_{i}",
                i + 1
            )

            self.drones.append(drone)

        # All agents, commander included - used for the
        # heartbeat/offboard/arm steps that should apply
        # to every vehicle in the swarm.
        self.all_agents = (
            [self.commander] + self.drones
        )

        # ----------------------------------------------------
        # Mission state
        # ----------------------------------------------------

        self.state = "INITIALIZE"

        self.state_time = time.time()

        self.search_index = 0

        self.failed_drone = None

        # ----------------------------------------------------
        # Main loop
        # ----------------------------------------------------

        self.timer = self.create_timer(
            1.0 / CONTROL_RATE,
            self.control_loop
        )

    # ========================================================
    # MAIN CONTROL LOOP
    # ========================================================

    def control_loop(self):

        # Always publish Offboard heartbeat
        for agent in self.all_agents:

            if agent.active:

                agent.publish_offboard_mode()

        # ----------------------------------------------------
        # INITIALIZE
        # ----------------------------------------------------

        if self.state == "INITIALIZE":

            self.get_logger().info(
                "Initializing swarm..."
            )

            self.state = "PRE_OFFBOARD"

            self.state_time = time.time()

        # ----------------------------------------------------
        # SEND SETPOINTS BEFORE OFFBOARD
        # ----------------------------------------------------

        elif self.state == "PRE_OFFBOARD":

            self.commander.goto(
                0.0,
                0.0,
                TAKEOFF_ALTITUDE
            )

            for i, drone in enumerate(self.drones):

                x, y = START_POSITIONS[i]

                drone.goto(
                    x,
                    y,
                    TAKEOFF_ALTITUDE
                )

            if time.time() - self.state_time > 2.0:

                self.state = "OFFBOARD"

                self.state_time = time.time()

        # ----------------------------------------------------
        # OFFBOARD
        # ----------------------------------------------------

        elif self.state == "OFFBOARD":

            for agent in self.all_agents:

                agent.set_offboard()

            # Keep resending for 2s - a single command
            # can be dropped over best-effort DDS.
            if time.time() - self.state_time > 2.0:

                self.state = "ARM"

                self.state_time = time.time()

        # ----------------------------------------------------
        # ARM
        # ----------------------------------------------------

        elif self.state == "ARM":

            for agent in self.all_agents:

                agent.arm()

            # Keep resending for 5s - gives PX4's health
            # checks (e.g. power status) time to clear,
            # and a single command can be dropped over
            # best-effort DDS.
            if time.time() - self.state_time > 5.0:

                self.state = "TAKEOFF"

                self.state_time = time.time()

        # ----------------------------------------------------
        # TAKEOFF
        # ----------------------------------------------------

        elif self.state == "TAKEOFF":

            self.commander.goto(
                0.0,
                0.0,
                TAKEOFF_ALTITUDE
            )

            for i, drone in enumerate(self.drones):

                if not drone.active:
                    continue

                x, y = START_POSITIONS[i]

                drone.goto(
                    x,
                    y,
                    TAKEOFF_ALTITUDE
                )

            if time.time() - self.state_time > 8.0:

                self.get_logger().info(
                    "ALL DRONES AIRBORNE"
                )

                self.state = "SEARCH"

                self.state_time = time.time()

        # ----------------------------------------------------
        # SEARCH
        # ----------------------------------------------------

        elif self.state == "SEARCH":

            self.execute_search()

            # Run search for 30 seconds
            if time.time() - self.state_time > 30:

                self.state = "FAILURE"

                self.state_time = time.time()

        # ----------------------------------------------------
        # FAILURE DEMONSTRATION
        # ----------------------------------------------------

        elif self.state == "FAILURE":

            # Simulate Drone 3 failure
            self.failed_drone = 3

            self.drones[
                self.failed_drone - 1
            ].active = False

            self.get_logger().warn(
                "=========================================="
            )

            self.get_logger().warn(
                "DRONE 3 FAILURE DETECTED"
            )

            self.get_logger().warn(
                "Initiating task reallocation..."
            )

            self.get_logger().warn(
                "=========================================="
            )

            self.state = "REDISTRIBUTE"

            self.state_time = time.time()

        # ----------------------------------------------------
        # TASK REDISTRIBUTION
        # ----------------------------------------------------

        elif self.state == "REDISTRIBUTE":

            self.reallocate_task()

            if time.time() - self.state_time > 12:

                self.state = "RETURN"

                self.state_time = time.time()

        # ----------------------------------------------------
        # RETURN
        # ----------------------------------------------------

        elif self.state == "RETURN":

            self.get_logger().info(
                "Mission complete."
            )

            self.get_logger().info(
                "Returning active swarm..."
            )

            self.commander.goto(
                0.0,
                0.0,
                TAKEOFF_ALTITUDE
            )

            for i, drone in enumerate(self.drones):

                if not drone.active:
                    continue

                x, y = START_POSITIONS[i]

                drone.goto(
                    x,
                    y,
                    TAKEOFF_ALTITUDE
                )

            if time.time() - self.state_time > 15:

                self.state = "LAND"

                self.state_time = time.time()

        # ----------------------------------------------------
        # LAND
        # ----------------------------------------------------

        elif self.state == "LAND":

            self.get_logger().info(
                "Landing swarm..."
            )

            for agent in self.all_agents:

                if agent.active:

                    agent.goto(
                        agent.x,
                        agent.y,
                        0.0
                    )

            self.state = "DONE"

        # ----------------------------------------------------
        # DONE
        # ----------------------------------------------------

        elif self.state == "DONE":

            pass

    # ========================================================
    # SEARCH ALGORITHM
    # ========================================================

    def execute_search(self):

        # Every 6 seconds move to next search point

        elapsed = time.time() - self.state_time

        index = int(elapsed / 6.0)

        point = index % len(SEARCH_POINTS)

        base_x, base_y = SEARCH_POINTS[point]

        for i, drone in enumerate(self.drones):

            if not drone.active:
                continue

            # Formation spacing
            offset_x = (i % 3) * 3.0
            offset_y = (i // 3) * 3.0

            target_x = base_x + offset_x
            target_y = base_y + offset_y

            drone.goto(
                target_x,
                target_y,
                TAKEOFF_ALTITUDE
            )

    # ========================================================
    # DYNAMIC TASK REALLOCATION
    # ========================================================

    def reallocate_task(self):

        self.get_logger().info(
            "Swarm evaluating available drones..."
        )

        # Drone 4 takes Drone 3's mission
        replacement = self.drones[3]

        if replacement.active:

            replacement.goto(
                10.0,
                10.0,
                TAKEOFF_ALTITUDE
            )

            self.get_logger().info(
                "Drone 4 assigned Drone 3 task."
            )

            self.get_logger().info(
                "Distributed task reallocation complete."
            )


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = SwarmCommander()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()
