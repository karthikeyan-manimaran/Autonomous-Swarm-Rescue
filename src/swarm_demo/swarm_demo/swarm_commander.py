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

    def __init__(self, node, drone_number):

        self.node = node
        self.number = drone_number

        # PX4 ROS 2 namespace
        self.namespace = f"/px4_{drone_number}"

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.armed = False
        self.active = True

        self.target_x = 0.0
        self.target_y = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # ----------------------------------------------------
        # Publishers
        # ----------------------------------------------------

        self.command_pub = node.create_publisher(
            VehicleCommand,
            f"{self.namespace}/fmu/in/vehicle_command",
            10
        )

        self.offboard_pub = node.create_publisher(
            OffboardControlMode,
            f"{self.namespace}/fmu/in/offboard_control_mode",
            10
        )

        self.trajectory_pub = node.create_publisher(
            TrajectorySetpoint,
            f"{self.namespace}/fmu/in/trajectory_setpoint",
            10
        )

        # ----------------------------------------------------
        # Subscribers
        # ----------------------------------------------------

        node.create_subscription(
            VehicleLocalPosition,
            f"{self.namespace}/fmu/out/vehicle_local_position",
            self.position_callback,
            qos
        )

        node.create_subscription(
            VehicleStatus,
            f"{self.namespace}/fmu/out/vehicle_status",
            self.status_callback,
            qos
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

        # PX4 instance:
        #
        # px4_1 -> MAV_SYS_ID 1
        # px4_2 -> MAV_SYS_ID 2
        # etc.
        #
        msg.target_system = self.number
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

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
            f"[D{self.number}] ARM"
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
            f"[D{self.number}] OFFBOARD"
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
        # ----------------------------------------------------

        self.drones = []

        for i in range(1, NUM_DRONES + 1):

            drone = DroneAgent(
                self,
                i
            )

            self.drones.append(drone)

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
        for drone in self.drones:

            if drone.active:

                drone.publish_offboard_mode()

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

            for drone in self.drones:

                drone.set_offboard()

            self.state = "ARM"

            self.state_time = time.time()

        # ----------------------------------------------------
        # ARM
        # ----------------------------------------------------

        elif self.state == "ARM":

            if time.time() - self.state_time > 1.0:

                for drone in self.drones:

                    drone.arm()

                self.state = "TAKEOFF"

                self.state_time = time.time()

        # ----------------------------------------------------
        # TAKEOFF
        # ----------------------------------------------------

        elif self.state == "TAKEOFF":

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

            for drone in self.drones:

                if drone.active:

                    drone.goto(
                        drone.x,
                        drone.y,
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
