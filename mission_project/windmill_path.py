import rclpy
from as2_python_api.drone_interface import DroneInterface
from time import sleep
import argparse


def windmill_mission(drone_inspector: DroneInterface, path_data: list, angle: float):

    speed = 0.5
    takeoff_height = 1.0
    path = path_data
    angle = angle

    sleep_time = 2.0

    print("Start Inspection")

    print("Arm")
    drone_inspector.offboard()
    sleep(sleep_time)
    print("Offboard")
    drone_inspector.arm()
    sleep(sleep_time)

    # TAKE OFF #
    print("Take Off")
    drone_inspector.takeoff(takeoff_height, speed=speed)
    print("Take Off done")
    sleep(1)

    # INSPECTION PATH#

    for goal in path[0][1]:
        print(f"Go to point {list(goal)}")
        drone_inspector.go_to.go_to_point_with_yaw(list(goal), speed=speed, angle=angle)
        sleep(sleep_time)
    # LANDING#
    print("Landing")
    drone_inspector.land(speed=0.5)
    print("Land done")


if __name__ == '__main__':
    rclpy.init()

    path = []
    path.append((0, [(-2, 0, 1), (-2, 0, 3), (-2, 0, 5)]))

    print(path[0][1])

    # path.append(0, (-1, 0, 1))
    # path.append(0, (-1, 0, 3))
    # path.append(0, (-1, 0, 5))

    uav = DroneInterface("drone0", verbose=False, use_sim_time=True)

    parser = argparse.ArgumentParser(description='Yaw of Drone')
    parser.add_argument('--yaw_angle', type=float, default=0, help='Yaw for drone')

    args = parser.parse_args()

    windmill_mission(uav, path, args.yaw_angle)

    uav.shutdown()
    rclpy.shutdown()
    print("clean exit")
