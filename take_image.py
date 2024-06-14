
import os
import numpy as np
import rclpy
from as2_python_api.drone_interface import DroneInterface
import cv2
from sensor_msgs.msg import Image


class DroneInspector(DroneInterface):
    def __init__(self, drone_id: str = "drone0", verbose: bool = False, use_sim_time: bool = False):
        super().__init__(drone_id=drone_id, verbose=verbose, use_sim_time=use_sim_time)

        self.create_subscription(
            Image, 'sensor_measurements/cam/image_raw', self.image_upload, 10)  # change topic name

        self.i = 0

    def image_upload(self, msg: Image):

        self.image_received = msg

    def save_image(self):
        self.get_logger().info('Image received')

        # Location and name
        folder_dir = "drone_images_manual"

        # Converting ROS images to compatible file
        image_np = np.frombuffer(self.image_received.data, dtype=np.uint8)

        image_np = image_np.reshape((self.image_received.height,
                                     self.image_received.width, -1))

        # Location and name
        image_name = f'image_taken{self.i}.png'
        image_path = os.path.join(folder_dir, image_name)

        # Adding image to folder
        cv2.imwrite(image_path, image_np)

        self.get_logger().info('Image uploaded')
        self.i += 1


def manual_image(drone_inspector: DroneInspector):
    while True:
        answer = input("T to take image, EXIT to shut off: ").lower()

        if answer == "t":
            drone_inspector.save_image()

        elif answer == "exit":
            print("Shutting down")
            break

        else:
            print("Unknown selection")


if __name__ == '__main__':
    rclpy.init()

    uav = DroneInspector("drone0", verbose=False, use_sim_time=True)

    manual_image(uav)

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
