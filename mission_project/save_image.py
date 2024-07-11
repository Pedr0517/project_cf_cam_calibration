import os
import numpy as np
import rclpy
from as2_python_api.drone_interface import DroneInterface
import cv2
from sensor_msgs.msg import Image
import tkinter as tk


class DroneInspector(DroneInterface):
    def __init__(self, drone_id: str = "drone0", verbose: bool = False, use_sim_time: bool = False):
        super().__init__(drone_id=drone_id, verbose=verbose, use_sim_time=use_sim_time)

        self.create_subscription(
            Image, '/drone_sim_aerostack_0/sensor_measurements/camera/image', self.image_upload, 10)  # change topic name

        self.i = 568

    def image_upload(self, msg: Image):
        """Will upload images to folder when called upon"""
        self.get_logger().info('Image received')

        self.image_received = msg

        # Location and name
        folder_dir = "./wide01_720_images_manual/"
        os.makedirs(folder_dir, exist_ok=True)

        # Converting ROS images to compatible file
        image_np = np.frombuffer(self.image_received.data, dtype=np.uint8)
        image_np = image_np.reshape((msg.height, msg.width, -1))

        # Location and name
        image_name = f'image_taken{self.i}.png'
        image_path = os.path.join(folder_dir, image_name)

        # Adding image to folder
        cv2.imwrite(image_path, image_np)

        self.get_logger().info(f'Image saved at {image_path}')
        self.i += 1


if __name__ == '__main__':
    rclpy.init()

    uav = DroneInspector("drone0", verbose=False, use_sim_time=True)

    root = tk.Tk()
    root.title("Drone Image Capture")
    root.geometry("400x200")

    label = tk.Label(root, text="Images are being saved automatically.")
    label.pack(expand=True)

    root.mainloop()

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
