
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
            Image, '/drone0/sensor_measurements/camera', self.image_upload, 10)  # change topic name

        self.image_received = None

        self.i = 0

    def image_upload(self, msg: Image):
        """Receives image data"""

        self.image_received = msg

    def save_image(self):
        """Will upload images to folder when called upon"""

        self.get_logger().info('Image received')

        # Location and name#
        folder_dir = "drone_images_manual"

        # Checks for foler path#
        if not os.path.exists(folder_dir):
            os.makedirs(folder_dir)

        if self.image_received is None:
            print("no image received yet")
            return
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


def manual_image(event, drone_inspector: DroneInspector):
    drone_inspector.save_image()


if __name__ == '__main__':
    rclpy.init()

    uav = DroneInspector("drone0", verbose=False, use_sim_time=True)

    root = tk.Tk()
    root.title("Drone Image Capture")
    root.geometry("400x200")

    label = tk.Label(root, text="Left click anywhere in this window to capture an image")
    label.pack(expand=True)

    root.bind("<Button-1>", lambda event: manual_image(event, uav))

    root.mainloop()

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
