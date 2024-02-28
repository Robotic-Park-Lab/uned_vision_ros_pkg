import logging
import time
import os
import rclpy
import yaml
import tf_transformations
from threading import Timer
import numpy as np
import random
from math import atan2, cos, sin, sqrt, pi
import shutil
import datetime
import platform
from cv_bridge import CvBridge
import cv2

from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, UInt16, UInt16MultiArray, Float64
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped
from sensor_msgs.msg import Image


import argparse
import asyncio
from typing import Final
import queue
from rich.console import Console

from open_gopro import Params, WiredGoPro, WirelessGoPro
from open_gopro.constants import WebcamError, WebcamStatus
from open_gopro.demos.gui.components.util import display_video_blocking
from open_gopro.gopro_base import GoProBase
from open_gopro.logger import setup_logging
from open_gopro.util import add_cli_args_and_parse

console = Console()

STREAM_URL: Final[str] = r"udp://0.0.0.0:8554"

def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Setup and view a GoPro webcam using TS protocol.")
    protocol = parser.add_argument_group("protocol", "Mutually exclusive Protocol option if not default wired USB.")
    group = protocol.add_mutually_exclusive_group()
    group.add_argument(
        "--wireless",
        action="store_true",
        help="Set to use wireless (BLE / WIFI) instead of wired (USB)) interface",
    )
    group.add_argument(
        "--cohn",
        action="store_true",
        help="Communicate via COHN. Assumes COHN is already provisioned.",
    )
    return add_cli_args_and_parse(parser)

class GoProCamera(Node):
    def __init__(self):
        super().__init__('gopro_node')
        # Params

        # Publisher
        self.publisher_status_ = self.create_publisher(String, '/gopro/status', 10)
        self.publisher_image_ = self.create_publisher(Image, '/gopro/image', 10)

        # Subscription
        self.subscription_cmd_ = self.create_subscription(String, '/gopro/cmd', self.cmd_callback, 10)
        
        asyncio.run(self.initialize(parse_arguments()))

    async def initialize(self, args: argparse.Namespace):
        self.get_logger().info('GoProCamera::inicialize() ok.')
        self.get_logger().info('OpenCV version: %s' % cv2.__version__)

        gopro: GoProBase | None = None

        try:
            async with WiredGoPro(args.identifier) as gopro:
                assert gopro

                await gopro.http_command.wired_usb_control(control=Params.Toggle.DISABLE)

                await gopro.http_command.set_shutter(shutter=Params.Toggle.DISABLE)
                if (await gopro.http_command.webcam_status()).data.status not in {
                    WebcamStatus.OFF,
                    WebcamStatus.IDLE,
                }:
                    self.get_logger().info("[blue]Webcam is currently on. Turning if off.")
                    assert (await gopro.http_command.webcam_stop()).ok
                    await self.wait_for_webcam_status(gopro, {WebcamStatus.OFF})

                self.get_logger().info("Starting webcam...")
                await gopro.http_command.webcam_start()
                # await self.wait_for_webcam_status(gopro, {WebcamStatus.HIGH_POWER_PREVIEW})

                # Start player
                self.camera = cv2.VideoCapture(STREAM_URL + "?overrun_nonfatal=1&fifo_size=50000000", cv2.CAP_FFMPEG)
                self.camera.set(cv2.CAP_PROP_FPS, 100)
                self.q: queue.Queue[Any] = queue.Queue()

                # Used to convert between ROS and OpenCV images
                self.br = CvBridge()

        except Exception as e:  # pylint: disable = broad-except
            self.get_logger().error(e)

        if gopro:
            await gopro.close()

        self.timer_task = self.create_timer(0.01, self.iterate)
        self.get_logger().info('GoProCamera::inicialized().')

    async def wait_for_webcam_status(gopro: GoProBase, statuses: set[WebcamStatus], timeout: int = 10) -> bool:
        """Wait for specified webcam status(es) for a given timeout

        Args:
            gopro (GoProBase): gopro to communicate with
            statuses (set[WebcamStatus]): statuses to wait for
            timeout (int): timeout in seconds. Defaults to 10.

        Returns:
            bool: True if status was received before timing out, False if timed out or received error
        """

        async def poll_for_status() -> bool:
            # Poll until status is received
            while True:
                response = (await gopro.http_command.webcam_status()).data
                if response.error != WebcamError.SUCCESS:
                    # Something bad happened
                    return False
                if response.status in statuses:
                    # We found the desired status
                    return True

        # Wait for either status or timeout
        try:
            return await asyncio.wait_for(poll_for_status(), timeout)
        except TimeoutError:
            return False
    
    def iterate(self):
        ret, frame = self.camera.read()
        self.q.put(frame)

        if ret:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV image to a ROS 2 image message
            data = self.br.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_image_.publish(data)
        else:
            self.camera.release()

    def cmd_callback(self, msg):
        self.get_logger().info('GoProCamera::CMD: %s' % (msg.data))


def main(args=None):
    rclpy.init(args=args)
    camera_node = GoProCamera()
    rclpy.spin(camera_node)

    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
