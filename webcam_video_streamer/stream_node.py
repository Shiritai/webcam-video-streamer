import argparse
from enum import Enum
import os
import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node class from ROS 2
import cv2  # Import OpenCV for computer vision tasks
from cv_bridge import CvBridge  # Import CvBridge to convert between ROS and OpenCV images
from sensor_msgs.msg import Image  # Import the Image message type from sensor_msgs

class StreamType(Enum):
    VIDEO = 'video'
    WEBCAM = 'webcam'

    @property
    def node(self):
        return f"{self.value}_node"

    @property
    def topic(self):
        return f"{self.value}_image"


class StreamNode(Node):
    def __init__(self,
                 file: str | None, # if is None, means using webcam
                 fps: int | None,
                 loop: bool | None,
                 rescale: float | None,
                 q_len: int | None):
        # Initialize the Node with the name according to stream type
        stream_type = StreamType.VIDEO if file is not None else StreamType.WEBCAM
        super().__init__(stream_type.node)

        # Create a publisher for the Image topic with queue size of 1
        if q_len is None:
            q_len = 1
        self.publisher_ = self.create_publisher(Image, stream_type.topic, q_len)

        # Open the video file or webcam
        if stream_type == StreamType.VIDEO and not os.path.exists(file):
            raise ValueError(f'File {file} does not exist')
        self.cap = cv2.VideoCapture(file if stream_type == StreamType.VIDEO else 0)

        # Create a timer to call stream_callback every (1 / fps) seconds
        if fps is None:
            fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.timer = self.create_timer(1 / fps, self.stream_callback)

        # Initialize the CvBridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        
        # inner parameters
        self.loop = loop
        self.video_size = None
        default_size = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                        int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if rescale is not None:
            self.video_size = tuple(int(a * rescale) for a in default_size)

        # print configurations
        stream_config = f"stream: {file if stream_type == StreamType.VIDEO else StreamType.WEBCAM}"
        self.get_logger().info(f'{type(self).__name__} is up with config [{stream_config}, fps: {fps}, loop: {self.loop}, video_size: {self.video_size if self.video_size is not None else default_size}]')

    def stream_callback(self):
        if self.cap.isOpened():
            # Capture a frame from the video
            ret, frame = self.cap.read()
            if not ret and self.loop:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0) # loop playing
                ret, frame = self.cap.read()
            
            if ret: # Check if the frame was captured successfully
                # resize and interpolate if needed
                if self.video_size is not None:
                    frame = cv2.resize(frame,
                                       self.video_size,
                                       interpolation=cv2.INTER_NEAREST)

                # Convert the OpenCV image to a ROS Image message
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                # set frame id & timestamp for profiling
                msg.header.frame_id = str(int(self.cap.get(cv2.CAP_PROP_POS_FRAMES)))
                msg.header.stamp = self.get_clock().now().to_msg()
                
                # Publish the Image message
                self.publisher_.publish(msg)
            else:
                self.get_logger().info(f'Stream ends')
                self.self_destroy()
        else:
            self.self_destroy()
            error_msg = 'OpenCV video is closed'
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)
    
    def self_destroy(self):
        self.timer.cancel()  # cancel timer callback
        self.cap.release()  # Release the video
        self.destroy_node()  # Destroy this node

def main(args=None):
    parser = argparse.ArgumentParser(description=f'ROS2 streamer that create topic with name according to stream type. ({", ".join([f"{t.value}: /{t.topic}" for t in StreamType])})')
    # required parameters
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--file', type=str, help='Use video with given video file path')
    group.add_argument('--webcam', help='Use webcam', action='store_true')
    # optional parameters
    parser.add_argument('--fps', type=int, required=False, help='Streaming FPS (required in webcam mode), this implies the play speed of video (fps > source fps means speed up, vice versa). Note that this is not the accurate FPS, which determines the timer interval of reading source frames.')
    parser.add_argument('--loop', help='Loop the video after it ends, only available in video streaming mode', action='store_true')
    parser.add_argument('--rescale', type=float, required=False, help='Rescale width and height of the streamer')
    parser.add_argument('--q_len', type=int, required=False, help='Publisher queue length, will use 1 if not set')
    cli_args = parser.parse_args()

    # check whether cli_args is valid
    if cli_args.webcam:
        if cli_args.loop:
            raise ValueError('Webcam is not loopable, --loop flag only applies when is video streaming mode')
        if cli_args.fps is None:
            raise ValueError('FPS is required for Webcam streaming')
    
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    # Create an instance of the StreamNode
    node = StreamNode(cli_args.file,
                     cli_args.fps,
                     cli_args.loop,
                     cli_args.rescale,
                     cli_args.q_len)
    try:
        rclpy.spin(node)  # Spin the node to keep it alive and processing callbacks
    except KeyboardInterrupt:
        pass  # Allow the user to exit with Ctrl+C
    finally:
        node.self_destroy()  # self destroy if needed
        rclpy.shutdown()  # Shut down ROS 2 Python client library

if __name__ == '__main__':
    main()

