# ROS2 package to publish webcam or video as topic with essential profiling messages

## Install

In your working directory `path/to/wd`, create an `src` directory:

```bash
cd path/to/wd
mkdir -p path/to/wd/src
```

Clone this repo

```bash
cd path/to/wd/src
git clone https://github.com/Shiritai/webcam-video-streamer.git
```

and then build this repo

```bash
cd path/to/wd
colcon build --packages-select webcam_video_streamer
```

## Usage

### Publisher

Source the package we just built before using it.

```bash
# get path of current running shell
_CURRENT_SHELL=$(readlink /proc/$$/exe)

# source setup script according to this shell
source path/to/wd/install/local_setup."${_CURRENT_SHELL##*/}"
```

Then we can publish

* **webcam** using the following command
    ```bash
    ros2 run webcam_video_streamer video_node --webcam --fps SOME_INT_VALUE
    ```
* **video** using the following command
    ```bash
    ros2 run webcam_video_streamer video_node --file path/to/video/file
    ```

:::info
Note that the published topic is named by the type of streaming (video: `/video_image`, webcam: `/webcam_image`).
:::

For more options, please check:

```bash
ros2 run webcam_video_streamer video_node -h
```

and see help message:

```bash
usage: video_node [-h] (--file FILE | --webcam) [--fps FPS] [--loop] [--rescale RESCALE] [--q_len Q_LEN]

ROS2 streamer that create topic with name according to stream type. (video: /video_image, webcam: /webcam_image)

options:
  -h, --help         show this help message and exit
  --file FILE        Use video with given video file path
  --webcam           Use webcam
  --fps FPS          Streaming FPS (required in webcam mode), this implies the play speed of video (fps > source fps means speed up, vice versa). Note that this is not the accurate FPS, which determines the timer interval of
                     reading source frames.
  --loop             Loop the video after it ends, only available in video streaming mode
  --rescale RESCALE  Rescale width and height of the streamer
  --q_len Q_LEN      Publisher queue length, will use 1 if not set
```

### Subscriber

Run the following command to see the streaming content

```bash
ros2 run rqt_image_view rqt_image_view
```

## Profiling

You can check ros2 topic to see current stream header:

> `TOPIC` can be `/video_image` (video) or `/webcam_image` (webcam)

* Frame ID
    ```bash
    ros2 topic echo TOPIC | grep frame_id
    ```
* Time stamp
    ```bash
    ros2 topic echo TOPIC | grep sec
    ```

## Note

Feel free to open issue or make PR if you have any idea.

## Reference

* [my_camera](https://github.com/pratikPhadte/my_camera/tree/main)
* [ros2_video_streamer](https://github.com/klintan/ros2_video_streamer/tree/master)
