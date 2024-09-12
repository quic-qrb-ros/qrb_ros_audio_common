# QRB ROS Audio Common

qrb_ros_audio_common is a package to provide two ros action for playback and capture.

## Overview

Provide two ROS node for playback and capture.
Two node support set audio parameter(volume, channels, sample_format, rate, repeat) to specify stream format.
Provide interface(ros2 communication) to Transfer audio data between ROS node.
For Playback, can receive PCM data or file path from other ROS node, then playback to Speaker 
For Capture, can get PCM data from Mic, then send PCM data to other ROS node or save to file

## Build

Currently, we only support use QCLINUX to build

1. Setup environments follow this document 's [Set up the cross-compile environment.](https://docs.qualcomm.com/bundle/publicresource/topics/80-65220-2/develop-your-first-application_6.html?product=1601111740013072&facet=Qualcomm%20Intelligent%20Robotics%20(QIRP)%20Product%20SDK&state=releasecandidate) part

2. Create `ros_ws` directory in `<qirp_decompressed_workspace>/qirp-sdk/`

3. Clone this repository under `<qirp_decompressed_workspace>/qirp-sdk/ros_ws`
     ```bash
     git clone https://github.qualcomm.com/QUIC-QRB-ROS/qrb_ros_audio_common.git
     ```
4. Build this project
     ```bash
     export AMENT_PREFIX_PATH="${OECORE_TARGET_SYSROOT}/usr;${OECORE_NATIVE_SYSROOT}/usr"
     export PYTHONPATH=${PYTHONPATH}:${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages

     colcon build --merge-install --cmake-args \
       -DPython3_ROOT_DIR=${OECORE_TARGET_SYSROOT}/usr \
       -DPython3_NumPy_INCLUDE_DIR=${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages/numpy/core/include \
       -DPYTHON_SOABI=cpython-310-aarch64-linux-gnu -DCMAKE_STAGING_PREFIX=$(pwd)/install \
       -DCMAKE_PREFIX_PATH=$(pwd)/install/share \
       -DBUILD_TESTING=OFF
     ```
5. Push to the device & Install
     ```bash
     cd `<qirp_decompressed_workspace>/qirp-sdk/ros_ws/install`
     tar czvf qrb_ros_audio.tar.gz lib share
     scp qrb_ros_audio.tar.gz root@[ip-addr]:/opt/
     ssh root@[ip-addr]
     (ssh) tar -zxf /opt/qrb_ros_audio.tar.gz -C /opt/qcom/qirp-sdk/usr/
     ```

## Run

This package supports running it from ros launch file or directly running it from command.

a.Run with launch file

1. Source this file to set up the environment on your device:
    ```bash
    ssh root@[ip-addr]
    (ssh) export HOME=/opt
    (ssh) source /opt/qcom/qirp-sdk/qirp-setup.sh
    (ssh) export ROS_DOMAIN_ID=xx
    (ssh) source /usr/bin/ros_setup.bash
    ```

2. Use this launch file to run this package
    ```bash
    (ssh) ros2 launch qrb_ros_audio_common component.launch.py
    ```

b. Run with command
1. Playback:
    ```bash
    ros2 run qrb_ros_audio_common audio_common_node_exec --ros-args -p Stream_type:="playback" -p action_name:="ros_audio_playback" -p topic_name:="ros_audio_data"
    ```

2. Capture:
    ```bash
    ros2 run qrb_ros_audio_common audio_common_node_exec --ros-args -p Stream_type:="capture" -p action_name:="ros_audio_capture" -p topic_name:="ros_audio_data"
    ```
After launch with component.launch.py, will start two action ros_audio_playback and ros_audio_capture.
For audio stream operate, refer to qrb_ros_audio_common/qrb_ros_audio_common_msgs.

open stream, for record need pass audio_info, for playback audio_info only need pass on pub PCM case.

a. Playback/Capture from/to file.
```bash
ros2 action send_goal <action_name> qrb_ros_audio_common_msgs/action/AudioCommon "{
audio_info: {
    channels: 2,
    sample_rate: 48000,
    sample_format: 16
  },
  command: 'open',
  volume: 100,
  file_path: '/tmp/xxx.wav',
  repeat: 1
}"
```
b. Playback/Capture from/to a topic.
```
ros2 action send_goal <action_name> qrb_ros_audio_common_msgs/action/AudioCommon "{
audio_info: {
    channels: 2,
    sample_rate: 48000,
    sample_format: 16
  },
    command: 'open',
    volume: 100,
    topic_name: "pcm_topic",
    pub_pcm: true,
}"
```
open command will return a stream handle.

stop/mute/stop/close stream
```
ros2 action send_goal <action_name> qrb_ros_audio_common_msgs/action/AudioCommon "{
  command: "start",
  stream_handle: <stream_handle>,
}"

ros2 action send_goal <action_name> qrb_ros_audio_common_msgs/action/AudioCommon "{
  command: "mute",
  mute: "true",
  stream_handle: <stream_handle>,
}"

ros2 action send_goal <action_name> qrb_ros_audio_common_msgs/action/AudioCommon "{
  command: "stop",
  stream_handle: <stream_handle>,
}"

ros2 action send_goal <action_name> qrb_ros_audio_common_msgs/action/AudioCommon "{
  command: "close",
  stream_handle: <stream_handle>,
}"
```

## Packages

Will update in the future.

## Resources


## Contributions

Thanks for your interest in contributing to qrb_ros_audio_common! Please read our [Contributions Page](CONTRIBUTING.md) for more information on contributing features or bug fixes. We look forward to your participation!

## License

qrb_ros_audio_common is licensed under the BSD-3-clause "New" or "Revised" License. 

Check out the [LICENSE](LICENSE) for more details.
