
# Project Title
<QRB ROS Audio Common>

## Overview
'qrb_ros_audio_common' is a ROS package for audio playback and capture.
support set audio parameter(volume, channels, sample_format, rate, repeat) before stream start.
Package Features:

| Features                   | description                                |
| -------------------------- | ------------------------------------------ |
| playback from file         | input wav file path to playback            |
| playback from PCM data     | specify a topic to get PCM data to playback|
| Capture to file            | input wav file path to capture             |
| Capture publish PCM data   | specify a topic to publish capture PCM data|

Capture usecase need pass audio parameter on stream open.
Playback usecase also can pass audio parameter, but audio parameter will overwrite by audio parameter on file.

This package use action server to implement playback and recording.
After action server start, use command to call server function.
ROS Action command :

| Action command     | description                                              |
| ------------------ | -------------------------------------------------------- |
| open               | open a playback/capture stream, will return stream handle|
| start              | start a opened stream                                    |
| stop               | stop a running stream                                    |
| close              | close a stream will release all resource                 |
| mute               | mute/unmute a stream                                     |

## Getting Started

<details><summary>Cross Compile with QCLINUX SDK</summary>

#### Cross Compile with QCLINUX SDK

Setup QCLINUX SDK environments:
- Reference [QRB ROS Documents: Getting Started](https://quic-qrb-ros.github.io/getting_started/environment_setup.html)

Create workspace in QCLINUX SDK environment and clone source code

```bash
mkdir -p <qirp_decompressed_workspace>/qirp-sdk/ros_ws
cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws

git clone https://github.com/quic-qrb-ros/qrb_ros_audio_common.git
git clone https://github.com/quic-qrb-ros/qrb_ros_interfaces.git
```

Build source code with QCLINUX SDK

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

Install ROS package to device

```bash
cd `<qirp_decompressed_workspace>/qirp-sdk/ros_ws/install`
tar czvf qrb_ros_audio.tar.gz lib share
scp qrb_ros_audio.tar.gz root@[ip-addr]:/opt/
ssh root@[ip-addr]
(ssh) tar -zxf /opt/qrb_ros_audio.tar.gz -C /opt/qcom/qirp-sdk/usr/
```

Login to device and run server

```bash
ssh root@[ip-addr]
(ssh) export HOME=/opt
(ssh) source /opt/qcom/qirp-sdk/qirp-setup.sh
(ssh) export ROS_DOMAIN_ID=xx
(ssh) source /usr/bin/ros_setup.bash
(ssh) ros2 launch qrb_ros_audio_common component.launch.py
```

</details>

<details open><summary>Native Build on Ubuntu</summary>

#### Native Build on Ubuntu

Prerequisites

- ROS 2: [Install ROS2 on Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

Create workspace and clone source code from GitHub:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/quic-qrb-ros/qrb_ros_audio_common.git
git clone https://github.com/quic-qrb-ros/qrb_ros_interfaces.git
```

install compile depency
```bash
apt update
apt-get install libpulse-dev libsndfile1-dev
```

Build source code

```bash
cd ~/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build
```

Run Server

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch qrb_ros_audio_common component.launch.py
```

</details>


### Run playback/Capture
After Server start, can use below commands to playback/capture.
Two action server will running, action_name define on component.launch.py.
By default, 
Playback will be 'ros_audio_playback'
Capture will be 'ros_audio_capture'

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


## Contributing

We would love to have you as a part of the QRB ROS community. Whether you are helping us fix bugs, proposing new features, improving our documentation, or spreading the word, please refer to our [contribution guidelines](./CONTRIBUTING.md) and [code of conduct](./CODE_OF_CONDUCT.md).

- Bug report: If you see an error message or encounter failures, please create a [bug report](../../issues)
- Feature Request: If you have an idea or if there is a capability that is missing and would make development easier and more robust, please submit a [feature request](../../issues)

<Update link with template>


## Documentation
Please visit [QRB ROS Documentation](https://quic-qrb-ros.github.io/) for more details.


## Authors

* **Ronghui Zhu** - *Initial work* - [quic-ronghuiz](https://github.com/quic-ronghuiz)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.


## License

Project is licensed under the [BSD-3-clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](./LICENSE) for the full license text.

