# SensorLog ROS Bridge

This software is a ROS bridge for the SensorLog iPhone app. By configuring SensorLog as a UDP client and setting it to send sensor data in JSON format, this program enables integration with ROS.

Apple store Link : [SensorLog app](https://apps.apple.com/us/app/sensorlog/id388014573)

## Installation

To use the SensorLog ROS Bridge, follow these steps:

1. Install the SensorLog app on your iPhone from the Apple App Store.
2. Configure SensorLog as a UDP client and set it to send sensor data in JSON format.
3. Clone this repository to your ROS workspace:

```bash
git clone https://github.com/gusugusu1018/sensorlog_bridge
```

Build the ROS package:

```bash
cd YOUR_CATKIN_WORKSPACE
catkin build
```

## Usage

1. Configure SensorLog to send sensor data to the IP address and port specified in the launch file.

2. Launch the SensorLog ROS Bridge node:

```bash
roslaunch sensorlog_bridge sensorlog_bridge.launch
```

Note: If you do not want to launch RViz, you can use the gui:=false argument to disable it.

```bash
roslaunch sensorlog_bridge sensorlog_bridge.launch gui:=false
```

3. The SensorLog ROS Bridge will receive the sensor data from SensorLog and publish it to ROS topics.

## UDP Data Format

The UDP raw data format for SensorLog is as follows:

```json
{
  "loggingSample": "30554",
  "loggingTime": "2023-06-05T23:37:35.646+09:00",
  "motionAttitudeReferenceFrame": "XArbitraryCorrectedZVertical",
  "motionGravityX": "0.003247",
  "motionGravityY": "-0.018757",
  "motionGravityZ": "-0.999819",
  "motionHeading": "-1.000000",
  "motionMagneticFieldCalibrationAccuracy": "2.000000",
  "motionMagneticFieldX": "-32.293961",
  "motionMagneticFieldY": "3.802150",
  "motionMagneticFieldZ": "-32.477192",
  "motionPitch": "0.018758",
  "motionQuaternionW": "0.999122",
  "motionQuaternionX": "0.009305",
  "motionQuaternionY": "0.002005",
  "motionQuaternionZ": "0.040804",
  "motionRoll": "0.003247",
  "motionRotationRateX": "0.001327",
  "motionRotationRateY": "-0.000258",
  "motionRotationRateZ": "-0.002521",
  "motionTimestamp_sinceReboot": "142714.045678",
  "motionUserAccelerationX": "-0.000042",
  "motionUserAccelerationY": "-0.000882",
  "motionUserAccelerationZ": "-0.002546",
  "motionYaw": "0.081604"
}
```

Please note that the provided data is an example and may vary based on the specific sensor readings and configurations.

## Supported ROS Distribution

The SensorLog ROS Bridge currently supports the ROS Noetic distribution.

## Contributions

Contributions to the SensorLog ROS Bridge are welcome! If you find any issues or have suggestions for improvements, please create a new issue or submit a pull request on the GitHub repository.

## License

This software is released under the MIT License.
