A ROS package for JR3 force/torque sensors with (so far) very basic functionality. Only one very rudimentary node is implemented that publishes F/T data of one sensor as a 6-element array.

Published topics:  
- ``jr3ft`` (std_msgs/Float32MultiArray)

Parameters:  
- ``rosjr3/rate`` (std::vector<int>, default: 1000): Sampling rate [Hz]
- ``rosjr3/full_scales`` (int, default: [100, 100, 200, 5, 5, 5]): Sensor's fulls scales [N, Nm]
- ``num_filter`` (int, default: 0): Filter number (0-6)
- ``num_sensor`` (int, default: 0): Sensor number (0, 1), if supported by DSP board

The driver used for interfacing the JR3 force/torque sensor's DSP board (PCI) can be found here: https://github.com/roboticslab-uc3m/jr3pci-linux
