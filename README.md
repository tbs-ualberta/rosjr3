A ROS package for JR3 force/torque sensors with (so far) very basic functionality
- ``rosjr3``: Node publishing F/T data of one sensor as ``WrenchStamped``-message
- ``rosjr3two``: Node publishing F/T data of two sensors as  ``WrenchStamped``-messages

#### Published topics:
- ``rosjr3``: ``jr3ft`` (geometry_msgs/WrenchStamped)
- ``rosjr3two``:
    - ``jr3ft0`` (geometry_msgs/WrenchStamped)
    - ``jr3ft1`` (geometry_msgs/WrenchStamped)

#### Parameters:
- ``rosjr3`` & ``rosjr3two``:
    - ``rate`` (int, default: 1000): Sampling rate [Hz]
    - ``full_scales`` (std::vector<int>, default: [100, 100, 200, 5, 5, 5]): Sensor's full scales [N, N, N, Nm, Nm, Nm]
    - ``num_filter`` (int, default: 0): Filter number (0-6)
- ``rosjr3`` only:
    - ``num_sensor`` (int, default: 0): Sensor number (0, 1), if supported by DSP board


The driver used for interfacing the JR3 force/torque sensor's DSP board (PCI) can be found here: https://github.com/roboticslab-uc3m/jr3pci-linux
