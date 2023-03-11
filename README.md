# aandd_ekew_driver

## prepare for python module
```.sh
pip install pyserial
```

## prepare for interfaces
this node uses weight_scale_interfaces and weight_scale_driver
```.sh
cd ~/dev_ws/src
git clone https://github.com/TechMagicKK/weight_scale_interfaces.git
git clone https://github.com/TechMagicKK/weight_scale_driver.git
cd ~/dev_ws
colcon build --cmake-clean-first --symlink-install --packages-select weight_scale_interfaces
colcon build --cmake-clean-first --symlink-install --packages-select weight_scale_driver
. install/local_setup.zsh
```

## rs-232c device setting
```.sh
cd ~/dev_ws/aandd_ekew_driver/launch
edit bringup.launch.py for change params (device, baudrate, rate)
```

## build
```.sh
cd ~/dev_ws
colcon build --cmake-clean-first --symlink-install --packages-select aandd_ekew_driver
. install/local_setup.zsh
```

## launch node
```.sh
ros2 launch aandd_ekew_driver bringup.launch.py
```

## launch node with fake device for test
```.sh
ros2 launch aandd_ekew_driver bringup.launch.py use_fake:=True
```

## change publish rate to 1.2[Hz]
```.sh
ros2 param set /aandd_ekew_node rate 1.2
```

## run test
```.sh
ros2 run weight_scale_driver weight_scale_test --ros-args -p device_node:=/aandd_ekew_node -p timeout_sec:=2.0
```
