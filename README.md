# aandd_ekew_driver

## prepare for python module
```.sh
pip install pyserial
```

## rs-232c device setting
```.sh
cd ~/dev_ws/aandd_ekew_driver/launch
edit bringup.launch.py for change params (device, rate)
```

## build
```.sh
cd ~/dev_ws
colcon build --cmake-clean-first --symlink-install --packages-select aandd_ekew_driver
. install/local_setup.zsh
```

## start node
```.sh
ros2 launch aandd_ekew_driver bringup.launch.py
```
## start test
```.sh
ros2 run aandd_ekew_driver aandd_ekew_driver_test
```
