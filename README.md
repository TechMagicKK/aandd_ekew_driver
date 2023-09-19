# aandd_ekew_driver_py

## install python module "serial"
```.sh
sudo rosdep init
rosdep update
rosdep install --from-paths aandd_ekew_driver_py -y --ignore-src
```

## prepare for interfaces (option)
this node uses weight_scale_interfaces
```.sh
cd ~/dev_ws/src
git clone https://github.com/TechMagicKK/weight_scale_interfaces.git
cd ~/dev_ws
colcon build --cmake-clean-first --symlink-install --packages-select weight_scale_interfaces
. install/local_setup.zsh
```

## rs-232c device setting
```.sh
cd ~/dev_ws/aandd_ekew_driver_py/launch
edit bringup.launch.py for change params (port, baudrate, rate)
```

## build
```.sh
cd ~/dev_ws
colcon build --cmake-clean-first --symlink-install --packages-select aandd_ekew_driver_py
. install/local_setup.zsh
```

## launch node
```.sh
ros2 launch aandd_ekew_driver_py bringup.launch.py
```

## launch node with fake device for test
```.sh
ros2 launch aandd_ekew_driver_py bringup.launch.py use_fake:=True
```

## change publish rate to 1.2[Hz]
```.sh
ros2 param set /aandd_ekew_node rate 1.2
```

