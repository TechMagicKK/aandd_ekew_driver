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

## fix the port name and add privilege(option)

1. check vendor id and product id.

   1. plug/unplug the usb cable in order to find the vendor id and product id of your device with `lsusb` command. For example, you can see the following text.
      ```shell
      Bus 004 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
      Bus 003 Device 004: ID 04f2:b7c0 Chicony Electronics Co., Ltd Integrated Camera
      Bus 003 Device 003: ID 06cb:00f9 Synaptics, Inc. 
      Bus 003 Device 005: ID 8087:0033 Intel Corp. 
      Bus 003 Device 007: ID 0584:b020 RATOC System, Inc. REX-USB60F
      Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
      Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
      Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
      ```

   2. hopefully you can find the new line. the example is the following.
      ```
      Bus 003 Device 007: ID 0584:b020 RATOC System, Inc. REX-USB60F
      ```

      The vendor id is `0584`, and product id is `b020`. **NOTE: This is example, the vendor id and product id depend on your device. It may be difference in your device.**

2. change  value in `aandd_ekew_driver_py/rules/99-weight-scale.rules` with your product id and product  id.

   1. open rule file
      ```shell
      cd ros_ws/src/aandd_ekew_driver_py/rules
      vim 99-weight-scale.rules
      ```

   2. change values for `idVendor` and `idProduct`. If your vendor id is `0584` and product id is `b020`, edit it as below.
      ```
      SUBSYSTEM=="tty", ATTRS{idVendor}=="0584", ATTRS{idProduct}=="b020", SYMLINK+="weightscale", MODE="0666"
      ```

3. copy the rule file into `/etc/udev/rules.d/`
   ```shell
   sudo cp 99-weight-scale.rules /etc/udev/rules.d
   ```

4. enable your rule
   ```shell
   sudo udevadm control --reload
   ```

5. unplug and plug your usb cable. If the change is successful, you can find `/dev/weightscale`.

6. change `port` to `weightscale` in `bringup.launch.py`.
   ```
   - port = launch.substitutions.LaunchConfiguration('port', default='/dev/ttyUSB0')
   + port = launch.substitutions.LaunchConfiguration('port', default='/dev/weightscale')
   ```

   
