# my_jackal
ROS1 package for Choreonoid.

## 1. Install
### teleop-twist-keyboard
```
sudo apt install ros-noetic-teleop-twist-keyboard
```

### my_jackal
```
cd ~/catkin_ws/src
git clone https://github.com/k38-suzuki/my_jackal.git
```

## 2. Build
```
cd ~/catkin_ws
catkin build -j8
source ~/catkin_ws/devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 3. Run
### Run (Jackal)
```
roslaunch my_jackal jackal.launch
```

### Run (Husky)
```
roslaunch my_jackal husky.launch
```

## 4. References
- https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/
- https://clearpathrobotics.com/husky-a300-unmanned-ground-vehicle-robot/
- http://www.nihonbinary.co.jp/Products/Robot/CLEARPATH_JACKAL.html
- http://www.nihonbinary.co.jp/Products/Robot/CLEARPATH_HUSKY.html
