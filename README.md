# [Verti-Wheeler](https://cs.gmu.edu/~xiao/Research/Verti-Wheelers/): Verti 4 Wheelers(V4W) and Verti 6 Wheelers(V6W) Installation.

![Suite image](https://github.com/RobotiXX/Verti-Wheelers/blob/main/Verti%20Wheelers.jpg)

### Dependencies :

> ROS Noetic.

> PyTorch.

> Torchvision

> Arduino [Arduino Mega 2560] (https://docs.arduino.cc/software/ide-v1/tutorials/Linux).

> Python 3.8.10.

> RGBD camera driver [Azure Kinect Camera](https://github.com/microsoft/Azure_Kinect_ROS_Driver).

### Create a workspace and clone sources

```
mkdir -p catkin_ws/src; cd catkin_ws/src; catkin_init_workspace
git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver
git clone https://github.com/microsoft/Azure-Kinect-Sensor-SDK
git clone https://github.com/RobotiXX/Verti-Wheelers.git
```

*Install dependencies with* `rosdep` : 
```
cd catkin_ws; rosdep install --from-paths . --ignore-src --rosdistro=noetic
```

### Build and source

```
cd catkin_ws; catkin build; source devel/setup.bash
```
### To Launch the Wheelers ROS Network

```
In terminal 1: roslaunch navstack_pub runCAR.launch
```

### To Launch the Three Different Algorithms

```
Open_loop launch:
In terminal 2: roslaunch navstack_pub open_loop.launch

Rule_based launch:
In terminal 2: roslaunch navstack_pub rule_based.launch

Neural Network Behavior cloning launch:
In terminal 2: cd catkin_ws/src/navstack_pub/src; python3 action_generator.py
```
### Sample trained model can be found here

> [Sample trained Model](https://github.com/RobotiXX/Verti-Wheelers/blob/main/navstack_pub/src/model_big_good.pt)


