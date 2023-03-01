# [Verti-Wheelers](https://cs.gmu.edu/~xiao/Research/Verti-Wheelers/): Verti 4 Wheeler(V4W) and Verti 6 Wheeler(V6W) Installation.

![Suite image](https://github.com/RobotiXX/Verti-Wheelers/blob/main/Verti%20Wheelers.jpg)

### Dependencies :

> ROS Noetic.

> PyTorch.

> Torchvision

> Arduino [Arduino Mega 2560](https://docs.arduino.cc/software/ide-v1/tutorials/Linux).

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

*Install Azure Kinect Sensor SDK on Ubuntu 20.04+ (Current Sensor SDK only supports for Ubuntu 18.04)* :
> 1. Downlaod the [k4a-tools 1.4.1amd64.deb](https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/) file.
> 2. Download the [libk4a1.4 and libk4a1.4-dev](https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/) files.
> 3. Install SDK.
> 4. Create a new file: sudo gedit /etc/udev/rules.d/99-k4a.rules, add the contents as below:
```
# Bus 002 Device 116: ID 045e:097a Microsoft Corp.  - Generic Superspeed USB Hub
# Bus 001 Device 015: ID 045e:097b Microsoft Corp.  - Generic USB Hub
# Bus 002 Device 118: ID 045e:097c Microsoft Corp.  - Azure Kinect Depth Camera
# Bus 002 Device 117: ID 045e:097d Microsoft Corp.  - Azure Kinect 4K Camera
# Bus 001 Device 016: ID 045e:097e Microsoft Corp.  - Azure Kinect Microphone Array

BUS!="usb", ACTION!="add", SUBSYSTEM!=="usb_device", GOTO="k4a_logic_rules_end"

ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097a", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097b", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097c", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097d", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097e", MODE="0666", GROUP="plugdev"

LABEL="k4a_logic_rules_end"
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
### Pytorch implementaion of the CNN from End to End Learning for Verti Wheelers


> [Sample trained Model](https://github.com/RobotiXX/Verti-Wheelers/blob/main/navstack_pub/src/model_big_good.pt)

> All the sample datasets can be found here [Dataverse](https://dataverse.orc.gmu.edu/dataset.xhtml?persistentId=doi:10.13021/orc2020/QSN50Q)


### Verti Wheelers Control for PS4 Controller
![Suite image](https://github.com/RobotiXX/Verti-Wheelers/blob/main/Joystick%20control.jpg)
