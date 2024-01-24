[_qb SoftHand Industry_](https://qbrobotics.com/product/qb-softhand-industry/) is an **adaptable yet robust robotic hand** gripper for collaborative robotic arms and for humanoid robots. It is in scale 1:1 with the average human hand and it is similarily composed by 19 completely desensorized phalanges and a flat palm.

The specific system of tendons connected to just a single motor provides the mechanical adaptability of the grasp to the shape and softness of any object without damaging it, i.e. exploiting the intrinsic capability of soft robots to continuously deform in a huge variety of possible configurations through interaction with external items.

In addition, the single actuator fixed directly on the back of the hand barely affects the total encumbrance and weight (only 480 grams) of the device.

## Table of Contents
1. [Installation](#markdown-header-installation)
   1. [Requirements](#markdown-header-requirements)
   1. [Ubuntu Packages](#markdown-header-ubuntu-packages)
   1. [Sources](#markdown-header-sources)
   1. [Device Setup](#markdown-header-device-setup)
1. [Usage](#markdown-header-usage)
   1. [Launch File](#markdown-header-launch-file)
   1. [GUI Control](#markdown-header-1.-gui-control)
   1. [API Control](#markdown-header-2.-api-control)
1. [Support, Bugs and Contribution](#markdown-header-support)
1. [Purchase](#markdown-header-purchase)


## Installation
### Requirements
If you have never set it up, you probably need to add your linux user to the `dialout` group to grant right access to the serial port resources. To do so, just open a terminal and execute the following command:
```
sudo gpasswd -a <user_name> dialout
```
where you need to replace the `<user_name>` with your current linux username.

_Note: don't forget to logout or reboot._

### Ubuntu Packages
If you prefer to leave your colcon workspace as it is, you can simply install all the ROS packages from the Ubuntu official repositories:
```
sudo apt update
sudo apt install ros-humble-qb-softhand-industry
```

### Sources
>Since you are interested in the ROS2 interfaces for our devices, it is assumed that you are familiar at least with the very basics of the ROS2 environment. If not, it might be useful to spend some of your time with [ROS2](http://docs.ros.org/en/humble/Tutorials.html) tutorials. After that, don't forget to come back here and start having fun with our Nodes.

Install the _qb SoftHand Industry_ packages for a ROS user is straightforward. Nonetheless the following are the detailed steps which should be easy to understand even for ROS beginners:

1. Clone the `qb_softhand_industry` packages to your Catkin Workspace, e.g. `~/colcon_ws`:
   ```
   cd `~/colcon_ws/src`
   git clone https://bitbucket.org/qbrobotics/qbshin-ros.git
   cd qbshin-ros
   git checkout production-humble
   ```
1. Install ROS dependencies using `rosdep`:
   ```
   cd `~/colcon_ws`
   rosdep install --from-paths src --ignore-src -r -y
   ```

1. Compile the packages using `colcon`:
   ```
   colcon build
   ```
   **Note:** depending on your ROS installation, you may need some extra packages to properly compile the code. Please use `sudo apt install ros-humble-<pkg-name>` for each package missing.

1. If you were not familiar with ROS you should be happy now: everything is done! Nonetheless, if you encounter some troubles during the compilation, feel free to ask for support on [our Bitbucket](https://bitbucket.org/account/user/qbrobotics/projects/ROS).

### Device Setup
To connect a _qb SoftHand Industry_ to your system is **read carefully** the [manual](https://qbrobotics.com/download/qb-softhand-industry-downloads/) to understand all the requirements and advices. The communication can happen via Ethernet, ( default address 192.168.1.110) in this case you have to configure your wired connection with an IP of the same family.
Alternatively USB serial connection can be established with the device (Mini-USB type B cable for Elmo driver is needed).

## Usage

### Launch file 

Once the device is physically connected to your system, you can launch the device node with the following command line:  

```
ros2 launch qb_softhand_industry_ros2_control qb_softhand_industry_display.launch.py use_rviz:=true 
```

###### The arguments explained
- `use_rviz [false]`: Choose whether or not to use rviz. If enabled you should see a virtual hand on screen performing a similar behavior.

   >Be aware that the _qb SoftHand_ is desensorized and therefore it is not possible to know exactly the position of each finger: the screen visualization is just the result of an estimation of the closure value and may differ from the real configuration of your _qb SoftHand_ (e.g. when grasping an object).

###### Additional arguments
- `ip_address [192.168.1.110]`: The IP address of the ELMO driver.
- `flange_angle [0]`: The angle of the configurable flange (in radians).
- `use_only_ethernet [true]`: Choose whether or not to look SoftHand Industry on serial ports.
- `serial_port_name []`: Use explicit serial port name, leave empty to search for serial ports.

###### Command the device
If everything went well, the launch file should have loaded and started the ros2 controller _forward position controller_.
In order to move the hand, commands have to be published on the topic /forward_position_controller/commands. This can be done with the following command:

```
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "layout:
  dim: []
  data_offset: 0
data: [VALUE]" -1
```
VALUE must be replaced with a number between 0 (_fully open_) and 1 (_fully close_).


#### 1. GUI Control
It is possible to control the device with the ROS2 tool **rqt_gui**. To do this the packages rqt_joint_trajectory_controller and rqt_controller_manager are needed, they can be installed with the command:

```
 sudo apt install ros-humble-rqt-joint-trajectory-controller ros-humble-rqt-controller-manager 
```

After the launch file has been executed, the user can run the *rqt_gui* tool (command ```ros2 run rqt_gui rqt_gui```), then open _Plugins_->_Robot Tools_->_Controller Manager_ and _Plugins_->_Robot Tools_->_Joint Trajectory Controller_.

Two GUIs should appear on the *rqt_gui* window, on the _Controller Manager_ GUI the user can switch controllers (deactivate one controller and activate the other). 

If we want to start with the synergy_trajectory_controller from launch command, just add the argument *robot_controller* like this:

```
ros2 launch qb_softhand_industry_ros2_control qb_softhand_industry_display.launch.py robot_controller:=synergy_trajectory_controller
```

Choosing the **synergy_trajectory_controller** allow the user to use the _Joint Trajectory Controller_ GUI to send commands to the device. 
That GUI consists in a screen with two empty dropdown menus, a red enable button below them, and a _speed scaling_ slider at the bottom:
1. Select the _Controller Manager_ namespace from the left menu, e.g. `/<robot_namespace>/control/controller_manager` (where `<robot_namespace>` is an additional argument of the launch file needed with several devices). This enables the right menu which provides all the controllers available for the connected device.
1. Select the _shin synergy trajectory controller_ from the second dropdown menu and enable it through the circular button.
1. A single slider will appear in the GUI to control the closure of the hand, which ranges from `0` (hand completely open) to `1` (hand completely closed). Move the slider to perform a complete grasp or to partially open/close the _qb SoftHand Industry_. You can also vary the speed through the bottom _speed scaling_ slider if you like a faster/slower motion. No other timing constraints can be set in this mode.

#### 2. API Control
If you need a complex (i.e. real) control application, e.g. the _qb SoftHand Industry_ is mounted on a robot which uses computer vision aid to grasp objects, the previous two control modes don't really help much. What we provide for real applications is the full ROS libraries to manage and control the _qb SoftHand Industry_.

You can refer to an example of the api available at [API Shin Example repository](https://bitbucket.org/qbrobotics/api-test-shin-v7/src/main/) .

>Our recommendation is to use as much as possible our resources, classes and macros to help you while developing your application. Don't reinvent the wheel!

At last, if you come up with a something useful for the whole community, it will be amazing if you propose your improvement with a Pull Request in the package of interest on [our Bitbucket](https://bitbucket.org/account/user/qbrobotics/projects/ROS).

## Support, Bugs and Contribution
Since we are not only focused on this project it might happen that you encounter some trouble once in a while. Maybe we have just forget to think about your specific use case or we have not seen a terrible bug inside our code. In such a case, we are really sorry for the inconvenience and we will provide any support you need.

To help you in the best way we can, we are asking you to do the most suitable of the following steps:

1. It is the first time you are holding a _qb_ product, or the first time you are using ROS, or even both: it is always a pleasure for us to solve your problems, but please consider first to read again the instructions above and the ROS tutorials. If you have ROS related questions the right place to ask is [ROS Answers](http://answers.ros.org/questions/).
1. You are a beginner user stuck on something you completely don't know how to solve or you are experiencing unexpected behaviour: feel free to contact us at [support+ros at qbrobotics.com](support+ros@qbrobotics.com), you will receive the specific support you need as fast as we can handle it.
1. You are quite an expert user, everything has always worked fine, but now you have founded something strange and you don't know how to fix it: we will be glad if you open an Issue in the package of interest on [our Bitbucket](https://bitbucket.org/account/user/qbrobotics/projects/ROS).
1. You are definitely an expert user, you have found a bug in our code and you have also correct it: it will be amazing if you open a Pull Request in the package of interest on [our Bitbucket](https://bitbucket.org/account/user/qbrobotics/projects/ROS); we will merge it as soon as possible.
1. You are comfortable with _qbrobotics®_ products but you are wondering whether is possible to add some additional software features: feel free to open respectively an Issue or a Pull Request in the package of interest on [our Bitbucket](https://bitbucket.org/account/user/qbrobotics/projects/ROS), according to whether it is just an idea or you have already provided your solution.

In any case, thank you for using [_qbrobotics®_](https://www.qbrobotics.com) solutions.

## Purchase
If you have just found out our company and you are interested in our products, come to [visit us](https://www.qbrobotics.com) and feel free to ask for a quote.

