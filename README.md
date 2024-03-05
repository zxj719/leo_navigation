## Following steps should be performed in raspberry pi(remote connect state).
Get familiar with git in my another repo([leo_workflow](https://github.com/zxj719/leo_workflow)), then download prepared pkg under src.

```bash
git clone -b teleop_leo https://github.com/zxj719/leo_navigation.git
```

rename the pkg: teleop_leo, build and launch the joystick one, more detailed [instruction](https://github.com/UoMMScRobotics/UOMDocumentationForLeoRover/blob/main/Task04_Steering_with_joystick/Joystick.md) .

In /etc/ros/robot.launch.xml:
```xml
<include file="/etc/ros/laser.launch.xml"/>
```

In /etc/ros/urdf/robot.urdf.xacro:
```xml
<xacro:include filename="/etc/ros/urdf/laser.urdf.xacro"/>
```
