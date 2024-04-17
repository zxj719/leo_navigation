# leo_navigation
## 1. prerequisition
```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-rplidar-ros
sudo apt install ros-humble-leo-viz
sudo apt install ros-humble-ros-gz
sudo apt install xterm
sudo apt install qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev
sudo apt install ros-humble-imu-filter-madgwick
sudo apt install ros-humble-robot-localization

```

## 2. setup rplidar
```bash
cd ~/ros2_ws
source install/setup.bash
sudo chmod 777 /dev/ttyUSB0
source src/rplidar_ros/scripts/create_udev_rules.sh
```
Now, you can launch the lidar node by
```bash
ros2 launch rplidar_ros view_rplidar_a2m12_launch.py
```
```
<?xml version="1.0" encoding="utf-8"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="leo" 
               params="default_antenna:=true
                       rockers_fixed:=true
                       footprint_link:=true
                       link_prefix:=''
                       joint_prefix:=''
                       imu_translation:='0.0628 -0.0314 -0.0393'
                       mecanum_wheels:=false">
  <!-- macro -->
    <xacro:macro name="rocker_link" params="name">
      <link name="${link_prefix}rocker_${name}_link">
        <inertial>
          <mass value="1.387336"/>
          <origin xyz="0 0.01346 -0.06506"/>
          <inertia
            ixx="0.002956" ixy="-0.000001489324" ixz="-0.000008103407"
            iyy="0.02924"  iyz="0.00007112"
            izz="0.02832"/>
        </inertial>
        <visual>
          <geometry>
            <mesh filename="file://$(find real_leo)/models/Rocker.dae"/>
          </geometry>
        </visual>
        <collision>
          <geometry>
            <mesh filename="file://$(find real_leo)/models/Rocker_outline.stl"/>
          </geometry>
        </collision>
      </link>
    </xacro:macro>

    <xacro:macro name="wheel_link" params="name model"> 
      <link name="${link_prefix}wheel_${name}_link">
        <inertial>
          <mass value="0.283642"/>
          <origin xyz="0 0.030026 0"/>
          <inertia
            ixx="0.000391"  ixy="0.00000123962" ixz="5.52582e-7"
            iyy="0.0004716" iyz="-0.000002082042"
            izz="0.000391"/>
        </inertial>
        <visual>
          <geometry>
            <mesh filename="file://$(find real_leo)/models/Wheel${model}.dae"/>
          </geometry>
        </visual>
        <collision>
          <origin xyz="0 0.04485 0" rpy="${pi/2} 0 0"/>
          <geometry>
            <cylinder radius="0.057" length="0.07"/>
          </geometry>
        </collision>
        <collision>
          <origin xyz="0 0.04485 0" rpy="${pi/2} 0 0"/>
          <geometry>
            <cylinder radius="0.0625" length="0.04"/>
          </geometry>
        </collision>
        <collision>
          <geometry>
            <mesh filename="file://$(find real_leo)/models/Wheel_outline.stl"/>
          </geometry>
        </collision>
      </link>
    </xacro:macro>

    <xacro:macro name="mecanum_wheel_link" params="name model"> 
      <link name="${link_prefix}wheel_${name}_link">
        <inertial>
          <mass value="0.672214"/>
          <origin xyz="0 -0.036852 0"/>
          <inertia
            ixx="0.0009778"  ixy="-8.10228e-7" ixz="3.2e-11"
            iyy="0.001321" iyz="5.7e-11"
            izz="0.0009773"/>
        </inertial>
        <visual>
          <geometry>
            <mesh filename="file://$(find real_leo)/models/WheelMecanum${model}.dae"/>
          </geometry>
        </visual>
        <collision>
          <origin xyz="0 -0.0317 0" rpy="${pi/2} 0 0"/>
          <geometry>
            <cylinder radius="0.0635" length="0.0515"/>
          </geometry>
        </collision>
        <collision>
          <geometry>
            <mesh filename="file://$(find real_leo)/models/Mecanum_wheel_outline.stl"/>
          </geometry>
        </collision>
      </link>
    </xacro:macro>

  <!-- LINKS -->

    <!-- base_footprint -->
    <xacro:if value="${footprint_link}">
      <link name="${link_prefix}base_footprint"/>
    </xacro:if>

    <link name="${link_prefix}base_link">
      <inertial>
        <mass value="1.584994"/>
        <origin xyz="-0.019662 0.011643 -0.031802"/>
        <inertia
          ixx="0.01042" ixy="0.001177" ixz="-0.0008871"
          iyy="0.01045" iyz="0.0002226"
          izz="0.01817"/>
      </inertial>
      <visual>
        <geometry>
          <mesh filename="file://$(find real_leo)/models/Chassis.dae"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <mesh filename="file://$(find real_leo)/models/Chassis_outline.stl"/>
        </geometry>
      </collision>
    </link>

    <xacro:rocker_link name="L"/>
    <xacro:rocker_link name="R"/>
    <xacro:if value="${mecanum_wheels}">
      <xacro:mecanum_wheel_link name="FL" model="B"/>
      <xacro:mecanum_wheel_link name="RL" model="A"/>
      <xacro:mecanum_wheel_link name="FR" model="A"/>
      <xacro:mecanum_wheel_link name="RR" model="B"/>
    </xacro:if>
    <xacro:unless value="${mecanum_wheels}">
      <xacro:wheel_link name="FL" model="A"/>
      <xacro:wheel_link name="RL" model="A"/>
      <xacro:wheel_link name="FR" model="B"/>
      <xacro:wheel_link name="RR" model="B"/>
    </xacro:unless>

    <xacro:if value="${default_antenna}">
      <link name="${link_prefix}antenna_link">
        <inertial>
          <mass value="0.001237"/>
          <origin xyz="0 0 0.028828"/>
          <inertia
            ixx="2.5529e-7" ixy="0.0" ixz="0.0"
            iyy="2.5529e-7" iyz="0.0"
            izz="1.354e-8"/>
        </inertial>
        <visual>
          <geometry>
            <mesh filename="file://$(find real_leo)/models/Antenna.dae"/>
          </geometry>
        </visual>
        <collision>
          <origin xyz="0 0 0.028"/>
          <geometry>
            <cylinder radius="0.0055" length="0.056"/>
          </geometry>
        </collision>
      </link>
    </xacro:if>

    <link name="${link_prefix}camera_frame"/>
    <link name="${link_prefix}camera_optical_frame"/>
    <link name="${link_prefix}imu_frame"/>

  <!-- JOINTS -->

    <xacro:if value="${footprint_link}">
      <joint name="${joint_prefix}base_joint" type="fixed">
        <origin xyz="0 0 0.19783" rpy="0 0 0"/>
        <parent link="${link_prefix}base_footprint"/>
        <child link="${link_prefix}base_link"/>
      </joint>
    </xacro:if>

    <xacro:if value="${rockers_fixed}">
      <xacro:property name="rockers_joint_type" value="fixed"/>
    </xacro:if>
    <xacro:unless value="${rockers_fixed}">
      <xacro:property name="rockers_joint_type" value="revolute"/>
    </xacro:unless>

    <joint name="${joint_prefix}rocker_L_joint" type="${rockers_joint_type}">
      <origin xyz="0.00263 0.14167 -0.04731" rpy="0 0 ${pi}"/>
      <parent link="${link_prefix}base_link"/>
      <child link="${link_prefix}rocker_L_link"/>
      <axis xyz="0 1 0"/>
      <limit effort="100.0" lower="-0" upper="0" velocity="100.0"/>
      <dynamics friction="1.0" damping="0.1"/>
    </joint>

    <joint name="${joint_prefix}rocker_R_joint" type="${rockers_joint_type}">
      <origin xyz="0.00263 -0.14167 -0.04731" rpy="0 0 0"/>
      <parent link="${link_prefix}base_link"/>
      <child link="${link_prefix}rocker_R_link"/>
      <axis xyz="0 1 0"/>
      <limit effort="100.0" lower="-0" upper="0" velocity="100.0"/>
      <dynamics friction="1.0" damping="0.1"/> 
      <mimic joint="rocker_L_joint"/>
    </joint>

    <joint name="${joint_prefix}wheel_FL_joint" type="continuous">
      <xacro:if value="${mecanum_wheels}">
        <origin xyz="-0.15256 -0.00884 -0.08802" rpy="0 0 0"/>
      </xacro:if>
      <xacro:unless value="${mecanum_wheels}">
        <origin xyz="-0.15256 -0.08214 -0.08802" rpy="0 0 0"/>
      </xacro:unless>
      <parent link="${link_prefix}rocker_L_link"/>
      <child link="${link_prefix}wheel_FL_link"/>
      <axis xyz="0 -1 0"/>
      <limit effort="2.0" velocity="6.0"/>
      <dynamics friction="0.3125" damping="0.1"/>
    </joint>

    <joint name="${joint_prefix}wheel_RL_joint" type="continuous">
      <xacro:if value="${mecanum_wheels}">
        <origin xyz="0.15256 -0.00884 -0.08802" rpy="0 0 0"/>
      </xacro:if>      
      <xacro:unless value="${mecanum_wheels}">
        <origin xyz="0.15256 -0.08214 -0.08802" rpy="0 0 0"/>
      </xacro:unless>
      <parent link="${link_prefix}rocker_L_link"/>
      <child link="${link_prefix}wheel_RL_link"/>
      <axis xyz="0 -1 0"/>
      <limit effort="2.0" velocity="6.0"/>
      <dynamics friction="0.3125" damping="0.1"/>
    </joint>

    <joint name="${joint_prefix}wheel_FR_joint" type="continuous">
      <xacro:if value="${mecanum_wheels}">
        <origin xyz="0.15256 -0.00884 -0.08802" rpy="0 0 0"/>
      </xacro:if>
      <xacro:unless value="${mecanum_wheels}">
        <origin xyz="0.15256 -0.08214 -0.08802" rpy="0 0 0"/>
      </xacro:unless>
      <parent link="${link_prefix}rocker_R_link"/>
      <child link="${link_prefix}wheel_FR_link"/>
      <axis xyz="0 1 0"/>
      <limit effort="2.0" velocity="6.0"/>
      <dynamics friction="0.3125" damping="0.1"/>
    </joint>

    <joint name="${joint_prefix}wheel_RR_joint" type="continuous">
      <xacro:if value="${mecanum_wheels}">
        <origin xyz="-0.15256 -0.00884 -0.08802" rpy="0 0 0"/>
      </xacro:if>
      <xacro:unless value="${mecanum_wheels}">
        <origin xyz="-0.15256 -0.08214 -0.08802" rpy="0 0 0"/>
      </xacro:unless>
      <parent link="${link_prefix}rocker_R_link"/>
      <child link="${link_prefix}wheel_RR_link"/>
      <axis xyz="0 1 0"/>
      <limit effort="2.0" velocity="6.0"/>
      <dynamics friction="0.3125" damping="0.1"/> 
    </joint>

    <xacro:if value="${default_antenna}">
      <joint name="${joint_prefix}antenna_joint" type="fixed">
        <origin xyz="-0.0052 0.056 -0.0065" rpy="0 0 0"/>
        <parent link="${link_prefix}base_link"/>
        <child link="${link_prefix}antenna_link"/>
      </joint>
    </xacro:if>

    <joint name="${joint_prefix}camera_joint" type="fixed">
      <origin xyz="0.0971 0 -0.0427" rpy="0 0.2094 0"/>
      <parent link="${link_prefix}base_link"/>
      <child link="${link_prefix}camera_frame"/>
    </joint>

    <joint name="${joint_prefix}camera_optical_joint" type="fixed">
      <origin xyz="0 0 0" rpy="-${pi/2} 0.0 -${pi/2}"/>
      <parent link="${link_prefix}camera_frame"/>
      <child link="${link_prefix}camera_optical_frame"/>
    </joint>

    <joint name="${joint_prefix}imu_joint" type="fixed">
      <origin xyz="${imu_translation}" rpy="0 0 0"/>
      <parent link="${link_prefix}base_link"/>
      <child link="${link_prefix}imu_frame"/>
    </joint>

  <!-- LIDAR PART IS COMING IN ANOTHER FILE! -->

  </xacro:macro>


<!-- ****************************************************** -->
<!-- LEO GAZEBO -->
  <xacro:macro name="leo_gazebo"
               params="robot_ns:='''">

    <xacro:property name="link_prefix" value=""/>
    <xacro:property name="topic_prefix" value="/model/leo_rover/"/>
    <xacro:if value="${robot_ns != '' and robot_ns != '/'}">
      <xacro:property name="link_prefix" value="${robot_ns}/"/>
    </xacro:if>

  <!-- base ODE properties -->
    <gazebo reference="${link_prefix}base_footprint">
      <collision>
        <surface>
          <friction>
            <ode>
              <mu>0.3</mu>
              <mu2>0.3</mu2>
            </ode>
            <bullet>
              <friction>0.3</friction>
              <friction2>0.3</friction2>
              <rolling_friction>0.1</rolling_friction>
            </bullet>
          </friction>
        </surface>
      </collision>
    </gazebo>

    <gazebo reference="${link_prefix}base_link">
      <collision>
        <surface>
          <friction>
            <ode>
              <mu>0.3</mu>
              <mu2>0.3</mu2>
            </ode>
            <bullet>
              <friction>0.3</friction>
              <friction2>0.3</friction2>
              <rolling_friction>0.1</rolling_friction>
            </bullet>
          </friction>
        </surface>
      </collision>
    </gazebo>

  <!-- rocker ODE properties -->
    <gazebo reference="${link_prefix}rocker_L_link">
      <collision>
        <surface>
          <friction>
            <ode>
              <mu>0.3</mu>
              <mu2>0.3</mu2>
            </ode>
            <bullet>
              <friction>0.3</friction>
              <friction2>0.3</friction2>
              <rolling_friction>0.1</rolling_friction>
            </bullet>
          </friction>
        </surface>
      </collision>
    </gazebo>

    <gazebo reference="${link_prefix}rocker_R_link">
      <collision>
        <surface>
          <friction>
            <ode>
              <mu>0.3</mu>
              <mu2>0.3</mu2>
            </ode>
            <bullet>
              <friction>0.3</friction>
              <friction2>0.3</friction2>
              <rolling_friction>0.1</rolling_friction>
            </bullet>
          </friction>
        </surface>
      </collision>
    </gazebo>
  <!-- wheel ODE properties -->
    <gazebo reference="${link_prefix}wheel_FL_link">
      <collision>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
              <fdir1>0 0 0</fdir1>
            </ode>
            <bullet>
              <friction>1</friction>
              <friction2>1</friction2>
              <rolling_friction>0.1</rolling_friction>
            </bullet>
          </friction>
        </surface>
      </collision>
    </gazebo>

    <gazebo reference="${link_prefix}wheel_FR_link">
      <collision>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
              <fdir1>0 0 0</fdir1>
            </ode>
            <bullet>
              <friction>1</friction>
              <friction2>1</friction2>
              <rolling_friction>0.1</rolling_friction>
            </bullet>
          </friction>
        </surface>
      </collision>
    </gazebo>

    <gazebo reference="${link_prefix}wheel_RL_link">
      <collision>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
              <fdir1>0 0 0</fdir1>
            </ode>
            <bullet>
              <friction>1</friction>
              <friction2>1</friction2>
              <rolling_friction>0.1</rolling_friction>
            </bullet>
          </friction>
        </surface>
      </collision>
    </gazebo>

    <gazebo reference="${link_prefix}wheel_RR_link">
      <collision>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
              <fdir1>0 0 0</fdir1>
            </ode>
            <bullet>
              <friction>1</friction>
              <friction2>1</friction2>
              <rolling_friction>0.1</rolling_friction>
            </bullet>
          </friction>
        </surface>
      </collision>
    </gazebo>

    <!-- rocker differential has been deleted from our package-->
    <!-- <gazebo>
      <plugin name="leo_gz::DifferentialSystem" filename="libleo_gz_differential_plugin.so">
        <jointA>rocker_L_joint</jointA>
        <jointB>rocker_R_joint</jointB>
        <forceConstant>100.0</forceConstant>
      </plugin>
    </gazebo> -->


<!-- GAZEBO PLUGINS -->

  <!-- JointStatePublisher -->
    <gazebo>
      <plugin
        filename="ignition-gazebo-joint-state-publisher-system"
        name="ignition::gazebo::systems::JointStatePublisher">
        <topic>${topic_prefix}joint_states</topic>
      </plugin>
    </gazebo>

  <!-- Imu -->
    <gazebo reference="${link_prefix}imu_frame">
      <sensor type="imu" name="leo_imu_sensor">
      <plugin filename="libignition-gazebo-imu-system.so"
        name="ignition::gazebo::systems::Imu">
      </plugin>
        <update_rate>100</update_rate>
        <visualize>true</visualize>
        <topic>${topic_prefix}imu/data_raw</topic>
        <always_on>1</always_on>
        <ignition_frame_id>${link_prefix}imu_frame</ignition_frame_id>
        <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
                <bias_mean>0.0000075</bias_mean>
                <bias_stddev>0.0000008</bias_stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
                <bias_mean>0.0000075</bias_mean>
                <bias_stddev>0.0000008</bias_stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
                <bias_mean>0.0000075</bias_mean>
                <bias_stddev>0.0000008</bias_stddev>
              </noise>
            </z>
          </angular_velocity>
          <linear_acceleration>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
                <bias_mean>0.1</bias_mean>
                <bias_stddev>0.001</bias_stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
                <bias_mean>0.1</bias_mean>
                <bias_stddev>0.001</bias_stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
                <bias_mean>0.1</bias_mean>
                <bias_stddev>0.001</bias_stddev>
              </noise>
            </z>
          </linear_acceleration>
        </imu>
      </sensor>
    </gazebo>

  <!-- Differential Drive -->
    <gazebo>
      <plugin filename="libignition-gazebo-diff-drive-system.so"
        name="ignition::gazebo::systems::DiffDrive">
        <!-- Wheel Joints -->
        <left_joint>wheel_FL_joint</left_joint>
        <right_joint>wheel_FR_joint</right_joint>
        <left_joint>wheel_RL_joint</left_joint>
        <right_joint>wheel_RR_joint</right_joint>

        <!-- Kinematics -->
        <wheel_separation>0.358</wheel_separation>
        <wheel_radius>0.0625</wheel_radius>

        <!-- TF Frames -->
        <frame_id>odom</frame_id>
        <child_frame_id>${link_prefix}base_footprint</child_frame_id>
        <odom_publish_frequency>50</odom_publish_frequency>

        <!-- topics -->
        <topic>${topic_prefix}cmd_vel</topic>
        <odom_topic>${topic_prefix}odom</odom_topic>
        <tf_topic>${topic_prefix}tf</tf_topic>

        <max_linear_acceleration>2.5</max_linear_acceleration>
        <max_angular_acceleration>3</max_angular_acceleration>
        <min_linear_acceleration>-2.5</min_linear_acceleration>
        <min_angular_acceleration>-3</min_angular_acceleration>
      </plugin>
    </gazebo>

  <!-- lidar -->
    <gazebo reference="laser_frame">
      
      <sensor type="gpu_lidar" name="leo_lidar_sensor">
      <plugin filename="libignition-gazebo-sensors-system.so" 
        name="ignition::gazebo::systems::Sensors">
          <render_engine>ogre2</render_engine>
      </plugin>
        <topic>${topic_prefix}scan</topic>
        <frame_id>laser_frame</frame_id>
        <ignition_frame_id>laser_frame</ignition_frame_id>

        <update_rate>10.0</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>512</samples>
              <resolution>1</resolution>
              <min_angle>-${pi*0.75}</min_angle>
              <max_angle>${pi*0.75}</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.10</min>
            <max>10.0</max>
            <resolution>0.03</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.001</stddev>
          </noise>
        </ray>
        <always_on>true</always_on>
        <visualize>false</visualize>
      </sensor>
    </gazebo>
  
  <!-- camera -->
    <gazebo reference="${link_prefix}camera_frame">
      <sensor name="camera" type="camera">
          <camera name="leo_camera">
              <horizontal_fov>1.9</horizontal_fov>
              <image>
                  <width>640</width>
                  <height>480</height>
                  <format>R8G8B8</format>
              </image>
              <clip>
                  <near>0.2</near>
                  <far>300</far>
              </clip>
              <noise>
                <type>gaussian</type>
                <mean>0.0</mean>
                <stddev>0.007</stddev>
              </noise>
              <distortion>
                <k1>-0.279817</k1>
                <k2>0.060321</k2>
                <k3>0.000487</k3>
                <p1>0.000310</p1>
                <p2>0.000000</p2>
                <center>0.5 0.5</center>
              </distortion>              
          </camera>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
          <visualize>true</visualize>
          <topic>${topic_prefix}camera/image_raw</topic>
          <ignition_frame_id>${link_prefix}camera_frame</ignition_frame_id>
      </sensor>
    </gazebo>
  </xacro:macro>


<!-- main  -->
  <xacro:macro name="leo_sim" 
               params="default_antenna:=true 
                       fixed:=false
                       robot_ns:=''
                       mecanum_wheels:=false">

    <xacro:property name="link_prefix" value=""/>
    <xacro:if value="${robot_ns != '' and robot_ns != '/'}">
      <xacro:property name="link_prefix" value="${robot_ns}/"/>
    </xacro:if>

    <xacro:leo default_antenna="${default_antenna}"
               rockers_fixed="false"
               link_prefix="${link_prefix}"
               mecanum_wheels="${mecanum_wheels}"/>

    <!-- <xacro:leo_gazebo robot_ns="${robot_ns}"/> -->

    <!-- <xacro:if value="${fixed}">
      <link name="world"/>
      <joint name="fixed" type="fixed">
        <parent link="world"/>
        <child link="${link_prefix}base_footprint"/>
        <origin xyz="0.0 0.0 1.0"/>
      </joint>
    </xacro:if> -->
  
  </xacro:macro>

</robot>
```
