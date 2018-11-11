# Enzymatic numerical P system - robot control using the Robot Operating System (ROS)

This package is a flexible robot controller that uses enzymatic numerical P systems (ENPS) as base model.
It allows the user to control ROS-compatible robots with great flexibility thanks to the use of ROS launch files.

This package depends on the `pep.py` script from the PeP repository [Python2 version](https://github.com/andrei91ro/pep/tree/python2_compatible_take_two).
The `pep.py` script must be placed in the `src` folder.

## Connection with ROS

This package uses ROS Parameters to define the connection between the controller and ROS, as shown in the diagram below:

[[https://github.com/andrei91ro/pep_controller/blob/master/diagram.png|alt=pep_controller_diagram]]

More specifically, starting from a skeleton launch file such as:
```xml

<launch>
    <arg name="pep_input_file" default="$(find pep_controller)src/pep_input/hello_world.pep"/>
    <arg name="loop_rate_hz" default="10"/>

    <group ns="constants">
        <param name="kP" value="10" />
    </group>

    <group ns="input_dev">
        <group ns="range">
            <param name="prox0" value="proximity0" />
            <param name="prox1" value="proximity1" />
        </group>
    </group>

    <group ns="output_dev">
        <group ns="cmd_vel">
            <group ns="linear">
                <param name="linear_x" value="x" />
                <param name="linear_y" value="y" />
                <param name="linear_z" value="z" />
            </group>
            <group ns="angular">
                <param name="angular_x" value="x" />
                <param name="angular_y" value="y" />
                <param name="angular_z" value="z" />
            </group>
        </group>
    </group>

	<node pkg="pep_controller" type="pep_controller.py" name="pep_control" output="screen">
        <param name="pepInputFile" value="$(arg pep_input_file)"/>
        <param name="loopRateHz" value="$(arg loop_rate_hz)"/>

        <!--remap the cmd_vel output topic to that expected by epuck_driver-->
        <remap from="cmd_vel" to="mobile_base/cmd_vel"/>
	</node>

	<!--<node pkg="rviz" type="rviz" name="rviz" />-->

</launch>
```
one would be able to use the following ENPS objects in the ENPS system defined in `pep_input/hello_world.pep`:

* `kP` always receive the constant value 10, before the start of each simulation step. Even if the value is consumed by the system, the controller will always restore it to its set value before each simulation step.

* `prox0` and `prox1` that receive the most recently received value from proximity sensors ([sensor_msgs/Range](http://docs.ros.org/jade/api/sensor_msgs/html/msg/Range.html)) with ROS topics `/proximity0` and `/proximity1` respectively.
    + The values are received and stored in the associated P object before the start of each ENPS simulation step.
* the current value of `linear_x`, `linear_y`, `linear_z`, `angular_x`, `angular_y`, `angular_z` would be used to compose a new [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) messsage.
    + The message is published immediately after the end of the simulation step;


# ROS compatibility

The package is designed to use the following topics:

* INPUT:
    + `sensor_msgs/Range` (see `launch/leader_follow_distance.launch`)
    + `sensor_msgs/Imu` (see `launch/drone_p_controller.launch`)
    + TF library translation/rotation (see `launch/leader_follow_tf.launch`)
    + any `std_msgs/Int32` numerical (single value topic) (see `signal_receive` in `launch/led_sync.launch`)
    + crazyflie `GenericLogData` (see [crazyflie_ros](https://github.com/whoenig/crazyflie_ros)) available in the `crazyflie` branch (see `launch/crazyflie_pi_waypoint_height.launch`)

* OUTPUT:
    + `geometry_msgs/Twist` in both its complete 6 DOF form (see `launch/leader_follow_tf.launch`) and a 2-DOF form for use on differential-drive robots (see `launch/leader_follow_distance.launch`)
    + any `std_msgs/UInt8MultiArray` value; used here for the `cmd_led` topic available for the [e_puck driver](https://github.com/gctronic/epuck_driver_cpp) LEDs.
    + any `std_msgs/Int32` numerical (single value topic) (see `signal_broadcast` in `launch/led_sync.launch`)
    + crazyflie `Hover` (see [crazyflie_ros](https://github.com/whoenig/crazyflie_ros)) available in the `crazyflie` branch (see `launch/crazyflie_pi_waypoint_height.launch`)

The package was tested on ROS Indigo on Ubuntu 14.04.

# Authors
Andrei George Florea, [Cătălin Buiu](http://catalin.buiu.net)

[Department of Automatic Control And Systems Engineering](http://acse.pub.ro),

Politehnica University of Bucharest

Bucharest, Romania.
