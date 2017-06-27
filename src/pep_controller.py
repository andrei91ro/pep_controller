#!/usr/bin/env python
# The MIT License (MIT)

# Copyright (c) 2016 Andrei George Florea, Catalin Buiu

# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# pep_controller - a generic robot controller that uses Pep, an (Enzymatic) Numerical P System simulato,r as backend

from __future__ import print_function, with_statement, division
import rospy
from std_msgs.msg import Int32 # for robotID topic
from geometry_msgs.msg import Twist # for cmd_vel topic
from sensor_msgs.msg import Range # for /proximity topic
from std_msgs.msg import UInt8MultiArray # for /cmd_led topic
import thread # for lock (mutex)
import pep # Numerical P system simulator
import time # for initial sleep
import tf # for tf translation listener
import math # for math.degrees()
##########################################################################
# auxiliary definitions

class RobotType():

    """Enumeration of known types of robots. Each type of robot requires specific output (effector) values processing"""

    diff_drive      = 1 # differential drive robot (e-puck, kephera)
# end class RobotType

##########################################################################
# class definitions

class Constant():

    """Class used to store a generic robot parameter that has a constant value (and may be defined using several values)"""

    def __init__(self, pObjects = None, values = []):
        self.pObjects = pObjects # initally array of strings and later crossreferenced to form an array of Pep P object
        self.currentValue = values

    def restorePobject(self):
        """Restores the constant parameter value of the P object that has could have been reset to 0 if it was part of a production function"""
        for i, pObject in enumerate(self.pObjects):
            pObject.value = self.currentValue[i]
    # end restorePobject()
# end class Constant

class Sensor():

    """Class used to store a generic robot sensor (that may be defined by several variables)
    Provides thread block (during main thread read)"""

    def __init__(self, pObjects = None, topic=""):
        self.pObjects = pObjects # initally array of strings and later crossreferenced to form an array of Pep P object
        self.topic = topic
        self.currentValue = None # should only be handled using setValue() (or read indirectly using pObjects[i].value)
        self.lock = thread.allocate_lock() # construct lock (mutex)

    def setValue(self, newValue):
        """Record a new sensor value (array of values)

        :newValue: new sensor reading in the form of an array of new values"""

        self.lock.acquire() # lock mutex
        self.currentValue = newValue
        self.lock.release() # unlock mutex
    # end setValue()

    def updatePobject(self):
        """ Updates the value of the P object to the current value of this sensor reading """
        if (self.currentValue == None):
            rospy.loginfo("no sensor reading from topic %s" % self.topic)
            # skip this P object update request
            return

        self.lock.acquire() # lock mutex
        for i, pObject in enumerate(self.pObjects):
            pObject.value = self.currentValue[i]
        self.currentValue = None # clear the current sensor measurement and wait for a new one
        self.lock.release() # unlock mutex
    # end updatePobject()
# end class Sensor

class Effector():

    """Base effector class that is extended for specific effectors.
    The main purpose of the class is to set a pattern of storing the current and previous output values in order to determine whether a command publish is necessary"""

    def __init__(self, pObjects = None, topic = ""):
        self.pObjects = pObjects # initally array of strings and later crossreferenced to form an array of Pep P object
        self.previousPobjectValues = [0] * len(pObjects)
        self.topic = topic

    def isNewValue(self):
        """Compares the previous and current values in order to determine whether they are different
        :returns: True / False depending on whether the two values are identical (in the context of the type of the variables)"""

        return [pObj.value for pObj in self.pObjects] != self.previousPobjectValues
    # end isNewValue()

    def updateValue(self):
        """Update the previous value to the current one (done at the end of a simulation step)"""

        for i, pObject in enumerate(self.pObjects):
            self.previousPobjectValues[i] = pObject.value
    # end updateValue()

# end class Effector

class EffectorDiffDriveMovement(Effector):

    """Differential drive movement effector
    Converts a pair of motor speeds into a geometry_msgs.Twist message"""

    def __init__(self, pObjects = None, topic = "", wheelSeparation = 5.3, wheelDiameter = 4):
        """Constructs a new differential drive effector based on a set of parameters:
            :pObjects: The P objects that the topic values will be linked to
            :topic: The topic to which this effector will publish
            :wheelSeparation: The distance between wheels in centimeters (cm)
            :wheelDiameter: The wheel diameter in centimeters (cm)"""
        Effector.__init__(self, pObjects, topic)
        self.publisher = rospy.Publisher(self.topic, Twist, queue_size=10)
        self.wheelSeparation = wheelSeparation
        self.wheelDiameter = wheelDiameter

    def updateValue(self):
        """Check the Pobject values for changes (since the previous simulation step) and publish a message if changes have occured"""

        if (self.isNewValue()):
            newValue = Twist()
            leftMotorSpeed = self.pObjects[0].value
            rightMotorSpeed = self.pObjects[1].value

            newValue.linear.x = (leftMotorSpeed + rightMotorSpeed) / 2
            newValue.angular.z = (rightMotorSpeed - leftMotorSpeed) / self.wheelSeparation

            #publish the newly constructed value
            self.publisher.publish(newValue)

        Effector.updateValue(self)
    # end updateValue()
# end class EffectorDiffDriveMovement

class EffectorTwistMovement(Effector):

    """Linear, angular (geometry_msgs.Twist) speeds movement effector
    This class exposes all 3 components (x, y, z) of each speed"""

    def __init__(self, pObjects = None, topic = ""):
        """Creates a new instance of the class based on a set of parameters
            :pObjects: The P objects that the topic values will be linked to. Expects an array of 6 values (linear(x, y, z), angular(x, y, z))
            :topic: The topic to which this effector will publish"""
        Effector.__init__(self, pObjects, topic)
        self.publisher = rospy.Publisher(self.topic, Twist, queue_size=10)

    def updateValue(self):
        """Check the Pobject values for changes (since the previous simulation step) and publish a message if changes have occured"""

        if (self.isNewValue()):
            newValue = Twist()

            newValue.linear.x = self.pObjects[0].value
            newValue.linear.y = self.pObjects[1].value
            newValue.linear.z = self.pObjects[2].value
            newValue.angular.x = self.pObjects[3].value
            newValue.angular.y = self.pObjects[4].value
            newValue.angular.z = self.pObjects[5].value
            #publish the newly constructed value
            self.publisher.publish(newValue)

        Effector.updateValue(self)
    # end updateValue()
# end class EffectorTwistMovement

class EffectorLED(Effector):

    """LED state control effector
    Converts an array of 10 boolean values (0 or 1) into a std_msgs.UInt8MultiArray message
    The LEDs are numbered according to the specification of the '-L' command of the Advanced Sercom Protocol:
        http://www.gctronic.com/doc/index.php/Advanced_sercom_protocol"""

    def __init__(self, pObjects = None, ledNumbers = None, topic = ""):
        """Constructs a new LED state effector based on a set of parameters:
            :pObjects: The P objects that the topic values will be linked to
            :ledNumbers: The active led index numbers (array of values between 0 and 9)
            :topic: The topic to which this effector will publish"""
        Effector.__init__(self, pObjects, topic)
        self.ledNumbers = ledNumbers
        self.publisher = rospy.Publisher(self.topic, UInt8MultiArray, queue_size=10)

    def updateValue(self):
        """Check the Pobject values for changes (since the previous simulation step) and publish a message if changes have occured
        NOTE: the function considers all pObject values >= 1 to be 1 and otherwise 0"""

        if (self.isNewValue()):
            newValue = UInt8MultiArray()
            newValue.data = [0] * 10
            # process only LEDs that have been included in 'cmd_led' group
            for i, led in enumerate(self.ledNumbers):
                newValue.data[led] = 1 if (self.pObjects[i].value >= 1) else 0

            #publish the newly constructed value
            self.publisher.publish(newValue)

        Effector.updateValue(self)

    # end updateValue()
# end class EffectorLED

class EffectorMessageBroadcaster(Effector):

    """Robot message broadcaster
    Publishes a numeric message, constructed from the value of a P object, to the entire ROS system"""

    def __init__(self, pObjects = None, topic = ""):
        """TODO: to be defined1. """
        Effector.__init__(self, pObjects, topic)
        self.publisher = rospy.Publisher(self.topic, Int32, queue_size=10)

    def updateValue(self):
        """Check the Pobject values for changes (since the previous simulation step) and publish (broadcast) a message if changes have occured"""

        #if (self.isNewValue()):
            # effectively broadcast the new message
        self.publisher.publish(self.pObjects[0].value)

        Effector.updateValue(self)
# end class MessageBroadcaster

class Controller():

    """Class that is used to store the current state of a generic robot within the Numeric P system controller"""

    def __init__(self, robotType = RobotType.diff_drive, pepInputFile = None):
        self.numPsystem = pep.readInputFile(pepInputFile) # Pep simulated Numerical P system (constructed from input file)
        self.sensors = {} # empty dictionary of sensors (Sensor objects)
        self.effectors = {} # empty dictionary of effectors (Effector objects)
        self.constants = {} # empty dictionary of constant parameters (Constant objects)
        self.robotType = robotType # a generic description of the robot (diff_drive, quadcopter, ...)
        self.tfListener = tf.TransformListener()
        self.tf_frames = {} # dictionary of {TF receivers: TF frames} (receivers are also the names of the associated sensor object in the sensors{} dictionary)

        self.numPsystem.print(withPrograms=True)
    # end __init__()

    def interfaceWithDevices(self, sensors, effectors, constants = {}):
        """Crossreferences string identifiers of P system variables with an actual P object instance
        Can fail if not all input / output device variables (that were declared using parameters) are used within the Numerical P system
        :sensors: dictionary of sensors {dev_name: Sensor object}
        :effectors: dictionary of effectors {dev_name: Effector object}
        :constants: dictionary of constants {constant_name: Constant object}"""

        self.sensors = sensors
        self.effectors = effectors
        self.constants = constants

        rospy.logdebug("interfaceWithDevices() Received parameter list of:\n sensors = %s\n effectors = %s\n constants = %s" % (sensors.keys(), effectors.keys(), constants.keys()))

        sensorPobjects = []
        effectorPobjects = []
        constantPobjects = []

        totalSensorPobjectCount = 0
        totalEffectorPobjectCount = 0
        # cross-reference string identifiers with references to Pobject instances
        for var in self.numPsystem.variables:
            totalSensorPobjectCount = 0
            for sensor in self.sensors.values():
                for i, pObject in enumerate(sensor.pObjects[:]):
                    totalSensorPobjectCount += 1 # count this sensor
                    if (pObject == var.name):
                        rospy.logdebug("Replacing %s sensor variable" % var.name)
                        # replace string with P object reference
                        sensor.pObjects[i] = var
                        sensorPobjects.append(var.name)

            totalEffectorPobjectCount = 0
            for effector in self.effectors.values():
                for i, pObject in enumerate(effector.pObjects[:]):
                    totalEffectorPobjectCount += 1 # count this effector
                    if (pObject == var.name):
                        rospy.logdebug("Replacing %s effector variable" % var.name)
                        # replace string with P object reference
                        effector.pObjects[i] = var
                        effectorPobjects.append(var.name)

            totalConstantPobjectCount = 0
            for constant in self.constants.values():
                for i, pObject in enumerate(constant.pObjects[:]):
                    totalConstantPobjectCount += 1 # count this constant
                    if (pObject == var.name):
                        rospy.logdebug("Replacing %s constant" % var.name)
                        # replace string with P object reference
                        constant.pObjects[i] = var
                        constantPobjects.append(var.name)

        rospy.loginfo("Pobjects used for\n sensors = %s,\n effectors = %s,\n constants = %s" % (sensorPobjects, effectorPobjects, constantPobjects))

        if (totalSensorPobjectCount != len(sensorPobjects)):
            rospy.logwarn("Not all sensor values (n = %d) have been modelled using P objects (m = %d)." % (totalSensorPobjectCount, len(sensorPobjects)))
            #exit(1)
            self.numPsystem.variables.append(pep.Pobject(name="dummySensor", value=0))
            dummySensor = self.numPsystem.variables[-1]
            for sensor in self.sensors.values():
                for i, pObject in enumerate(sensor.pObjects[:]):
                    if (type(pObject) == str):
                        rospy.logwarn("    Replacing '%s' sensor component with dummySensor Pobject" % pObject)
                        sensor.pObjects[i] = dummySensor

        if (totalEffectorPobjectCount != len(effectorPobjects)):
            rospy.logwarn("Not all effector values (n = %d) have been modelled using P objects (m = %d)." % (totalEffectorPobjectCount, len(effectorPobjects)))
            #exit(1)
            self.numPsystem.variables.append(pep.Pobject(name="dummyEffector", value=0))
            dummyEffector = self.numPsystem.variables[-1]
            for effector in self.effectors.values():
                for i, pObject in enumerate(effector.pObjects[:]):
                    if (type(pObject) == str):
                        rospy.logwarn("    Replacing '%s' effector component with dummyEffector Pobject" % pObject)
                        effector.pObjects[i] = dummyEffector

        if (totalConstantPobjectCount != len(constantPobjects)):
            rospy.logwarn("Not all constant values (n = %d) have been modelled using P objects (m = %d). Shutting down controller!" % (totalConstantPobjectCount, len(constantPobjects)))
            exit(1)
    # end interfaceWithDevices()

    def handleDistanceSensors(self, data, sensor_id):
        """Distance sensor handler function that is called when receiving a message on a distance sensor topic

        :data: The contents of the message
        :message_id: Sensor identifier (used as key in the messages dictionary)"""

        rospy.logdebug("recording new value from sensor %s" % sensor_id)
        self.sensors[sensor_id].setValue([data.range * 15])
    # end handleDistanceSensors()

    def handleTfTransformReceive(self, data, sensor_id):
        """TF tranform receive handler function that is called when receiving a new transform
        The sensor data is joined as a single array of 6 values (translation.x,y,z, rotation.x,y,z)

        :data: array of 6 values (translation(x, y, z), rotation(x, y, z))
        :message_id: Sensor identifier (used as key in the messages dictionary)"""

        rospy.logdebug("recording new tf transform from frame %s" % sensor_id)
        self.sensors[sensor_id].setValue(data)
    # end handleTfTransformReceive()

    def handleMessageReceiveFromRobot(self, data, sensor_id):
        """Handler function that is called whenever a new message arrives from another robot.

        :data: The contents of the message
        :sensor_id: Message identifier (used as key in the sensors dictionary)"""

        rospy.logdebug("received new message from %s = %s" % (sensor_id, data))
        self.sensors[sensor_id].setValue([data.data])
    # end handleMessageReceiveFromRobot()

    def runControlStep(self):
        """Executes one control step consisting of preparing input data, executing one P system simulation step and lastly publishing new output values
        :returns: True / False depending on the success of the execution"""

        # prepare sensor input
        for sensor in self.sensors.values():
            sensor.updatePobject()

        # restore constant values into constants that could have been reset to 0 due to their presence in production rules
        for constant in self.constants.values():
            constant.restorePobject()

        self.numPsystem.print()

        try:
            self.numPsystem.runSimulationStep()
        except RuntimeError:
            rospy.logerr("Error encountered during execution of the numerical P system. Stopping controller")
            return False

        # check effector P objects for changes and if so then publish new messages to effectors
        for effector_name, effector in self.effectors.items():
            rospy.logdebug("updating effector %s: %s" % (effector_name, effector.pObjects))
            effector.updateValue()

        print("After sim step:")
        self.numPsystem.print()

        return True
    # end runControlStep()
# end class Controller

def pep_controller():
    rospy.init_node('pep_controller')
    #rospy.init_node('pep_controller', log_level=rospy.DEBUG)

    # read all parameters
    rate = rospy.Rate(rospy.get_param("~loopRateHz")) # 10hz default
    initialSleepSec = float(rospy.get_param("~initialSleepSec", 5)) # 10 seconds default
    pep_input_file = rospy.get_param("~pepInputFile")
    rospy.logwarn("Default value of 0 will be used for the robotID parameter if not supplied")
    robotID = int(rospy.get_param("~robotID", 0))
    rospy.logwarn("Default value of 'base_link' will be used for the tf_base_frame parameter if not supplied")
    tf_base_frame = rospy.get_param("~tf_base_frame", "base_link")
    controller = Controller(pepInputFile = pep_input_file)

    # build a publisher for the robotID topic (std_msgs.Int32) for use in a swarm
    pub_robotID = rospy.Publisher("robotID", Int32, queue_size=10)

    # retrieve and process the constants group (dictionary) of parameters
    constants = {}
    try:
        constants_group = rospy.get_param("constants")
        for const_name, const_value in constants_group.items():
            rospy.loginfo("Adding constant %s" % const_name)
            # if this constant's value is the robotID
            if (const_value == "robotID"):
                constants[const_name] = Constant(pObjects = [const_name], values = [robotID])
                rospy.loginfo("Adding constant robotID")
            else:
                constants[const_name] = Constant(pObjects = [const_name], values = [const_value])
    except KeyError:
        rospy.logwarn("No other constants, except robotID, have been set/detected")

    # retrieve and process the input group (dictionary) of parameters
    sensors = {}
    try:
        input_dev_group = rospy.get_param("input_dev/range")
        for dev_name, dev_topic in input_dev_group.items():
            rospy.loginfo("Adding Range sensor %s" % dev_name)
            sensors[dev_name] = Sensor(pObjects = [dev_name], topic = dev_topic)
            # create a subscriber
            rospy.Subscriber(dev_topic, Range, controller.handleDistanceSensors, dev_name)
    except KeyError:
        rospy.logwarn("No 'input_dev/range' (sensor_msgs.Range) sensors have been set/detected")

    try:
        input_tf = rospy.get_param("input_dev/tf")
        for receiver in input_tf.keys():
            try:
                tf_frame = rospy.get_param("input_dev/tf/%s/frame" % receiver)
            except KeyError:
                rospy.logerr("No 'frame' parameter has been defined / detected within /input_dev/tf/%s" % receiver)
                rospy.logwarn("Please define a 'frame' parameter in order to use the TF input device")
                # move on to the next receiver as this one is incomplete
                continue

            rospy.loginfo("Adding TF transform receiver %s for frame %s" % (receiver, tf_frame))
            try:
                translation = rospy.get_param("input_dev/tf/%s/translation" % receiver)
                rotation = rospy.get_param("input_dev/tf/%s/rotation" % receiver)
            except KeyError:
                rospy.logerr("No 'translation' or 'rotation' groups have been defined / detected within /input_dev/tf/%s" % receiver)
                rospy.logwarn("Please define both translation and rotation groups even if not used entirely in order to use the TF input device")
                # move on to the next receiver as this one is incomplete
                continue

            # sorted(dictionary) returns a list of dictionary keys sorted by their values
            tf_pObjects = sorted(translation)
            tf_pObjects.extend(sorted(rotation)) # concatenate the two lists

            sensors[receiver] = Sensor(pObjects = tf_pObjects, topic = ("tf/%s" % tf_frame))
            controller.tf_frames[receiver] = tf_frame
    except KeyError:
        rospy.logwarn("No 'input_dev/tf/' receivers have been set/detected")

    try:
        input_signal_receive = rospy.get_param("input_dev/signal_receive")
        for dev_name, dev_topic in input_signal_receive.items():
            rospy.loginfo("Adding signal receiver object %s" % dev_name)
            sensors[dev_name] = Sensor(pObjects = [dev_name], topic = dev_topic)
            #sensors[dev_name].setValue([0])
            # create a subscriber
            rospy.Subscriber(dev_topic, Int32, controller.handleMessageReceiveFromRobot, dev_name)
    except KeyError:
        rospy.logwarn("No 'input_dev/signal_receive' robot signal receiver has been set/detected")

    # retrieve and process the output group (dictionary) of parameters
    effectors = {}
    try:
        output_cmd_vel = rospy.get_param("output_dev/cmd_vel")
        try:
            linear = rospy.get_param("output_dev/cmd_vel/linear")
            angular = rospy.get_param("output_dev/cmd_vel/angular")

            # sorted(dictionary) returns a list of dictionary keys sorted by their values
            twist_pObjects = sorted(linear)
            twist_pObjects.extend(sorted(angular))

            effectors["cmd_vel"] = EffectorTwistMovement(pObjects = twist_pObjects, topic = "cmd_vel")
        except KeyError:
            rospy.logwarn("No 'linear' or 'angular' groups have been defined / detected within output_dev/cmd_vel")
            rospy.logerr("Please define both linear and angular groups even if not used entirely in order to use the output device")
    except KeyError:
        rospy.logwarn("No 'output_dev/cmd_vel' effector has been set/detected")

    try:
        output_cmd_vel = rospy.get_param("output_dev/cmd_vel_diff_drive")
        effectors["cmd_vel_diff_drive"] = EffectorDiffDriveMovement(pObjects = output_cmd_vel.keys(), topic = "cmd_vel")
    except KeyError:
        rospy.logwarn("No 'output_dev/cmd_vel_diff_drive' effector has been set/detected")

    try:
        output_cmd_led = rospy.get_param("output_dev/cmd_led")
        # sorted(output_cmd_led) returns a list of dictionary keys (param name) sorted by their values
        effectors["cmd_led"] = EffectorLED(pObjects = sorted(output_cmd_led), ledNumbers = sorted(output_cmd_led.values()),  topic = "cmd_led")
    except KeyError:
        rospy.logwarn("No 'output_dev/cmd_led' effector has been set/detected")

    try:
        signal_broadcast_group = rospy.get_param("output_dev/signal_broadcast")
        for dev_name, dev_topic in signal_broadcast_group.items():
            rospy.loginfo("Adding broadcast object %s" % dev_name)
            effectors[dev_name] = EffectorMessageBroadcaster(pObjects = [dev_name], topic = dev_topic)
    except KeyError:
        rospy.logwarn("No 'output_dev/signal_broadcast' robot signal emmiter effector has been set/detected")

    controller.interfaceWithDevices(sensors, effectors, constants)

    rospy.loginfo("Initial sleep of %f seconds" % initialSleepSec)
    time.sleep(initialSleepSec)

    while not rospy.is_shutdown():
        if (controller.tfListener.frameExists(tf_base_frame)):
            for receiver, tf_frame in controller.tf_frames.items():
                if (controller.tfListener.frameExists(tf_frame) == False):
                    rospy.logwarn("TF frame %s does not exist. Skipping this frame" % tf_frame)
                    continue
                try:
                    # get a transform between me (tf_base_frame) and a another frame (tf_frame)
                    (trans, rot) = controller.tfListener.lookupTransform(tf_base_frame, tf_frame, rospy.Time(0))
                    #(trans, rot) = controller.tfListener.lookupTransform(tf_frame, tf_base_frame, rospy.Time(0))
                    data = list(trans)
                    # convert rotation from quaternion to euler angles
                    rot = tf.transformations.euler_from_quaternion(rot)
                    # convert angles from radians to degrees
                    rot = [math.degrees(x) for x in rot]
                    data.extend(rot)
                    # execute the new sensor data handler
                    controller.handleTfTransformReceive(data, receiver)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logwarn("TF transform exception for frame %s. Skipping this frame" % tf_frame)
                    continue
        else:
            rospy.logwarn("Base TF frame %s does not exist, SKIPPING ALL TF processing" % tf_base_frame)

        # if errors are encountered during Pep execution
        if (controller.runControlStep() == False):
            rospy.logerr("errors were encountered during Pep execution")
            return

        pub_robotID.publish(robotID)
        rate.sleep()

if __name__ == '__main__':
    try:
        pep_controller()
    except rospy.ROSInterruptException:
        pass
