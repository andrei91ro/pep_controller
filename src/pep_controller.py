#!/usr/bin/env python
# The MIT License (MIT)

# Copyright (c) 2016 Andrei George Florea, Catalin Buiu

# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# pep_controller - a generic robot controller that uses Pep, an (Enzymatic) Numerical P System simulato,r as backend

import rospy
#from std_msgs.msg import String
from geometry_msgs.msg import Twist # for cmd_vel topic
from sensor_msgs.msg import Range # for cmd_vel topic
import thread # for lock (mutex)
import pep # Numerical P system simulator
##########################################################################
# auxiliary definitions

class RobotType():

    """Enumeration of known types of robots. Each type of robot requires specific output (effector) values processing"""

    diff_drive      = 1 # differential drive robot (e-puck, kephera)
# end class RobotType

##########################################################################
# class definitions

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
        if (type(self.currentValue) != list):
            rospy.loginfo("no sensor reading from topic %s" % self.topic)
            # skip this P object update request
            return

        self.lock.acquire() # lock mutex
        for i, pObject in enumerate(self.pObjects):
            pObject.value = self.currentValue[i]
        self.lock.release() # unlock mutex
    # end updatePobject()
# end class Sensor

class Effector():

    """Base effector class that is extended for specific effectors.
    The main purpose of the class is to set a pattern of storing the current and previous output values in order to determine whether a command publish is necessary"""

    def __init__(self, pObjects = None, topic = ""):
        self.pObjects = pObjects # initally array of strings and later crossreferenced to form an array of Pep P object
        self.previousPobjectValues = [0] * len(pObjects)

    def isNewValue(self):
        """Compares the previous and current values in order to determine whether they are different
        :returns: True / False depending on whether the two values are identical (in the context of the type of the variables)"""

        return [pObj.value for pObj in self.pObjects] == self.previousPobjectValues
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

    def __init__(self, pObjects = None, topic = "", motorMin = 0, motorMax = 100, twistMaxLinearSpeed = 10, twistMaxAngularSpeed = 10):
        Effector.__init__(self, pObjects, topic)
        self.topic = topic
        self.publisher = rospy.Publisher(self.topic, Twist, queue_size=10)

        self.motorMin = 0 # robot specific minimum motor speed
        self.motorMax = 100 # robot specific maximum motor speed
        # TODO take the sign into account (-10 is ok for Twist speeds and is a maximum backward speed)
        self.twistMaxLinearSpeed = 10 # user specified maximum linear speed
        self.twistMaxAngularSpeed = 10 # user specified maximum linear speed

    def updateValue(self):
        """Check the Pobject values for changes (since the previous simulation step) and publish a message if changes have occured"""

        if (self.isNewValue()):
            newValue = Twist()
            leftMotorSpeed = self.pObjects[0].value
            rightMotorSpeed = self.pObjects[1].value
            # TODO consult epuck_driver and docs for a real conversion formula
            newValue.linear.x = (1 - (leftMotorSpeed - rightMotorSpeed) / (self.motorMax - self.motorMin)) * self.twistMaxLinearSpeed
            newValue.angular.z = (leftMotorSpeed - rightMotorSpeed) / (self.motorMax - self.motorMin) * self.twistMaxLinearSpeed

            #publish the newly constructed value
            self.publisher.publish(newValue)

        Effector.updateValue(self)

    # end setValueFromMotorSpeeds()
# end class EffectorDiffDriveMovement

class Controller():

    """Class that is used to store the current state of a generic robot within the Numeric P system controller"""

    def __init__(self, robotType = RobotType.diff_drive, pepInputFile = None):
        self.numPsystem = pep.readInputFile(pepInputFile) # Pep simulated Numerical P system (constructed from input file)
        self.sensors = {} # empty dictionary of sensors (Sensor objects)
        self.effectors = {} # empty dictionary of effectors (Effector objects)
        self.robotType = robotType # a generic description of the robot (diff_drive, quadcopter, ...)
    # end __init__()

    def interfaceWithDevices(self, sensors, effectors):
        """Crossreferences string identifiers of P system variables with an actual P object instance
        Can fail if not all input / output device variables (that were declared using parameters) are used within the Numerical P system
        :sensors: dictionary of sensors {dev_name: Sensor object}
        :effectors: dictionary of effectors {dev_name: Effector object}"""

        self.sensors = sensors
        self.effectors = effectors

        sensorPobjects = []
        effectorPobjects = []

        # cross-reference string identifiers with references to Pobject instances
        for var in self.numPsystem.variables:
            totalSensorPobjectCount = 0
            for sensor in self.sensors.values():
                for i, pObject in enumerate(sensor.pObjects[:]):
                    totalSensorPobjectCount += 1 # count this sensor
                    if (pObject == var.name):
                        # replace string with P object reference
                        sensor.pObjects[i] = var
                        sensorPobjects.append(var.name)

            totalEffectorPobjectCount = 0
            for effector in self.effectors.values():
                for i, pObject in enumerate(effector.pObjects[:]):
                    totalEffectorPobjectCount += 1 # count this effector
                    if (pObject == var.name):
                        # replace string with P object reference
                        effector.pObjects[i] = var
                        effectorPobjects.append(var.name)

        rospy.loginfo("Pobjects used for sensors = %s, Pobjects used for effectors = %s" % (sensorPobjects, effectorPobjects))

        if (totalSensorPobjectCount != len(sensorPobjects)):
            rospy.logerr("Not all sensor values (n = %d) have been modelled using P objects (m = %d). Shutting down controller!" % (totalSensorPobjectCount, len(sensorPobjects)))
            exit(1)
        if (totalEffectorPobjectCount != len(effectorPobjects)):
            rospy.logerr("Not all effector values (n = %d )have been modelled using P objects (m = %d). Shutting down controller!" % (totalEffectorPobjectCount, len(effectorPobjects)))
            exit(1)
    # end interfaceWithDevices()

    def handleDistanceSensors(self, data, sensor_id):
        """Distance sensor handler function that is called when receiving a message on a distance sensor topic"""
        rospy.logdebug("recording new value from sensor %s" % sensor_id)
        self.sensors[sensor_id].setValue(data)
    # end handleDistanceSensors()

    def runControlStep(self):
        """Executes one control step consisting of preparing input data, executing one P system simulation step and lastly publishing new output values
        :returns: True / False depending on the success of the execution"""

        # prepare sensor input
        for sensor in self.sensors.values():
            sensor.updatePobject()

        try:
            self.numPsystem.runSimulationStep()
        except RuntimeError:
            rospy.logerr("Error encountered during execution of the numerical P system. Stopping controller")
            return False

        # check effector P objects for changes and if so then publish new messages to effectors
        for effector in self.effectors.values():
            effector.updateValue()

        return True
    # end runControlStep()
# end class Controller

def pep_controller():
    rospy.init_node('pep_controller')
    rate = rospy.Rate(10) # 10hz

    # read all parameters

    pep_input_file = rospy.get_param("~pepInputFile")
    controller = Controller(pepInputFile = pep_input_file)

    # retrieve and process the input group (dictionary) of parameters
    sensors = {}
    input_dev_group = rospy.get_param("input_dev")
    for dev_name, dev_topic in input_dev_group.items():
        rospy.loginfo("Adding sensor %s" % dev_name)
        sensors[dev_name] = Sensor(pObjects = [dev_name], topic = dev_topic)
        # create a subscriber
        rospy.Subscriber(dev_topic, Range, controller.handleDistanceSensors, dev_name)

    # retrieve and process the outpu  group (dictionary) of parameters
    effectors = {}
    #output_dev_group = rospy.get_param("~output_dev")
    #for dev_name, dev_topic in output_dev_group.items():
        #effectors[dev_name] = Effector(pObjects = [dev_name], topic = dev_topic)
    output_cmd_vel = rospy.get_param("output_dev/cmd_vel")
    effectors["cmd_vel"] = EffectorDiffDriveMovement(pObjects = output_cmd_vel.keys(), topic = "cmd_vel")

    controller.interfaceWithDevices(sensors, effectors)

    while not rospy.is_shutdown():
        # if errors are encountered during Pep execution
        if (controller.runControlStep() == False):
            return
        rate.sleep()

if __name__ == '__main__':
    try:
        pep_controller()
    except rospy.ROSInterruptException:
        pass
