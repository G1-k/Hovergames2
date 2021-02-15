#!/usr/bin/env python

# ROS Node - Action Server - ROS-IOT Bridge
"""
Import Modules
"""
import threading
import rospy
import actionlib


from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotResult
from pkg_ros_iot_bridge.msg import msgMqttSub

from pyiot import iot


class RosIotBridgeActionServer(object):
    """
    Class to Bridge ROS and IOT
    """

    def __init__(self):
        """
        Constructor
        """
        # Initialize action server
        self._as = actionlib.ActionServer('/action_ros_iot',
                                          msgRosIotAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)

        '''
        * self.on_goal - points to a function which will be called
                          when the Action Server receives a Goal.

        * self.on_cancel - points to a function which will be called
                          when the Action Server receives a Cancel Request.
        '''

        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        self._config_spread_sheet_id = param_config_iot['google_apps']['spread_sheet_id']
        print(param_config_iot)

        # Initialize ROS Topic Publication
        # Mqtt Messages published to ROS topic
        self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_ros_topic,
                                               msgMqttSub,
                                               queue_size=10)

        # Subscribe to Mqtt topic
        ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                              self._config_mqtt_server_url,
                                              self._config_mqtt_server_port,
                                              self._config_mqtt_sub_topic,
                                              self._config_mqtt_qos)
        if(ret == 0):
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")

        # Start the Action Server
        self._as.start()

        rospy.loginfo("Started ROS-IoT Bridge Action Server.")

    def mqtt_sub_callback(self, client, userdata, message):
        """
        This is a callback function for MQTT Subscriptions
        """
        payload = str(message.payload.decode("utf-8"))

        print("[MQTT SUB CB] Message: ", payload)
        print("[MQTT SUB CB] Topic: ", message.topic)

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload

        self._handle_ros_pub.publish(msg_mqtt_sub)

    def on_goal(self, goal_handle):
        """
        This function will be called when Action Server receives a Goal
        """

        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)

        # Validate incoming goal parameters
        if(goal.protocol == "mqtt"):

            if((goal.mode == "pub") or (goal.mode == "sub")):
                goal_handle.set_accepted()

                # Start a new thread to process new MQTT goal from the client
                thread = threading.Thread(name="worker",
                                          target=self.process_mqtt_goal,
                                          args=(goal_handle,))
                thread.start()

        elif(goal.protocol == "http"):

            goal_handle.set_accepted()

            # Start a new thread to process new HTTP goal from the client
            thread2 = threading.Thread(name="worker2",
                                       target=self.process_http_goal,
                                       args=(goal_handle,))

            thread2.start()

        else:
            goal_handle.set_rejected()
            return

    def process_mqtt_goal(self, goal_handle):
        """
        This function is called as a separate thread to process Mqtt Goal
        """

        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()

        # Goal Processing
        if(goal.protocol == "mqtt"):
            rospy.logwarn("MQTT")

            if(goal.mode == "pub"):
                rospy.logwarn("MQTT PUB Goal ID: " + str(goal_id.id))

                rospy.logwarn(goal.topic + " > " + goal.message)

                ret = iot.mqtt_publish(self._config_mqtt_server_url,
                                       self._config_mqtt_server_port,
                                       goal.topic,
                                       goal.message,
                                       self._config_mqtt_qos)

                if(ret == 0):
                    rospy.loginfo("MQTT Publish Successful.")
                    result.flag_success = True
                else:
                    rospy.logerr("MQTT Failed to Publish")
                    result.flag_success = False

            elif(goal.mode == "sub"):
                rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic)

                ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                                      self._config_mqtt_server_url,
                                                      self._config_mqtt_server_port,
                                                      goal.topic,
                                                      self._config_mqtt_qos)
                if(ret == 0):
                    rospy.loginfo("MQTT Subscribe Thread Started")
                    result.flag_success = True
                else:
                    rospy.logerr("Failed to start MQTT Subscribe Thread")
                    result.flag_success = False

        rospy.loginfo("Send Mqtt goal result to client")
        if (result.flag_success is True):
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Mqtt Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

    def process_http_goal(self, goal_handle):
        """
        This function is called as a separate thread to process HTTP Goal
        """
        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing HTTP goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()

        if(goal.protocol == "http"):
            rospy.logwarn("HTTP")
            rospy.logwarn("HTTP Goal ID: " + str(goal_id.id))

            # Convert the goal payload to dictionary
            http_parameters = eval(goal.message.replace("[", "{").replace("]", "}"))
            # Get the spreadsheet id from goal topic
            spreadsheet_url = 'https://script.google.com/macros/s/'+goal.topic+'/exec'

            ret = iot.http_get(spreadsheet_url, **http_parameters)

            if(ret == 0):
                rospy.loginfo("HTTP Thread Started")
                result.flag_success = True
            else:
                rospy.logerr("Failed to start HTTP Thread")
                result.flag_success = False

        rospy.loginfo("Send goal result to client")
        if (result.flag_success is True):
            rospy.loginfo("HTTP Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("HTTP Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + "HTTP Goal Processing Done.")

    def on_cancel(self, goal_handle):
        """
        Called when Goal Cancel request is send to the Action Server
        """
        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()


def main():
    """
    Main Function
    """
    # Initialize ROS Node
    rospy.init_node('node_ros_iot_bridge_action_server')

    # Create Action Server Object
    action_server = RosIotBridgeActionServer()

    rospy.spin()


if __name__ == '__main__':
    main()
