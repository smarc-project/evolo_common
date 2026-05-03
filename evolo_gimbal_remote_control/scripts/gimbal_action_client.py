#! /usr/bin/env python3

# This node listens to a json message on {command_topic} and
# calls different action servers depending on the command in that json
#
# json looks like this
# EULER
# {
#   "mode" : "EULER"
#   "pitch" : 3
#   "roll" : 3
#   "yaw" : 3
# }
# ros2 topic pub /cam_ctrl_input_topic std_msgs/msg/String data:\ \'\{\"mode\":\"EULER\",\"roll\":0.2,\"pitch\":0.3,\"yaw\":2.4}\'\
#
# POI
# {
#   "mode" : "GEO_POI"
#   "latitude" : 58
#   "longitude" : 18
#   "altitude" : 3
# }
#
# STOP
# {
#   "mode" : "STOP"
# }
# ros2 topic pub /cam_ctrl_input_topic std_msgs/msg/String data:\ \'\{\"mode\":\"STOP\"}\'\
#
#
#

import json
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from smarc_action_base.smarc_action_base import ActionClientState, ActionType
from wasp_bt.bt.client import BTActionClient
from smarc_msgs.action import BaseAction



class Camera_control_client:

    def __init__(self, node : Node):
        self._node = node

        # Json cmd
        self.json_cmd = None
        
        # Actionserver goal
        self._ac_goal = BaseAction.Goal()
        
        # Action client to be called with the goal
        self.active_ac = None

        self._node.declare_parameter('input_topic', "cam_ctrl_input_topic")
        input_topic = self._node.get_parameter('input_topic').value

        self.subscription_command = self._node.create_subscription(String,input_topic, self.command_callback,10)

        self.gimbal_set_rpy_ac =  BTActionClient(node=self._node, action_name='gimbal_set_rpy', action_type=ActionType(BaseAction))
        self.gimbal_set_geopoint_ac = BTActionClient(node=self._node, action_name='gimbal_set_geopoint', action_type=ActionType(BaseAction))
        self.gimbal_track_img_poi_ac = BTActionClient(node=self._node, action_name='gimbal_track_img_poi', action_type=ActionType(BaseAction))
        self.gimbal_track_odom_poi_ac = BTActionClient(node=self._node, action_name='gimbal_track_odom_poi', action_type=ActionType(BaseAction))
        self.gimbal_stop_ac = BTActionClient(node=self._node, action_name='gimbal_stop', action_type=ActionType(BaseAction))

        #TODO Yolo action servers?

        self._action_clients = [
                                self.gimbal_set_rpy_ac,
                                self.gimbal_set_geopoint_ac,
                                self.gimbal_track_img_poi_ac,
                                self.gimbal_track_odom_poi_ac,
                                self.gimbal_stop_ac
                                
        ]

        setup_success = self.setup()
        self._node.get_logger().info(f"Setup success: {setup_success}")

        #Run Action clients here for better luck with thread problems
        self.timer = self._node.create_timer(2.0,self.timer_callback)


    def timer_callback(self):
        if self.json_cmd != None:
            
            if("mode" in self.json_cmd.keys()):
                command_type = self.json_cmd["mode"]

                try:
                    #Set the goal and the action server to be called
                    if command_type == "EULER": #Euler callback
                        self._set_goal(self.gimbal_set_rpy_ac, {
                            "roll": self.json_cmd["roll"],
                            "pitch": self.json_cmd["pitch"],
                            "yaw": self.json_cmd["yaw"]
                        })
                    elif command_type == "GEO_POI": #POI callback
                        self._set_goal(self.gimbal_set_geopoint_ac, {
                            "latitude": self.json_cmd["latitude"],
                            "longitude": self.json_cmd["longitude"],
                            "altitude": self.json_cmd["altitude"]
                        })
                    elif command_type == "IMG_POI": #Image POI
                        pass
                        #self._set_goal(self.gimbal_track_img_poi_ac, {
                        #    "gain": 1.2
                        #})
                    elif command_type == "ODOM_POI": #Odom POI
                        pass
                        #self._set_goal(self.gimbal_track_odom_poi_ac, {
                        #    "gain": 1.2
                        #})
                    elif command_type == "STOP": #Stop
                        self._set_goal(self.gimbal_stop_ac, {})
                    else:
                        self._node.get_logger().error("Unknown camera command")
                except Exception as e:
                    self._node.get_logger().error(f"Falied to set goal: {e}")
        
        #Set json command to None so we don't call the same action server again next time
        self.json_cmd = None
        
        if(self.active_ac != None and self._goal != None):
            try:
                self.active_ac.send_goal(self._goal)
                self._node.get_logger().info(f"Set goal for {self.active_ac.action_type}.")
            except Exception as e:
                self._node.get_logger().error(f"Error sending goal to AC {self.active_ac.action_type} : {e}.")

        #Reset varaibles
        self.active_ac = None
        self._goal = None

        #Print action client status
        self._node.get_logger().info("------------------")    
        for _ac in self._action_clients:
            self._node.get_logger().info(f"Status of actionserver:  {_ac._action_name} : {_ac.state.name}.")    
            
    # Callback for json messages from mqtt
    def command_callback(self, msg : String):
        # Parse JSON and call action server
        self._node.get_logger().info(f"Received JSON command: {msg.data}")
        try:
            self.json_cmd = json.loads(msg.data)
        except Exception as e:
            self._node.get_logger().info(f"Faled to parse JSON command: {msg.data}")


    def setup(self) -> bool:
        self._node.get_logger().info("Setting up actions...")

        for ac in self._action_clients:
            ac._setup()
            if ac.state != ActionClientState.READY:
                self._node.get_logger().info(f"{ac.action_type} failed to setup! State: {str(ac.state)}")
                return False
        
        self._node.get_logger().info("All actions setup successfully!")
        return True


    def _set_goal(self, action_client: BTActionClient, goal_dict: dict) -> bool:
        try:
            self._goal = BaseAction.Goal()
            self._goal.goal.data = json.dumps(goal_dict)
            self.active_ac = action_client
            return True
        except Exception as e:
            self._node.get_logger().info(f"Failed to set goal for {action_client.action_type}: {e}")
            return False


def main(args=None, namespace=None):
    rclpy.init(args=args)

    _node = Node('evolo_camera_action_client')

    cameractrl = Camera_control_client(_node)
    
    executor = MultiThreadedExecutor()
    executor.add_node(_node)
    rclpy.spin(_node, executor=executor)
    rclpy.shutdown()

if __name__ == "__main__":
    main()



    

