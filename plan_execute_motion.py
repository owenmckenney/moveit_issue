#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import PoseStamped, Pose
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_msgs.msg import PlanningScene, RobotState
from ament_index_python import get_package_share_directory


class UR5MotionPlanner(Node):
    def __init__(self):
        super().__init__('ur5_motion_planner')
        self.get_logger().info("Initializing UR5 Motion Planner Node")
        
        moveit_config_builder = MoveItConfigsBuilder("ur")
        moveit_config_builder.moveit_cpp(file_path=get_package_share_directory("ur_moveit_config") + "/config/moveit_cpp.yaml")
        self.moveit_config_dict = moveit_config_builder.to_moveit_configs().to_dict()
        
        time.sleep(1)
        
        self.ur5 = MoveItPy(
            node_name="moveit_py_planning_scene",
            config_dict=self.moveit_config_dict
        )
        
        self.ur5_arm = self.ur5.get_planning_component("ur_manipulator")
        self.planning_scene_monitor = self.ur5.get_planning_scene_monitor()
        self.planning_scene_pub = self.create_publisher(PlanningScene, "planning_scene", 10)
        
        self.get_logger().info("MoveItPy instance created")

        if not self.ur5_arm:
            self.get_logger().error("Failed to retrieve planning component 'ur_manipulator'")
            return

    def do_motion(self):
        self.ur5_arm.set_start_state_to_current_state()
        
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose.orientation.w = 1.0
        pose_goal.pose.position.x = 0.129
        pose_goal.pose.position.y = 0.432
        pose_goal.pose.position.z = 0.0
        
        self.ur5_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="tool0")
        
        plan_result = self.ur5_arm.plan() 
        
        if plan_result:
            self.get_logger().info("Plan found, executing trajectory")
            robot_trajectory = plan_result.trajectory
            success = self.ur5.execute(robot_trajectory, controllers=[])
            
            if success:
                self.get_logger().info("Trajectory executed successfully")
            else:
                self.get_logger().error("Trajectory execution failed")
                
        else:
            self.get_logger().error("Planning failed: no valid plan found")  

def main(args=None):
    rclpy.init()
    node = UR5MotionPlanner()
    
    node.do_motion()
    
    rclpy.spin_once(node, timeout_sec=10)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()