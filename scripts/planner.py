#!/usr/bin/env python
import rospy
import actionlib 
import numpy
import tf
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.srv import GetPlan

class Grid:
    """
    helper class to keep grid-specific details
    """    
    def __init__(self, occupancy_grid_data, width, height, resolution):
        self.grid = numpy.reshape(occupancy_grid_data, (height, width))
        self.resolution = resolution
        self.width = width
        self.height = height
        self.resolution = resolution
    
    def cell_at(self, x, y):
        return self.grid[y, x]

class Planner:
    def __init__(self, destinations = [], capacity = 10000):
        """
        Initializes a rospy node to let the SimpleActionClient publish and subscribe
        Sets up server and validates goal locaions. Keeps track of control center and capacity
        """
        rospy.init_node('move_to_goal_py') 
        msg = rospy.wait_for_message("map", OccupancyGrid)
        self.map = Grid(msg.data, msg.info.width, msg.info.height, msg.info.resolution)
        self.marker_publisher = rospy.Publisher("visualization_marker", Marker, queue_size=1)
        self.setup_server()
        self.destinations = destinations  
        self.capacity = capacity
        self.listener = tf.TransformListener()
        # self.validate_destinations() Assume destinations are valid
        self.control_center = self.get_current_position()
        self.curr_marker_id = 0
        self.make_deliveries()
        rospy.signal_shutdown("Delivery Complete!")
        
    
    def make_deliveries(self):
        """
        Makes deliveries to destinations ordered by set heuristic and accommodates for 
        robot's carrying capacity
        """
        package_number = 0
        curr_load = self.capacity
        while package_number < len(self.destinations):
            goal_x, goal_y = self.destinations[package_number]
            if curr_load == 0:
                result = self.move_to_goal(self.control_center.position[0], self.control_center.position[1])
                rospy.loginfo("RETURNING TO CONTROL CENTER")
                curr_load = self.capacity
            else:
                result = self.move_to_goal(goal_x, goal_y)
                rospy.loginfo("DELIVERED PACKAGE " + str(package_number))
                curr_load -= 1
            package_number += 1

        rospy.loginfo("COMPLETED ALL DELIVERIES")       
        if result:
            rospy.loginfo("Goal execution done!")
        else:
            rospy.loginfo("Something went wrong!")


    def validate_destinations(self):
        """
        filters out destinations with no a reachable path from the start location
        """
        rospy.loginfo("Checking validitiy of destinations...")
        print(self.map.cell_at(4.5, 0))


    def setup_server(self):
        """"
        Create an action client called "move_base" with action definition file "MoveBaseAction", 
        Waits until the action server has started up and started listening for goals.
        """
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()


    def move_to_goal(self, x, y):
        """
        Moves robot to a specified location within the map, and published the path traversed
        with markers.
        """        
        curr = self.get_current_position()
        start = PoseStamped()
        start.header.seq = 0
        start.header.frame_id = "map"
        start.header.stamp = rospy.Time(0)
        start.pose.position.x = curr.position[0]
        start.pose.position.y = curr.position[1]
       
       # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
      
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

       # Sends the goal to the action server.
        self.client.send_goal(goal)

       # Waits for the server to finish performing the action.
        wait = self.client.wait_for_result()

        get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        req = GetPlan()
        req.start = start
        req.goal = goal.target_pose
        req.tolerance = .5
        resp = get_plan(req.start, req.goal, req.tolerance)
        path = resp.plan.poses

        print("Path reached? : ", wait , "Path length", len(path))
        
        self.publish_path(path)

       # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
        # Result of executing the action
            return self.client.get_result()   


    def publish_path(self, path):
        """
        publishes path locations as markers 
        """
         
        if not path:
            print("No path found between", self.start, "and", self.goal)
            return
        for pose_stamp in path:
            self.publish_marker(pose_stamp, self.curr_marker_id)
            self.curr_marker_id += 1

    def publish_marker(self, curr_pose, id):
        """
        publishes marker to map, indicating path from start to goal
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.POINTS
        marker.id = id
        marker.action = marker.ADD 
        marker.pose = curr_pose
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0 
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_publisher.publish(marker)


    def get_current_position(self):
        """
        Returns the current pose of the robot in the map reference frame
        """
        current_pose = Pose()
        self.listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(5.0))
        (current_pose.position, current_pose.orientation) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        return current_pose


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        destinations = [[3.0, 3.5], [6.5, 4.9], [7.0, 1.2] , [4.5, 7.5], [7.8, 4.5]]
        plan = Planner(destinations=destinations)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
