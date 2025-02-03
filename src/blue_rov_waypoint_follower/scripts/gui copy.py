#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, PoseArray
from tf.transformations import *
from tkinter import Tk, Label, Entry, Button, LabelFrame, LEFT

# initialize the waypoints list as empty
global goal_waypoints
goal_waypoints = PoseArray()
goal_waypoints.header.seq= 'base_frame'

global last_waypoint
last_waypoint = Pose()

class WaypointGui:
    def __init__(self, master):
        self.master = master
        self.master.title("Manage Waypoints")

        # Create two frames for waypoints
        self.waypoint1_frame = LabelFrame(master, text="Add a global waypoint to the waypoint list")
        self.waypoint1_frame.pack(padx=10, pady=10, fill="both", expand=True)

        self.waypoint2_frame = LabelFrame(master, text="Add a waypoint relative to the last waypoint")
        self.waypoint2_frame.pack(padx=10, pady=10, fill="both", expand=True)

        # Create fields for waypoint 1
        self.create_waypoint_fields(self.waypoint1_frame, '1')

        # Create fields for waypoint 2
        self.create_waypoint_fields(self.waypoint2_frame, '2')

        # publish the goal waypoints
        self.publish_goal_waypoints= Button(master, text="Update the plot", command=self.submit_goal_waypoints)
        self.publish_goal_waypoints.pack(side="top", fill='both', expand=True, padx=10, pady=10)


        # Initialize ROS node
        rospy.init_node('waypoint_gui', anonymous=True)
        self.pub1 = rospy.Publisher('add_new_goal', Pose, queue_size=10)
        self.pub2 = rospy.Publisher('waypoint_topic_2', Pose, queue_size=10)
        self.pub3 = rospy.Publisher('update_waypoint_goal_plot',PoseArray, queue_size=10)

    def create_waypoint_fields(self, frame, suffix):
        """ Helper function to create labeled entry widgets """
        x_label = Label(frame, text="X:")
        x_label.grid(row=0, column=0, padx=5, pady=5)
        setattr(self, f"x{suffix}_entry", Entry(frame))
        getattr(self, f"x{suffix}_entry").grid(row=0, column=1, padx=5, pady=5)

        y_label = Label(frame, text="Y:")
        y_label.grid(row=1, column=0, padx=5, pady=5)
        setattr(self, f"y{suffix}_entry", Entry(frame))
        getattr(self, f"y{suffix}_entry").grid(row=1, column=1, padx=5, pady=5)

        z_label = Label(frame, text="Z:")
        z_label.grid(row=2, column=0, padx=5, pady=5)
        setattr(self, f"z{suffix}_entry", Entry(frame))
        getattr(self, f"z{suffix}_entry").grid(row=2, column=1, padx=5, pady=5)

        roll_label = Label(frame, text="Roll:")
        roll_label.grid(row=3, column=0, padx=5, pady=5)
        setattr(self, f"roll{suffix}_entry", Entry(frame))
        getattr(self, f"roll{suffix}_entry").grid(row=3, column=1, padx=5, pady=5)

        pitch_label = Label(frame, text="Pitch:")
        pitch_label.grid(row=4, column=0, padx=5, pady=5)
        setattr(self, f"pitch{suffix}_entry", Entry(frame))
        getattr(self, f"pitch{suffix}_entry").grid(row=4, column=1, padx=5, pady=5)

        yaw_label = Label(frame, text="Yaw:")
        yaw_label.grid(row=5, column=0, padx=5, pady=5)
        setattr(self, f"yaw{suffix}_entry", Entry(frame))
        getattr(self, f"yaw{suffix}_entry").grid(row=5, column=1, padx=5, pady=5)

        submit_button = Button(frame, text=f"Submit Waypoint {suffix}", command=lambda: self.submit_waypoint(suffix))
        submit_button.grid(row=6, column=0, columnspan=2, pady=10)

    def submit_waypoint(self, suffix):
        """ Extract values from entries and publish them """
        try:
            x = float(getattr(self, f"x{suffix}_entry").get())
            y = float(getattr(self, f"y{suffix}_entry").get())
            z = float(getattr(self, f"z{suffix}_entry").get())
            roll = float(getattr(self, f"roll{suffix}_entry").get())
            pitch = float(getattr(self, f"pitch{suffix}_entry").get())
            yaw = float(getattr(self, f"yaw{suffix}_entry").get())

            # Create and publish the Pose message
            pose_msg = Pose()
            pose_msg.position.x = x
            pose_msg.position.y = y
            pose_msg.position.z = z

            # Convert from roll, pitch, yaw to quaternion
            quaternion = quaternion_from_euler(roll, pitch, yaw)
            pose_msg.orientation.x = quaternion[0]
            pose_msg.orientation.y = quaternion[1]
            pose_msg.orientation.z = quaternion[2]
            pose_msg.orientation.w = quaternion[3]

            # Publish to the correct topic
            if suffix == '1':
                self.pub1.publish(pose_msg)
                goal_waypoints.poses.append(pose_msg)
                
                
            elif suffix == '2':
                self.pub2.publish(pose_msg)
                goal_waypoints.poses.append(pose_msg)
                

            rospy.loginfo(f"Published waypoint {suffix}:\n {pose_msg}")
        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint {suffix}: {ve}")
    
    def submit_goal_waypoints(self):
        """ Publish the waypoints list to the plot when the button is pressed """
        if not goal_waypoints:
            rospy.logwarn("Waypoint array is empty")
            
        else:
            
            self.pub3.publish(goal_waypoints)
            print(goal_waypoints.poses)
            rospy.loginfo("Published goal waypoints array")


def main():
    root = Tk()
    gui = WaypointGui(root)
    root.mainloop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass