#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray
from tf.transformations import quaternion_from_euler
from tkinter import Tk, Label, Entry, Button, LabelFrame, messagebox

class WaypointGui:
    def __init__(self, master):
        self.master = master
        self.master.title("Manage Waypoints")

        self.goal_waypoints = PoseArray()
        self.goal_waypoints.header.frame_id = 'base_frame'

        self.home_pose = Pose()

        # Create frames for waypoints
        self.waypoint1_frame = LabelFrame(master, text="Add a global waypoint to the waypoint list")
        self.waypoint1_frame.pack(padx=10, pady=10, fill="both", expand=True)

        self.waypoint2_frame = LabelFrame(master, text="Add a waypoint relative to the last waypoint")
        self.waypoint2_frame.pack(padx=10, pady=10, fill="both", expand=True)

        # Create fields for waypoint 1 and 2
        self.create_waypoint_fields(self.waypoint1_frame, '1')
        self.create_waypoint_fields(self.waypoint2_frame, '2')

        # Add Set Home Position Button
        self.home_button = Button(master, text="Set Home Position", command=self.set_home_position)
        self.home_button.pack(side="top", fill='both', expand=True, padx=10, pady=5)

        # Add go home Button (Clears waypoints and Adds Home)
        self.go_home_button = Button(master, text="Go Home", command=self.go_home)
        self.go_home_button.pack(side="top", fill='both', expand=True, padx=10, pady=5)
        
        # Add Erase Waypoints Button
        self.erase_waypoints_button = Button(master, text="Erase Waypoints", command=self.erase_waypoints)
        self.erase_waypoints_button.pack(side="top", fill='both', expand=True, padx=10, pady=5)

        # Publish button
        self.publish_goal_waypoints = Button(master, text="Visualize Waypoint List", command=self.visualize_waypoints)
        self.publish_goal_waypoints.pack(side="top", fill='both', expand=True, padx=10, pady=5)

        # # Initialize ROS node
        # self.pub1 = rospy.Publisher('new__global_goal', Pose, queue_size=10)
        # self.pub2 = rospy.Publisher('waypoint_topic_2', Pose, queue_size=10)
        self.pub3 = rospy.Publisher("waypoint_plot_viualization", PoseArray, queue_size=10)

    def create_waypoint_fields(self, frame, suffix):
        # fields = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        # self.entries = {}

        # for i, field in enumerate(fields):
        #     label = Label(frame, text=f"{field.capitalize()}:")
        #     label.grid(row=i, column=0, padx=5, pady=5)
        #     entry = Entry(frame)
        #     entry.grid(row=i, column=1, padx=5, pady=5)
        #     self.entries[f"{field}{suffix}"] = entry
        
        # submit_button = Button(frame, text=f"Submit Waypoint {suffix}", command=lambda: self.submit_waypoint(suffix))
        # submit_button.grid(row=len(fields), column=0, columnspan=2, pady=10)
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

    def set_home_position(self):
        """ Sets the home position """
        pose_msg = Pose()
        pose_msg.position.x = 0.0
        pose_msg.position.y = 0.0
        pose_msg.position.z = 0.0

        quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
        pose_msg.orientation.x = quaternion[0]
        pose_msg.orientation.y = quaternion[1]
        pose_msg.orientation.z = quaternion[2]
        pose_msg.orientation.w = quaternion[3]

        self.home_pose = pose_msg
        # self.pub1.publish(pose_msg)
        self.goal_waypoints.poses.append(pose_msg)
        rospy.loginfo("Set home position and published")

    def erase_waypoints(self):
        """ Erase the waypoints array """
        self.goal_waypoints.poses.clear()
        rospy.loginfo("Erased all waypoints")
        messagebox.showinfo("Success", "All waypoints have been erased.")

    def go_home(self):
        """ Clears waypoints and adds home position as the first waypoint """
        self.goal_waypoints.poses.clear()
        self.goal_waypoints.poses.append(self.home_pose)
        rospy.loginfo("Reset waypoints to home position")
        messagebox.showinfo("Success", "Waypoints have been reset to home position.")

    # need to add conversion to clobal frame if relative waypoint is given
    def submit_waypoint(self, suffix):
        """ Extract values from entries and publish them """
        try:
            x = float(getattr(self, f"x{suffix}_entry").get())
            y = float(getattr(self, f"y{suffix}_entry").get())
            z = float(getattr(self, f"z{suffix}_entry").get())
            roll = float(getattr(self, f"roll{suffix}_entry").get())*(np.pi / 180)
            pitch = float(getattr(self, f"pitch{suffix}_entry").get())*(np.pi / 180)
            yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)

            # # note: converts to radians
            # x = float(self.entries[f"x{suffix}"].get())
            # y = float(self.entries[f"y{suffix}"].get())
            # z = float(self.entries[f"z{suffix}"].get())
            # roll = float(self.entries[f"roll{suffix}"].get())
            # pitch = float(self.entries[f"pitch{suffix}"].get())
            # yaw = float(self.entries[f"yaw{suffix}"].get())
            # roll = float(self.entries[f"roll{suffix}"].get()*(np.pi / 180))
            # pitch = float(self.entries[f"pitch{suffix}"].get()*(np.pi / 180))
            # yaw = float(self.entries[f"yaw{suffix}"].get()*(np.pi / 180))
        
            pose_msg = Pose()
            pose_msg.position.x = x
            pose_msg.position.y = y
            pose_msg.position.z = z

            quaternion = quaternion_from_euler(roll, pitch, yaw)
            pose_msg.orientation.x = quaternion[0]
            pose_msg.orientation.y = quaternion[1]
            pose_msg.orientation.z = quaternion[2]
            pose_msg.orientation.w = quaternion[3]

            if suffix == '2':
                # need to convert to the global frame
                # self.pub2.publish(pose_msg)
                rospy.loginfo(f"need to convert this to global waypoint:\n {pose_msg}")
            
            self.goal_waypoints.poses.append(pose_msg)
            rospy.loginfo(f"Added waypoint to list:\n {pose_msg}")
            # rospy.loginfo(f"Published waypoint {suffix}:\n {pose_msg}")
        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint {suffix}: {ve}")
            messagebox.showerror("Input Error", f"Invalid input for waypoint: {ve}")

    def visualize_waypoints(self):
        """ Publish the waypoints list to the plot when the button is pressed """
        if not self.goal_waypoints.poses:
            rospy.logwarn("Waypoint array is empty")
        else:
            # rospy.loginfo("waypoints: \n" + self.goal_waypoints.poses)
            self.pub3.publish(self.goal_waypoints)
            rospy.loginfo("Published goal waypoints array")

def main():
    rospy.init_node('waypoint_gui', anonymous=True)
    root = Tk()
    gui = WaypointGui(root)
    root.mainloop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
