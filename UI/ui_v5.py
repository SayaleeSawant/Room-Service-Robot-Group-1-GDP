# Tkinter imports
import tkinter as tk
from tkinter import ttk as ttk
# ROS imports
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal

# Initializes a rospy node to let the SimpleActionClient publish and subscribe
rospy.init_node('movebase_client_py')

# Create an action client called "move_base" with action definition file "MoveBaseAction"
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

# Waits until the action server has started up and started listening for goals.
client.wait_for_server()

# Function linked to home button
def go_home():
    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # Move 0.5 meters forward along the x axis of the "map" coordinate frame
    goal.target_pose.pose.position.x = home_pos[0]
    goal.target_pose.pose.position.y = home_pos[1]
    # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0
    # Send nav goal to server
    client.send_goal(goal)
    print "Heading home"

# Event handler for list update
def text_box_update(event):
    print "Navigation goal updated, current selection : ",list.get()

# Function linked to Go dest buton
def go_dest():
    room_number = list.get()
    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # Move 0.5 meters forward along the x axis of the "map" coordinate frame
    goal.target_pose.pose.position.x = room_list[room_number][0]
    goal.target_pose.pose.position.y = room_list[room_number][1]
    # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0
    # Send nav goal to server
    client.send_goal(goal)
    # wait = client.wait_for_result()
    print "Destination goal sent to robot"

room_list = {"3000":(179,102), "3002":(179,108), "3004":(179,109), "3005":(178,115), "3006":(178,115), "3007":(178,116), "3008":(179,116), "3009":(177,122), "3011":(177,124), "3013":(177,130), "3014":(179,130), "3015":(177,131), "3016":(179,131), "3017":(177,137), "3018":(179,137), "3019":(177,138), "3020":(179,138), "3021":(177,145), "3022":(179,145), "3023":(177,146), "3024":(179,146), "3025":(177,153), "3027":(177,154),  "3034":(157,181), "3036":(151,183), "3038":(146,186), "3040":(145,187), "3042":(139,188), "3044":(137,189), "3046":(133,189), "3048":(131,189), "3050":(125,189), "3052":(124,189), "3054":(118,188), "3056":(117,189), "3211":(177,26.5), "3213":(177,27.1), "3215":(178,34.2), "3217":(179,35.7), "3219":(178,42.5), "3220":(180,42.4), "3221":(178,43.9), "3222":(180,43.8), "3223":(178,49.6), "3224":(180,49.7), "3225":(178,50.8), "3226":(180,50.5), "3227":(178,56.8), "3228":(180,56.7), "3229":(178,57.9), "3230":(180,58), "3231":(178,63.9), "3233":(178,65.2), "3235":(178,71.3), "3236":(180,71.3), "3237":(178,72.5), "3238":(180,72.5), "3239":(178,78.4), "3240":(180,78.4), "3242":(180,79.4), "3244":(180,85.6)}

home_pos = (179,94)

## UI
window = tk.Tk()

# Welcome message
frame_welcome = tk.Frame()
# Home button frame
frame_home_but = tk.Frame()
# Room list frame
frame_room_list = tk.Frame()
# Robot status frame
frame_robot_status = tk.Frame()
# Go button
frame_dest_but = tk.Frame()

# Welcome Message
greeting = tk.Label(text="Welcome to group 1 robot interface",master=frame_welcome)
greeting.pack()

# Go Home button
home_button = tk.Button(
    text="Go Home",
    width=25,
    height=5,
    bg="blue",
    fg="white",
    command = go_home,
    master=frame_home_but
)
home_button.pack()

# Go to destination button
dest_button = tk.Button(
    text="Go",
    width=25,
    height=5,
    bg="purple",
    fg="white",
    command = go_dest,
    master = frame_dest_but
)
dest_button.pack()

# Room list
value = tk.StringVar()
list = ttk.Combobox(frame_room_list, values=sorted(room_list.keys(),key=int),textvariable=value)
list.bind("<<ComboboxSelected>>",text_box_update)
list.set("Select destination")
list.pack()

# Robot status
status = tk.Message(master=frame_robot_status,
    text="Robot status : ok",
    fg="green")
status.pack()

# Packing frames, order matters
frame_welcome.pack(side=tk.TOP)
frame_robot_status.pack()
frame_home_but.pack(side=tk.LEFT)
frame_room_list.pack(side=tk.LEFT)
frame_dest_but.pack(side=tk.LEFT)
# Display
window.mainloop()
