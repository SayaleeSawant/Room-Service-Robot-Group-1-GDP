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

room_list = {"3036":(155,183),"3038":(150,185),"3040":(148,185),"3042":(143,187),"3044":(141,187),"3046":(135,187),"3048":(134,188),"3050":(128,188),"3052":(127,188),"3054":(122,189),"3056":(120,189),"3060":(90,192),"3062":(88,192),"3064":(84,193),"3066":(82,193),"3068":(82,193),"3070":(75,194),"3072":(69.7,194),"3074":(68.7,194),"3076":(62,194),"3078":(61,194),"3080":(55.5,193),"3082":(48.7,193),"3027":(180,158),"3025":(180,157),"3023":(179.5,151),"3021":(179,150),"3022":(180,150),"3024":(180,150),"3020":(180,146),"3018":(180,145),"3019":(179,146),"3017":(179,145),"3016":(180,140),"3014":(180,139),"3015":(178,140),"3013":(179,139),"3011":(179,135),"3009":(179,133),"3008":(178,129),"3006":(179,128),"3007":(178,129),"3005":(178,128),"3004":(179,123),"3002":(179,122),"3000":(179,118),"3244":(177,106),"3242":(180,79.6),"3240":(176,101),"3239":(175,102),"3235":(174,95),"3236":(175,95),"3237":(174,96),"3238":(174,96),"3233":(172,90),"3231":(172,90),"3229":(171,85),"3227":(171,84),"3230":(172,85),"3225":(169,79),"3226":(170,79),"3227":(169,78),"3228":(170,78),"3222":(169,74),"3220":(169,73),"3219":(168,73),"3217":(167,68),"3215":(166,67),"3213":(163,61),"3211":(163,61)}
home_pos = (166,177)

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
