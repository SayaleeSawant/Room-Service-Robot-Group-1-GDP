import tkinter as tk
from tkinter import ttk as ttk
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal

# Initializes a rospy node to let the SimpleActionClient publish and subscribe
rospy.init_node('movebase_client_py')

# Create an action client called "move_base" with action definition file "MoveBaseAction"
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

# Waits until the action server has started up and started listening for goals.
client.wait_for_server()

# Method linked to home button
def go_home():
    print "Heading home"

# Event handler for list update
def text_box_update(event):
    print "Navigation goal updated, current selection : ",list.get()

def go_dest():
    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # Move 0.5 meters forward along the x axis of the "map" coordinate frame
    goal.target_pose.pose.position.x = 0.5
    # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0
    client.send_goal(goal)
    wait = client.wait_for_result()
    print "Destination goal sent to robot"

room_list = {"3034":(0,0),"3032":(100,100),"3030":(200,200)}

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
list = ttk.Combobox(frame_room_list, values=room_list.keys(),textvariable=value)
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
