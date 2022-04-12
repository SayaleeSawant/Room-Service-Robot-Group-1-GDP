import rosnode
import rospy, json
from std_msgs.msg import String
import time
from ..constants import nodes

class Feedback:
    node_status = { nodes.AMCL : False, "toast": "", "final_goal":False }
    def __init__(self):
        self.feedback_topic = rospy.Publisher('/feedback', String, queue_size=0)

    def feedback_publish(self):
        self.feedback_topic.publish(json.dumps(Feedback.node_status))
        print("Feedback Publish")

    def monitor_node(self, node_name):
        result = rosnode.rosnode_ping(node_name, max_count=1, verbose=False)
        try:
            if result != Feedback.node_status[node_name]:
        	Feedback.node_status[node_name] = result
        	self.feedback_publish()
        except KeyError:
            print('Unknown Node:' + node_name)

    def send_toast(self, message):
        Feedback.node_status["toast"] = message
        self.feedback_publish()
    
    def send_notif(self):
        Feedback.node_status["final_goal"] = True
        self.feedback_publish()
	Feedback.node_status["final_goal"] = False
