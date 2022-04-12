import rospy
from nav_msgs.msg import MapMetaData
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
import math, json, os, glob
import sys, select, os, subprocess, fnmatch, base64
from PIL import Image

class Map:
    #static variable
    map_metadata = None
    def __init__(self, path, format_type, extension):
        self.path = path
        self.format = format_type
        self.extension = extension
        self.pub = rospy.Publisher('/map_list', String, queue_size=0)
        self.map_process_id = None

    def start_map(self, name):
        yaml_name = self.path+name+".yaml"
        print(yaml_name)
        self.map_process_id = subprocess.Popen(['rosrun', 'map_server', 'map_server', yaml_name])

    def save_map(self, name):
        subprocess.Popen("rosrun map_server map_saver -f "+self.path+'/'+name, shell=True)
    
    def stop_map(self):
        try:
            self.map_process_id.terminate()
            self.map_process_id = None
        except:
            print('no running map server')

    def del_map(self, name):
        for f in glob.glob(self.path+name +".*"):
            os.remove(f);

    def metadata_callback(self, map_metadata):
        print(">>>>>MAP METADATA SET<<<<<")
        Map.map_metadata = map_metadata

    def map_listener(self):
        rospy.Subscriber('/map_metadata', MapMetaData, self.metadata_callback)

    def maplist_publisher(self):
        print('publishing map list')
        self.pub.publish( self.get_list() )
            
    def getBase64String(self, imageFile):
        return base64.b64encode(imageFile.read())

    def convertImgToBase64String(self, img_path):
        with open(img_path, "rb") as imageFile:
            encoded_img = self.getBase64String(imageFile)
        return encoded_img

    def get_list(self):
        listOfFiles = os.listdir(self.path)
        pattern = self.extension
        map_list = []
        for map_name in listOfFiles:
          if fnmatch.fnmatch(map_name, pattern):
            map_path = self.path+ map_name
            print(map_path)
            Image.open(map_path).save(map_path + ".png")
            # map_list[ "".join(map_name.split(".")[:-1]) ] = self.convertImgToBase64String(map_path + ".png")
            name = "".join(map_name.split(".")[:-1])
            img = self.convertImgToBase64String(map_path + ".png")
            map_list.append({ 'name' : name, 'img' : img })
        return str(map_list)
