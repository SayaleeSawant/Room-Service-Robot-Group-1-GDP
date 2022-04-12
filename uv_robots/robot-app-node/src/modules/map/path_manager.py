#! /usr/bin/env python
import rospy
import os
import json
from std_msgs.msg import String

class PathManager():
        def __init__(self, pathsDstn):
                self.pathsDstn = pathsDstn
                self.pub = rospy.Publisher('/map_path', String, queue_size=0)
                self.default_json ='{"paths":[], "docks":[]}'
                
        def pathPublisher(self, mapName):
                self.pub.publish(self.retrievePath(mapName))
                
        def createPath(self, pathData, pathName, mapName):
                self.mapPath = self.pathsDstn+mapName+'.json'

                try:
                        with open(self.mapPath, 'r') as f:
                                try:
                                        paths = json.load(f)
                                except ValueError:
                                        print("unable to decode json data creating empty json")
                                        paths = json.loads(self.default_json)
                except IOError:
                        print("File not accessible")
                        paths = json.loads(self.default_json)

                new_path = '{ "name": "'+pathName+'", "path": '+str(pathData) +'}'
                print(new_path)
                paths['paths'].append( json.loads(new_path))
                
                with open(self.mapPath, 'w') as f:
                        f.write(json.dumps(paths, sort_keys=True, indent=4, separators=(',', ': ')))
                        rospy.loginfo("Saved path data into: "+str(mapName)+".json")
                        
        def createDock(self, dockData, mapName):
                self.mapPath = self.pathsDstn+mapName+'.json'

                try:
                        with open(self.mapPath, 'r') as f:
                                try:
                                        paths = json.load(f)
                                except ValueError:
                                        print("unable to decode json data creating empty json")
                                        paths = json.loads(self.default_json)
                except IOError:
                        print("File not accessible")
                        paths = json.loads(self.default_json)
                #print(type(dockData["x"]))
                new_dock = '{ "x": '+str(dockData["x"])+', "y": '+str(dockData["y"])+ ', "angle": '+str(dockData["angle"]) +'}'
                print(new_dock)
                paths['docks'] = [json.loads(new_dock)]
                
                with open(self.mapPath, 'w') as f:
                        f.write(json.dumps(paths, sort_keys=True, indent=4, separators=(',', ': ')))
                        rospy.loginfo("Saved dock data into: "+str(mapName)+".json")
                        
        def retrievePath(self, mapName):
                self.mapPath = self.pathsDstn+mapName+'.json'
                paths = json.loads(self.default_json)
                try:
                        with open(self.mapPath) as f:
                        	paths = json.load(f)
                except IOError,ValueError:
                        print('No paths found for this map')
                return json.dumps(paths)
	
        def deletePath(self, mapName, pathName):
                self.mapPath = self.pathsDstn+mapName+'.json'

                new_paths = json.loads(self.default_json)
                try:
                        with open(self.mapPath, 'r') as f:
                                try:
                                        paths = json.load(f)
                                        for path in paths['paths']:
                                                if(path['name'] != pathName):
                                                        new_paths['paths'].append(path)
                                                
                                except ValueError, KeyError:
                                        print("path not found data failed to delete path")
                                      
                        with open(self.mapPath, 'w') as f:
                                f.write(json.dumps(new_paths, sort_keys=True, indent=4, separators=(',', ': ')))
                                rospy.loginfo("Saved path data into: "+str(pathName)+".json")
                        
                except IOError:
                        print('Unable to find' + mapName)

                                                              
	#Potential Feature
	#def editPath(self):
