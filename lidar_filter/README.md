# LIDAR Filter

Simple overview of use/purpose.
This filter is used for filtering out specific LIDAR data points from it original LIDAR data topic "/scan". Hence, it has three filtering functionality; angularFilterIn, angularFilterOut, minRangeFilter.

## Parameter configuartion

Parameters can be modified in the config/lidarFilter.yaml file.

```
#Angular Bound Filter In
angleFilterIn: false

#Angular Bound Filter Out
angleFilterOut: true

#Minimum Range Filter
minRangeFilter: true

#Parameters value
lowerAngle: 1.57
upperAngle: 4.71
minRange: 0.5
```

The specific filters can be enabled and disabled through the boolean input of true/ false.

All three parameter values take float. 

## Launch 

To launch the script run the following code or copy the launch node into custom launch file.

```
roslaunch lidar_filter lidarFilter.launch
```

## Filtered LIDAR data

The filtered data is published on the /filtered_scan topic. This topic needs to be subscribed by SLAM and Navigation in place of the /scan topic.