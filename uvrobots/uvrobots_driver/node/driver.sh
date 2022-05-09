#!/bin/bash
echo "Starting UV Robots - Driver"
echo "Checking Arduino Ports"

PORT_COUNT=0
PORT_ARRAY=()
for i in $(seq 1 $No_Of_PCB);
do
	if [ -z `python -m serial.tools.list_ports 1A86:7523 -q -n $i` ]
	then
		((PORT_COUNT = PORT_COUNT + 0));
	else
		PORT_ARRAY+=("`python -m serial.tools.list_ports 1A86:7523 -q -n $i`");
		((PORT_COUNT = PORT_COUNT + 1));
	fi
done

echo "Arduino active on these ports: ${PORT_ARRAY[@]}" 

#Give root access to Arduino PCB ports
for i in ${!PORT_ARRAY[@]};
do
	count=$(($i-1));
	sudo chmod 666 ${PORT_ARRAY[$count]};
done


#Run ROS serial Arduino
echo "ROSSerial-Arduino running in ports: ${PORT_ARRAY[@]}"

if [ $PORT_COUNT -eq 1 ]
then
	echo "Expected number of active ports: $No_Of_PCB Actual port count: $PORT_COUNT";
	echo "PORT1: ${PORT_ARRAY[0]}";
	roslaunch uvrobots_driver arduino_access.launch port_count:=$PORT_COUNT port1:=${PORT_ARRAY[0]};
elif [ $PORT_COUNT -eq 2 ]
then
	echo "Expected number of active ports: $No_Of_PCB Actual port count: $PORT_COUNT";
	echo "PORT1: ${PORT_ARRAY[0]} PORT2: ${PORT_ARRAY[1]}";
	roslaunch uvrobots_driver arduino_access.launch port_count:=$PORT_COUNT port1:=${PORT_ARRAY[0]} port2:=${PORT_ARRAY[1]};
elif [ $PORT_COUNT -eq 3 ]
then
	echo "Expected number of active ports: $No_Of_PCB Actual port count: $PORT_COUNT";
	echo "PORT1: ${PORT_ARRAY[0]} PORT2: ${PORT_ARRAY[1]} PORT3: ${PORT_ARRAY[2]}";
	roslaunch uvrobots_driver arduino_access.launch port_count:=$PORT_COUNT port1:=${PORT_ARRAY[0]} port2:=${PORT_ARRAY[1]} port3:=${PORT_ARRAY[2]};
else
	echo "Expected number of active ports: $No_Of_PCB Actual port count: $PORT_COUNT";
fi
