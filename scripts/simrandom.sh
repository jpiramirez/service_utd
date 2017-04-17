#!/bin/bash

rm ~/.ros/*.txt

for j in `seq 1 3`
do

	roscore &
	sleep 3
	roslaunch service_utd start_gazebo.launch &
	sleep 3
	rosrun rviz rviz &
	sleep 3
	for i in `seq 1 40`
	do
		roslaunch service_utd urban_search_sim_multiuav_batch.launch detection_radius:=0 collision_radius:=1 memsize:=1 plan_length:=0
	done
	killall roslaunch
	killall rviz
	killall roscore

done


cat ~/.ros/14*.txt > ~/RandWalk.txt
rm ~/.ros/*.txt
