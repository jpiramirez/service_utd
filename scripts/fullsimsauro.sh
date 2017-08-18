#!/bin/bash

# 2 agents

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
		roslaunch service_utd urbansearch2agents.launch detection_radius:=5 collision_radius:=1 memsize:=1 plan_length:=1
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D5P1.txt
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
		roslaunch service_utd urbansearch2agents.launch detection_radius:=5 collision_radius:=5 memsize:=1 plan_length:=5
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D5P5.txt
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
		roslaunch service_utd urbansearch2agents.launch detection_radius:=100 collision_radius:=1 memsize:=1 plan_length:=1
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D100P1.txt
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
		roslaunch service_utd urbansearch2agents.launch detection_radius:=100 collision_radius:=5 memsize:=1 plan_length:=5
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D100P5.txt
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
		roslaunch service_utd urbansearch2agents.launch detection_radius:=20 collision_radius:=1 memsize:=1 plan_length:=1
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D20P1.txt
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
		roslaunch service_utd urbansearch2agents.launch detection_radius:=20 collision_radius:=5 memsize:=1 plan_length:=5
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D20P5.txt
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
		roslaunch service_utd urbansearch2agents.launch detection_radius:=5 collision_radius:=5 memsize:=1 plan_length:=3
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D5P3.txt
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
		roslaunch service_utd urbansearch2agents.launch detection_radius:=20 collision_radius:=5 memsize:=1 plan_length:=3
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D20P3.txt
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
		roslaunch service_utd urbansearch2agents.launch detection_radius:=100 collision_radius:=5 memsize:=1 plan_length:=3
	done
	killall roslaunch
	killall rviz
	killall roscore

done


cat ~/.ros/14*.txt > ~/D100P3.txt
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
		roslaunch service_utd urbansearch2agents.launch detection_radius:=0 collision_radius:=1 memsize:=1 plan_length:=0
	done
	killall roslaunch
	killall rviz
	killall roscore

done


cat ~/.ros/14*.txt > ~/RandWalk.txt
rm ~/.ros/*.txt

mkdir ~/exp2agents
mv ~/D*.txt ~/exp2agents
mv ~/RandWalk.txt ~/exp2agents

# 4 agents

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
		roslaunch service_utd urbansearch4agents.launch detection_radius:=5 collision_radius:=1 memsize:=1 plan_length:=1
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D5P1.txt
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
		roslaunch service_utd urbansearch4agents.launch detection_radius:=5 collision_radius:=5 memsize:=1 plan_length:=5
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D5P5.txt
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
		roslaunch service_utd urbansearch4agents.launch detection_radius:=100 collision_radius:=1 memsize:=1 plan_length:=1
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D100P1.txt
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
		roslaunch service_utd urbansearch4agents.launch detection_radius:=100 collision_radius:=5 memsize:=1 plan_length:=5
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D100P5.txt
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
		roslaunch service_utd urbansearch4agents.launch detection_radius:=20 collision_radius:=1 memsize:=1 plan_length:=1
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D20P1.txt
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
		roslaunch service_utd urbansearch4agents.launch detection_radius:=20 collision_radius:=5 memsize:=1 plan_length:=5
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D20P5.txt
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
		roslaunch service_utd urbansearch4agents.launch detection_radius:=5 collision_radius:=5 memsize:=1 plan_length:=3
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D5P3.txt
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
		roslaunch service_utd urbansearch4agents.launch detection_radius:=20 collision_radius:=5 memsize:=1 plan_length:=3
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D20P3.txt
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
		roslaunch service_utd urbansearch4agents.launch detection_radius:=100 collision_radius:=5 memsize:=1 plan_length:=3
	done
	killall roslaunch
	killall rviz
	killall roscore

done


cat ~/.ros/14*.txt > ~/D100P3.txt
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
		roslaunch service_utd urbansearch4agents.launch detection_radius:=0 collision_radius:=1 memsize:=1 plan_length:=0
	done
	killall roslaunch
	killall rviz
	killall roscore

done


cat ~/.ros/14*.txt > ~/RandWalk.txt
rm ~/.ros/*.txt

mkdir ~/exp4agents
mv ~/D*.txt ~/exp4agents
mv ~/RandWalk.txt ~/exp4agents

# 6 agents

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
		roslaunch service_utd urbansearch6agents.launch detection_radius:=5 collision_radius:=1 memsize:=1 plan_length:=1
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D5P1.txt
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
		roslaunch service_utd urbansearch6agents.launch detection_radius:=5 collision_radius:=5 memsize:=1 plan_length:=5
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D5P5.txt
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
		roslaunch service_utd urbansearch6agents.launch detection_radius:=100 collision_radius:=1 memsize:=1 plan_length:=1
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D100P1.txt
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
		roslaunch service_utd urbansearch6agents.launch detection_radius:=100 collision_radius:=5 memsize:=1 plan_length:=5
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D100P5.txt
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
		roslaunch service_utd urbansearch6agents.launch detection_radius:=20 collision_radius:=1 memsize:=1 plan_length:=1
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D20P1.txt
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
		roslaunch service_utd urbansearch6agents.launch detection_radius:=20 collision_radius:=5 memsize:=1 plan_length:=5
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D20P5.txt
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
		roslaunch service_utd urbansearch6agents.launch detection_radius:=5 collision_radius:=5 memsize:=1 plan_length:=3
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D5P3.txt
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
		roslaunch service_utd urbansearch6agents.launch detection_radius:=20 collision_radius:=5 memsize:=1 plan_length:=3
	done
	killall roslaunch
	killall rviz
	killall roscore

done

cat ~/.ros/14*.txt > ~/D20P3.txt
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
		roslaunch service_utd urbansearch6agents.launch detection_radius:=100 collision_radius:=5 memsize:=1 plan_length:=3
	done
	killall roslaunch
	killall rviz
	killall roscore

done


cat ~/.ros/14*.txt > ~/D100P3.txt
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
		roslaunch service_utd urbansearch6agents.launch detection_radius:=0 collision_radius:=1 memsize:=1 plan_length:=0
	done
	killall roslaunch
	killall rviz
	killall roscore

done


cat ~/.ros/14*.txt > ~/RandWalk.txt
rm ~/.ros/*.txt

mkdir ~/exp6agents
mv ~/D*.txt ~/exp6agents
mv ~/RandWalk.txt ~/exp6agents
