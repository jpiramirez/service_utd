#!/bin/bash
for i in `seq 1 $1`
  do
    # Bringing down the controllers to avoid having windup of the inputs
    rosservice call /uav1/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['controller/twist'], strictness: 2}"
    rosservice call /uav2/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['controller/twist'], strictness: 2}"
    rosservice call /uav3/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['controller/twist'], strictness: 2}"
    rosservice call /uav4/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['controller/twist'], strictness: 2}"
    rosservice call /uav1/controller_manager/unload_controller "name: 'controller/twist'"
    rosservice call /uav2/controller_manager/unload_controller "name: 'controller/twist'"
    rosservice call /uav3/controller_manager/unload_controller "name: 'controller/twist'"
    rosservice call /uav4/controller_manager/unload_controller "name: 'controller/twist'"

    # Returning the robots to their original positions
    rosservice call /gazebo/reset_world

    # Bringing up the controllers again
    rosservice call /uav1/controller_manager/load_controller "name: 'controller/twist'"
    rosservice call /uav2/controller_manager/load_controller "name: 'controller/twist'"
    rosservice call /uav3/controller_manager/load_controller "name: 'controller/twist'"
    rosservice call /uav4/controller_manager/load_controller "name: 'controller/twist'"

    POSITION=`rosrun service_utd randvertex.py`
    # Positioning the ground robot at a random road intersection
    #rosservice call /gazebo/set_model_state '{model_state: { model_name: p3dx, pose: { position: '"$POSITION"', orientation: {x: 0, y: 0, z: 0, w: 0 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'
    TIMES=`date +%s`
    roslaunch service_utd urban_search_sim_multiuav_batch.launch
    TIMEE=`date +%s`
    echo $(($TIMEE-$TIMES)) >> times.txt
    #sleep 5


  done
