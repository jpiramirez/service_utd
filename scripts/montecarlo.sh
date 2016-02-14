#!/bin/bash
for i in `seq 1 $1`
  do
    roslaunch service_utd urban_search_sim_multiuav_batch.launch
    sleep 10
  done
