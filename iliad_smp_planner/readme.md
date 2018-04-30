This is a  ROS Global planner, made using carrot planner as base.

Upon a goal request (coming through move_base), it connects to
"get_path" service from iliad_smp to generate a global path.
 
In order to create a proper request to iliad_smp, it mimics some of the
behaviour of click_n_point and orunav_vehicle_execution_node nodes. 

It is subscribed to the map, and needs three parameters 
(load_operation,load_detect,robot_id).

It assumes flag use_vector_map_and_geofence_ to be false. 
This flag is from orunav_vehicle_execution_node. I let it to false, 
so the oru target generation is simpler. 


Services:
"get_path" from iliad_smp.

Topics 
"/map" from map-server ?

Params:
load_operation,load_detect,robot_id

Missing:
launcher including iliad_smp to generate trajectories, set parameters and move_base.
