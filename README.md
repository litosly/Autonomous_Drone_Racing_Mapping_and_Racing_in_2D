# Autonomous_Drone_Racing_Mapping_and_Racing_in_2D
2D mapping and planning

Kinematic framework:
class drone:

initial input: 
1. start xy position and angle theta. (theta start from pointing to the right and clock-wise)
2. initial map data
3. print_info print some more detailed info

func:
get_cur_pos: get current x, y, theta
get_cur_map: get drone map in np array
plot_cur_map: plot drone map
get_cur_time: get current elapsed time
get_traj: get trajectory (travelled positions)

update_map: main update function, feed in xy speed and angle, has speed and angular velocity limit,
          update the position of drone, the map drone sees and maintained other vairables specified above.
          
Current bug: 
