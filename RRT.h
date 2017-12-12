#include "RRT_Prim.h"
#include "common.h"

const int max_tree_size = 10000; // Max tree size (to keep memory usage low)
const float lr_dist = 10.0f; // Laser range finder range

void start_RRT(Autopilot_Interface&, mavlink_mp_traj_t &, float [3], float [4]);
node* RRT_loop(node* root, Autopilot_Interface&, mavlink_mp_traj_t &); 
