#include "types.h"
#include "common.h"
#include <vector>
#include <stack>
#include <queue>
#include <math.h>
#include <time.h>
#include <algorithm>
#include <stdio.h>
#include <string.h>

#define PI 3.1415f

const float V = 5.0f; // Flight velocity
const float bf = 2.0f; // Buffer around obstacles
const int max_tr_deg = 80; // Max turning rate
const int max_zr = 0; // Max climb rate
const float coll_dist = 0.2f; // Gap between collision checks
const float ATA_dist = 5.0f; // Distance required to do ATA
const float C2H_dist = 5.0f; // Distance required to do C2H
const float t_td = 0.23f; // Time-delay constant
const float t_int = 0.5f; // Planning horizon interval
const float ts = 0.5f; // Maximum time between nodes
const float ts_max = 1.0f; // Maximum time for a primitive
const int near_max = 10; // Maximum number of next near attempts
const int bias_freq = 30; // Frequency of selecting goal as "random" node
const float hdg_del_dome = 2.32f; // Heading of x-axis of Stinger Dome relative to NED frame

node* new_node(const float, const float, const float, const float, const int, 
	const int, const int, const float, const float, node* const);
void rand_node_coord(node*, const node* const, const int);
node* add_sibling(node*, node*);
node* add_child(node*, node*);
void trim_end_states(node*, const node* const, const int, const int, const float);
node* extend_tree(node*, node* const, node* const, const int);
node* steer_an(node* const, node* const);
node* steer_agile(node* const, const agile_man_t);
bool update_tree(node**, node* const);
disp disp_info(node* const, node* const);
bool collision(node* const);
bool out_of_world(node* const);
bool inside_object(node* const);
bool intersects_new_found_obs(node* const, node* const);
void free_tree(node**);
void prune_tree(node**, node* const);
std::vector<list> near(node* const, node* const, const int);
bool comp_prox(const list &, const list &);
bool comp_near(const list &, const list &);
bool comp_snear(const list &, const list &);
int tree_size(node* const);
void add_to_near_vec(node* const, node* const, std::vector<list>*);
std::stack<node*> root_to_end(node* const, node* const);
void add_to_commit(const node* const);
void add_to_commit_2(const node* const);
ptr_to_DCM create_DCM(const float, const float, const float);
float norm(node* const, node* const);
float norm(const float, const float, const float, const float);
float norm(const float, const float, const float, const float, const float, const float);
void create_world(const int);
bool goal_reached(node* const, node* const, const int);
void rotate_arrvec(ptr_to_DCM, float[]);
void limit_angle(float &); 
float hover_hdg(const float, const float, const float);
bool intersection(const float, const float, const float, const float, const float, const float, const float, const float);
void update_tree_for_new_obstacles(node**);
void prune_new_obs_collisions(node**, node**, const float);
node* initialize_world(const int nw, const float p_init[3], const float hdg_init);
node* initialize_world_2(const int nw, const float p_init[3], const float hdg_init);
void cleanup_tree_and_world(node** root);
