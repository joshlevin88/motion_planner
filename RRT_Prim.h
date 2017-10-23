#include "types.h"
#include "common.h"
#include <vector>
#include <stack>
#include <queue>
#include <math.h>
#include <time.h>
#include <algorithm>
#include <stdio.h>

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

node* new_node(float, float, float, float, int, int, int, float, float, node*);
void rand_node_coord(node*, node*, int);
node* add_sibling(node*, node*);
node* add_child(node*, node*);
void trim_end_states(node*, node*, int, int, float);
node* extend_tree(node*, node*, node*, int);
node* steer_an(node*, node*);
node* steer_agile(node*, const agile_man_t);
disp disp_info(node*, node*);
bool collision(node*);
void free_tree(node**);
std::vector<list> near(node*, node*, int);
bool comp_prox(const list &, const list &);
bool comp_near(const list &, const list &);
bool comp_snear(const list &, const list &);
int tree_size(node*);
void add_to_near_vec(node*, node*, std::vector<list>*);
void prune_tree(node**, node*);
bool update_tree(node**, node*);
std::stack<node*> root_to_end(node*, node*);
void add_to_commit(node*);
ptr_to_DCM create_DCM(float, float, float);
float norm(node*, node*);
float norm(const float, const float, const float, const float);
float norm(const float, const float, const float, const float, const float, const float);
void create_world(const int);
bool goal_reached(node*, node*, int);
void rotate_arrvec(ptr_to_DCM, float[3]);
void limit_angle(float &); 
float hover_hdg(const float&, const float&, const float&);
bool intersection(const float, const float, const float, const float, const float, const float, const float, const float);
void update_tree_for_new_obstacles(node**);
void prune_new_obs_collisions(node**, node**, float);
bool out_of_world(node*);
bool inside_object(node*);
bool intersects_new_found_obs(node*, node*);
