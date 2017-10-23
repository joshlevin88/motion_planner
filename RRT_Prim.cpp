#include "RRT_Prim.h"

std::queue<std::vector<float> > new_obs; // Unknown obstacles found via laser range finder 
std::vector<std::vector<float> > found_obs; // Unknown obstacles found via laser range finder 

// Allocate dynamic memory and initialize new node
node* new_node(float coord_x, float coord_y, float coord_z, float hdg, int tr_deg, int zr, int type, float t, float cost, node* parent)
{
	node* new_node = (node*)malloc(sizeof(node));

	if (new_node) {
		new_node->coord[0] = coord_x;
		new_node->coord[1] = coord_y;
		new_node->coord[2] = coord_z;
		new_node->hdg = hdg;
		new_node->tr_deg = tr_deg;
		new_node->zr = zr;
		new_node->type = type;
		new_node->t = t;
		new_node->cost = cost;
		new_node->next = NULL;
		new_node->child = NULL;
		new_node->parent = parent;
	}

	return new_node;
}

// Generate random node (revisit because this will create garbage)
void rand_node_coord(node* rand_node, node* goal_node, int iter)
{
	// Every bias_freq iterations, return goal node
	if (iter % bias_freq == 0) {
		rand_node->coord[0] = goal_node->coord[0];
		rand_node->coord[1] = goal_node->coord[1];
		rand_node->coord[2] = goal_node->coord[2];
	}
	else{
		rand_node->coord[0] = static_cast <float> (rand() % 30000) / 30000.0f * w.lims[0];
		rand_node->coord[1] = static_cast <float> (rand() % 30000) / 30000.0f * w.lims[1];
		rand_node->coord[2] = static_cast <float> (rand() % 30000) / 30000.0f * w.lims[2];
	}
}

// Add node to tree
node* add_child(node* existing_parent, node* new_child)
{
	if (existing_parent == NULL) return NULL;

	if (existing_parent->child) return add_sibling(existing_parent->child, new_child);

	else{
		new_child->parent = existing_parent;
		return existing_parent->child = new_child;
	}
}

// Add node as sibling if parent node already has a child
node* add_sibling(node* existing_child, node* new_sibling)
{
	if (existing_child == NULL) return NULL;

	while (existing_child->next) existing_child = existing_child->next;

	new_sibling->parent = existing_child->parent;
	return existing_child->next = new_sibling;
}

// Calculate trim states (coordinates and heading) at end of primitive
void trim_end_states(node* current, node* from, int tr_deg, int zr, float dt)
{
	float tr_rad = -tr_deg*PI / 180.0f;

	// If straight 
	if (tr_deg == 0){
		current->coord[0] = from->coord[0] + V*dt*cosf(from->hdg)*cosf(asinf(zr / V));
		current->coord[1] = from->coord[1] + V*dt*sinf(from->hdg)*cosf(asinf(zr / V));
		current->coord[2] = from->coord[2] + zr*dt;
	}
	// If turning
	else{
		current->coord[0] = from->coord[0] + (-V / tr_rad*sinf(from->hdg - tr_rad*dt) + V / tr_rad*sinf(from->hdg))*cosf(asinf(zr / V));
		current->coord[1] = from->coord[1] + (V / tr_rad*cosf(from->hdg - tr_rad*dt) - V / tr_rad*cosf(from->hdg))*cosf(asinf(zr / V));
		current->coord[2] = from->coord[2] + zr*dt;
	}

	// Calculate heading and keep between -PI and PI
	current->hdg = from->hdg - tr_rad*dt;
	limit_angle(current->hdg);
}

// Extend tree
node* extend_tree(node* root, node* rand_node, node* goal, int gn)
{
	std::vector<list> near_vec;
	node* prim_root = NULL;
	node* prim_end = NULL;

	// Nearness criteria depends on whether feasible path has been found
	int n;
	if (!path_found) n = 0;
	else n = 2;

	// Get vector of nodes orderered by nearness
	near_vec = near(root, rand_node, n);

	int i = 0;
	int m = static_cast<int>(near_vec.size());

	// Look for a desirable primitive over a maximum number of nearest nodes
	while (i < std::min(near_max, m)){

		// Do not extend tree from a hover
		if (near_vec[i].n->type == 3){
			i++;
			continue;
		}

		// If the nearest node is within range of the final goal, only try C2H maneuver
		float D2G = sqrtf(powf(w.goals[w.n_goals - 1][0] - near_vec[i].n->coord[0], 2) + powf(w.goals[w.n_goals - 1][1] - near_vec[i].n->coord[1], 2) + powf(w.goals[w.n_goals - 1][2] - near_vec[i].n->coord[2], 2));
		float ngr_1 = C2H_dist + V*t_td + w.goals[w.n_goals - 1][4];

		if (D2G <= ngr_1){
			prim_root = steer_agile(near_vec[i].n, C2H);
			// Add C2H maneuver if there is no collision and it ends in goal region
			D2G = sqrtf(powf(w.goals[w.n_goals - 1][0] - prim_root->child->coord[0], 2) + powf(w.goals[w.n_goals - 1][1] - prim_root->child->coord[1], 2) + powf(w.goals[w.n_goals - 1][2] - prim_root->child->coord[2], 2));
			if (!collision(prim_root) && D2G <= w.goals[gn][4]){
				add_child(near_vec[i].n, prim_root);
				break;
			}
			else free_tree(&prim_root);
		}
		// Else, try normal primitive
		else{
			prim_root = steer_an(near_vec[i].n, rand_node);
			// If there is a collision on the first try, attempt ATA
			if (collision(prim_root) && i == 0){
				free_tree(&prim_root);
				prim_root = steer_agile(near_vec[i].n, ATA);
			}

			//If no collision, extend tree with primitive (unless it ends too close to goal region)
			prim_end = prim_root;
			while (prim_end->child != NULL){
				prim_end = prim_end->child;
			}

			D2G = sqrtf(powf(w.goals[w.n_goals - 1][0] - prim_end->coord[0], 2) + powf(w.goals[w.n_goals - 1][1] - prim_end->coord[1], 2) + powf(w.goals[w.n_goals - 1][2] - prim_end->coord[2], 2));
			float ngr_2 = C2H_dist + V*t_td - w.goals[w.n_goals - 1][4];

			if (!collision(prim_root) && D2G > ngr_2){
				add_child(near_vec[i].n, prim_root);
				break;
			}
			else free_tree(&prim_root);
		}

		i++;
	}

	return prim_root;
}

// Generate maneuver primitive steering towards node
node* steer_an(node* from, node* towards)
{
	int tr_deg, zr;
	float L;
	node* temp = NULL;

	// First node of primtive is time-delayed node
	node* td = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 1, from->t + t_td, 0.0f, from);
	trim_end_states(td, from, from->tr_deg, from->zr, t_td);
	disp d = disp_info(from, td);
	td->cost = from->cost + d.cost;

	// Find best trim primitive and time spent along it
	float r_G = norm(td, towards);
	float theta_G = atan2f(towards->coord[1] - td->coord[1], towards->coord[0] - td->coord[0]) - td->hdg;

	limit_angle(theta_G);

	float r_c = sqrtf(powf(towards->coord[0] - td->coord[0], 2) + powf(towards->coord[1] - td->coord[1], 2)) / (2 * sinf(theta_G));

	if (fabsf(theta_G) <= 0.01f) L = r_G; // if theta_G == 0
	else L = r_G*theta_G / sinf(theta_G); // if theta_G != 0

	float dt_max = std::min((L / V), ts_max); // Max time on primitive must not be greater than const ts_max
	float dt = std::min(ts, dt_max); // Time step must not be greater than dt_max;

	tr_deg = int(V / r_c * 180.0f / PI);
	tr_deg = ((tr_deg + 10 / 2) / 10) * 10; // Round turning rate to nearest 10 deg
	zr = int((towards->coord[2] - td->coord[2]) / dt_max);

	// Keep turning and climb/descent rate within allowable bounds
	if (tr_deg > max_tr_deg) tr_deg = max_tr_deg;
	else if (tr_deg < -max_tr_deg) tr_deg = -max_tr_deg;
	if (zr > max_zr) zr = max_zr;
	else if (zr < -max_zr) zr = -max_zr;

	// Assign primitive's turning and climb/descent rate to time-delayed node
	td->tr_deg = tr_deg;
	td->zr = zr;

	// Add intermediate nodes along trim primitive to tree
	node* last = td;
	while (dt < dt_max){
		temp = new_node(0.0f, 0.0f, 0.0f, 0.0f, tr_deg, zr, 0, td->t + dt, 0.0f, last);
		trim_end_states(temp, td, tr_deg, zr, dt);
		temp->cost = last->cost + disp_info(last, temp).cost;
		add_child(last, temp);

		last = temp;
		dt += ts;
	}

	// For dt = dt_max
	temp = new_node(0.0f, 0.0f, 0.0f, 0.0f, tr_deg, zr, 0, td->t + dt_max, 0.0f, last);
	trim_end_states(temp, td, tr_deg, zr, dt_max);
	temp->cost = last->cost + disp_info(last, temp).cost;
	add_child(last, temp);


	return td; // Time-delayed node is root of primitive
}

// Generate agile maneuver primitive 
node* steer_agile(node* from, const agile_man_t agile_man)
{
	/*
	// Optimized ATA and C2H maneuver trajectories
	float dx_man[14], dy_man[14], dz_man[14], t_man[14];

	float dx_ATA[14] = { 1.3831f, 2.4070f, 3.2652f, 4.1171f, 4.7425f, 4.9805f, 5.0326f, 4.8323f, 4.2057f, 3.4872f, 2.6745f, 1.5137f, 0.4182f, 0.0f };
	float dy_ATA[14] = { 0.0041f, 0.0370f, 0.1092f, 0.2218f, 0.3066f, 0.3109f, 0.2524f, 0.1077f, -0.0235f, -0.0477f, -0.0339f, -0.0077f, -0.0002f, 0.0f };
	float dz_ATA[14] = { 0.0012f, 0.0373f, 0.1580f, 0.4257f, 0.7802f, 1.0111f, 1.1569f, 1.1725f, 0.8915f, 0.5674f, 0.2876f, 0.0526f, -0.0014f, 0.0f };
	float t_ATA[14] = { 0.1971f, 0.3439f, 0.4784f, 0.6487f, 0.8458f, 0.9926f, 1.1271f, 1.2974f, 1.4945f, 1.6413f, 1.7758f, 1.9461f, 2.1024f, 2.1623f };

	float dx_C2H[14] = { 0.2538f, 1.1928f, 2.2255f, 2.8981f, 3.4204f, 3.9541f, 4.4108f, 4.6521f, 4.8088f, 4.9322f, 4.9922f, 5.0000f, 4.9961f, 4.9930f };
	float dy_C2H[14] = { 0.0f };
	float dz_C2H[14] = { -0.0002f, 0.0007f, 0.0677f, 0.1874f, 0.3305f, 0.5262f, 0.7360f, 0.8648f, 0.9564f, 1.0360f, 1.0851f, 1.1011f, 1.1070f, 1.1084f };
	float t_C2H[14] = { 0.0362f, 0.1707f, 0.3263f, 0.4422f, 0.5483f, 0.6828f, 0.8384f, 0.9543f, 1.0604f, 1.1949f, 1.3505f, 1.4664f, 1.5725f, 1.7070f };

	float dx_H2C[14] = { 0.0000f, 0.0000f, 0.0001f, 0.0000f, 0.0100f, 0.0745f, 0.2653f, 0.4949f, 0.7648f, 1.1751f, 1.7188f, 2.1514f, 2.5566f, 3.0763f };
	float dy_H2C[14] = { 0.0f };
	float dz_H2C[14] = { 0.0000f, -0.0030f, -0.0195f, -0.0447f, -0.0774f, -0.1237f, -0.1506f, -0.1367f, -0.1022f, -0.0446f, 0.0148f, 0.0402f, 0.0477f, 0.0477f };
	float t_H2C[14] = {0.0281f, 0.1325f, 0.2532f, 0.3431f, 0.4255f, 0.5298f, 0.6506f, 0.7405f, 0.8229f, 0.9272f, 1.0479f, 1.1379f, 1.2202f, 1.3246f};

	// If ATA
	if (m_type == 2){
	for (int i = 0; i < 14; i++){
	dx_man[i] = dx_ATA[i];
	dy_man[i] = dy_ATA[i];
	dz_man[i] = dz_ATA[i];
	t_man[i] = t_ATA[i];
	}
	}

	// If C2H
	else if (m_type == 3){
	for (int i = 0; i < 14; i++){
	dx_man[i] = dx_C2H[i];
	dy_man[i] = dy_C2H[i];
	dz_man[i] = dz_C2H[i];
	t_man[i] = t_C2H[i];
	}
	}

	// If H2C
	else if (m_type == 4){
	for (int i = 0; i < 14; i++){
	dx_man[i] = dx_H2C[i];
	dy_man[i] = dy_H2C[i];
	dz_man[i] = dz_H2C[i];
	t_man[i] = t_H2C[i];
	}
	}

	// Rotate to align with heading at from node
	ptr_to_DCM DCM = create_DCM(0.0f, 0.0f, -from->hdg);
	for (int i = 0; i < 14; i++){
	float temp[3] = { dx_man[i], dy_man[i], dz_man[i] };
	rotate_arrvec(DCM, temp);
	dx_man[i] = temp[0];
	dy_man[i] = temp[1];
	dz_man[i] = temp[2];
	}
	*/

	float dx_end, dy_end, dz_end, t_end;
	int m_type;

	switch (agile_man) {
	case ATA:
		m_type = 2;
		dx_end = 0.0f;
		dy_end = 0.0f;
		dz_end = 0.0f;
		t_end = 1.7734f;
		break;
	case C2H:
		m_type = 3;
		dx_end = 3.9098f;
		dy_end = 0.0f;
		dz_end = 0.4816f;
		t_end = 1.6667f;
		break;
	case H2C:
		m_type = 4;
		dx_end = 3.0763f;
		dy_end = 0.0f;
		dz_end = -0.0477f;
		t_end = 1.3246f;
		break;
	default:
		printf("Unrecognized agile maneuver type.\n");
	}

	// Rotate to align with heading at from node
	ptr_to_DCM DCM = create_DCM(0.0f, 0.0f, -from->hdg);
	float temp[3] = { dx_end, dy_end, dz_end };
	rotate_arrvec(DCM, temp);
	dx_end = temp[0];
	dy_end = temp[1];
	dz_end = temp[2];
	free(DCM);
	DCM = NULL;

	// Root of primtive is time-delayed node
	node* td = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 1, from->t + t_td, 0.0f, from);
	trim_end_states(td, from, from->tr_deg, from->zr, t_td);
	td->cost = from->cost + disp_info(from, td).cost;

	// End node
	node* end = new_node(td->coord[0] + dx_end, td->coord[1] + dy_end, td->coord[2] + dz_end, 0.0f, 0, 0, m_type, td->t + t_end, 0.0f, td);
	end->cost = td->cost + 50.0f;
	end->hdg = td->hdg + PI;

	limit_angle(end->hdg); // Make sure heading lies between -PI and PI

	add_child(td, end); // Append end node to time-delayed node

	return td; // Time-delayed node is root of primitive
}

// Update tree in accordance with aircraft's real-time motion
bool update_tree(node** root, node* goal)
{
	bool done = false; // True if aircraft reached the end of the trajectory

	// Get node nearest goal
	int n;
	if (!path_found) n = 0;
	else n = 1;
	std::vector<list> nearest_vec = near(*root, goal, n);
	node* end = nearest_vec[0].n;

	// Get stack of nodes after current root to that nearest node
	std::stack<node*> r2e_list = root_to_end(*root, end);

	// Get node at or just past time interval on way to nearest node
	node* comm_end = *root;
	while ((comm_end->t - (*root)->t) < t_int){

		// Return if goal node has been reached
		float D2G = sqrtf(powf(w.goals[w.n_goals - 1][0] - comm_end->coord[0], 2) + powf(w.goals[w.n_goals - 1][1] - comm_end->coord[1], 2) + powf(w.goals[w.n_goals - 1][2] - comm_end->coord[2], 2));
		if (D2G <= w.goals[w.n_goals - 1][4]){
			printf("Goal node reached :)\n");
			return done = true;
		}

		// If the current optimal node is reached, continue along children
		if (r2e_list.empty()){
			if (comm_end->child != NULL){
				comm_end = comm_end->child;
			}

			// If no children, algorithm has failed, go into hover
			else{
				printf("Caught up to trajectory end :(\n");

				// End with C2H
				node* C2H_root = steer_agile(comm_end, C2H);
				add_to_commit(C2H_root);
				add_to_commit(C2H_root->child);
				free_tree(&C2H_root);

				return done = true;
			}
		}
		else{
			comm_end = r2e_list.top();
			r2e_list.pop();
		}

		add_to_commit(comm_end);

		if (comm_end->type == 2) printf("ATA at [%.1f, %.1f, %.1f]\n", (double)comm_end->coord[0], (double)comm_end->coord[1], (double)comm_end->coord[2]);
		else if (comm_end->type == 3) printf("C2H at [%.1f, %.1f, %.1f]\n", (double)comm_end->coord[0], (double)comm_end->coord[1], (double)comm_end->coord[2]);

	}

	// Prune tree
	prune_tree(root, comm_end);
	*root = comm_end;
	(*root)->parent = NULL;

	return done;
}

// Displacement information for trajectory (length and cost)
disp disp_info(node* from, node* to)
{
	float L;

	float r = norm(from, to);
	float theta = atan2f(to->coord[1] - from->coord[1], to->coord[0] - from->coord[0]) - from->hdg;

	limit_angle(theta);

	if (fabsf(theta) <= 0.01f) L = r; // if theta == 0	
	else L = r*theta / sinf(theta); // if theta != 0

	//float xy_norm = sqrtf(powf(to->coord[0] - from->coord[0], 2) + powf(to->coord[1] - from->coord[1], 2));
	//float r_c = xy_norm / 2 * sinf(theta);
	//float fpa = atan2f(to->coord[2] - from->coord[2], xy_norm);

	disp d = { L, L }; // smoothness: 1/r_c + fabsf(fpa)

	return d;
}

// Check for collision
bool collision(node* root)
{
	node* from = root;
	node* to = NULL;
	node* temp = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0, 1.0f, 0.0f, NULL);
	node* last = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0, 1.0f, 0.0f, NULL);
	float dt, dt_end;

	while (from->child != NULL){

		to = from->child;

		// Check if node is in world
		if (out_of_world(to)) {
			free(temp);
			free(last);
			return true;
		}

		dt = coll_dist / V; // Check for collision every coll_dist
		if (to->type < 2) dt_end = (to->t - from->t); // Non-agile
		else if (to->type == 2) dt_end = ATA_dist / V; // ATA
		else dt_end = C2H_dist / V; // C2H

		memcpy(last, from, sizeof(node));
		while (dt < dt_end){

			// Check if node collides with known obstacles or path to node intersects new found obstacles
			trim_end_states(temp, from, to->tr_deg, to->zr, dt);
			if (inside_object(temp) || intersects_new_found_obs(last, temp)){
				free(temp);
				free(last);
				return true;
			}

			memcpy(last, temp, sizeof(node));
			dt += coll_dist / V;
		}

		// Check end of path for collisions with known or new found obstacles
		trim_end_states(temp, from, to->tr_deg, to->zr, dt_end);
		if (inside_object(temp) || intersects_new_found_obs(last, temp)){
			free(temp);
			free(last);
			return true;
		}

		from = to;
	}

	free(temp);
	free(last);
	return false;
}

bool out_of_world(node* n)
{
	if (n->coord[0] < bf || n->coord[0] > (w.lims[0] - bf) ||
		n->coord[1] < bf || n->coord[1] > (w.lims[1] - bf) ||
		n->coord[2] < bf || n->coord[2] > (w.lims[2] - bf)){
		return true;
	}

	return false;
}

bool inside_object(node* n)
{
	for (int i = 0; i < w.n_obs; i++){
		if (n->coord[0] >= (w.obs[i][0] - bf) &&
			n->coord[0] <= (w.obs[i][0] + w.obs[i][3] + bf) &&
			n->coord[1] >= (w.obs[i][1] - bf) &&
			n->coord[1] <= (w.obs[i][1] + w.obs[i][4] + bf) &&
			n->coord[2] >= (w.obs[i][2] - bf) &&
			n->coord[2] <= (w.obs[i][2] + w.obs[i][5] + bf)){
			return true;
		}
	}

	return false;
}

bool intersects_new_found_obs(node* from, node* to)
{
	if (found_obs.empty()){
		return false;
	}

	for (int i = 0; i < found_obs.size(); ++i){
		if (intersection(from->coord[0], from->coord[1], to->coord[0], to->coord[1],
			found_obs[i][0], found_obs[i][1], found_obs[i][2], found_obs[i][3])){
			return true;
		}
	}

	return false;
}

// Deallocate memory of tree
void free_tree(node** n)
{
	if (*n == NULL)
		return;

	free_tree(&((*n)->child));
	free_tree(&((*n)->next));

	free(*n);
	*n = NULL;
}

// Prune tree (free memory of nodes that aren't in new_root's tree)
void prune_tree(node** root, node* new_root)
{
	bool tbd = true;

	if (*root == NULL)
		return;

	if (*root == new_root) {
		tbd = false;
	}

	if (tbd){
		prune_tree(&(*root)->child, new_root);
		prune_tree(&(*root)->next, new_root);
		free(*root);
		*root = NULL;
	}
	else{
		prune_tree(&(*root)->next, new_root);
	}
}

// Create list of nodes in order of some nearness criteria
std::vector<list> near(node* root, node* to, int near_type)
{
	std::vector<list> near_vec; // Vector of nodes with nearness information
	add_to_near_vec(root, to, &near_vec); // Add all nodes in tree, with nearness information, to vector
	if (near_type == 0) std::sort(near_vec.begin(), near_vec.end(), comp_near); // Sort via nearness criteria
	else if (near_type == 1) std::sort(near_vec.begin(), near_vec.end(), comp_prox); // Sort via proximity
	else std::sort(near_vec.begin(), near_vec.end(), comp_snear); // Sort via smart nearness criteria

	return near_vec;
}

// Euclidean distance comparison
bool comp_prox(const list &a, const list &b)
{
	return a.proximity < b.proximity;
}

// Nearness comparison
bool comp_near(const list &a, const list &b)
{
	return a.nearness < b.nearness;
}

// Smart nearness comparison
bool comp_snear(const list &a, const list &b)
{
	return a.smart_nearness < b.smart_nearness;
}

// Get size of tree
int tree_size(node* root)
{
	if (root == NULL)
		return 0;
	else{
		return (tree_size(root->child) + 1 + tree_size(root->next));
	}
}

// Add nodes in tree, with nearness information, to vector
void add_to_near_vec(node* n, node* to, std::vector<list>* near_vec)
{
	if (n == NULL) return;

	// Nearness info is based on distance from time-delayed node to to node
	node* td = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 1, n->t + t_td, 0.0f, NULL);
	trim_end_states(td, n, n->tr_deg, n->zr, t_td);
	td->cost = n->cost + disp_info(n, td).cost;

	float D = sqrtf(powf(n->coord[0] - start_coord[0], 2) + powf(n->coord[1] - start_coord[1], 2) + powf(n->coord[2] - start_coord[2], 2));
	float L = n->cost;
	float D2G = norm(n, to);

	// Node address and nearness info
	list l;
	l.n = n;
	l.proximity = D2G;
	l.nearness = disp_info(td, to).length;
	l.smart_nearness = (L + D2G) / D;

	(*near_vec).push_back(l); // Add to vector

	free(td);
	td = NULL;

	// Recursively iterate through function for each node
	if (n->child != NULL) add_to_near_vec(n->child, to, near_vec);
	if (n->next != NULL) add_to_near_vec(n->next, to, near_vec);

	return;
}

// Create stack of nodes starting right after root and going to end
std::stack<node*> root_to_end(node* root, node* end)
{
	node* temp = end;
	std::stack<node*> r2e_dq;
	r2e_dq.push(temp); // Initialize deque with end node
	while (temp->parent != root && temp->parent != NULL){
		temp = temp->parent;
		r2e_dq.push(temp); // Add to front of deque
	}

	return r2e_dq;
}

// Add node to vector of committed nodes
void add_to_commit(node* n)
{
	static int num = 0;

	float rel_pos[3] = { n->coord[0] - start_coord[0],
		n->coord[1] - start_coord[1],
		n->coord[2] - start_coord[2] };
	rotate_arrvec(w.DCM, rel_pos);
	float pos[3] = { rel_pos[0] + w.offset[0] + start_coord[0],
		rel_pos[1] + w.offset[1] + start_coord[1],
		rel_pos[2] + w.offset[2] + start_coord[2] };
	float hdg = n->hdg + w.offset[3];
	limit_angle(hdg);

	++num;
}

// Create Direction Cosine Matrix
ptr_to_DCM create_DCM(float phi, float the, float psi)
{
	ptr_to_DCM DCM = (ptr_to_DCM)malloc(sizeof(*DCM));

	(*DCM)[0][0] = cosf(the)*cosf(psi);
	(*DCM)[0][1] = cosf(the)*sinf(psi);
	(*DCM)[0][2] = -sinf(the);

	(*DCM)[1][0] = -cosf(phi)*sinf(psi) + sinf(phi)*sinf(the)*cosf(psi);
	(*DCM)[1][1] = cosf(phi)*cosf(psi) + sinf(phi)*sinf(the)*sinf(psi);
	(*DCM)[1][2] = sinf(phi)*cosf(the);

	(*DCM)[2][0] = sinf(phi)*sinf(psi) + cosf(phi)*sinf(the)*cosf(psi);
	(*DCM)[2][1] = -sinf(phi)*cosf(psi) + cosf(phi)*sinf(the)*sinf(psi);
	(*DCM)[2][2] = cosf(phi)*cosf(the);

	return DCM;
}

// Calculate L2-norm between two nodes
float norm(node* n1, node* n2)
{
	return sqrtf(powf(n2->coord[0] - n1->coord[0], 2) + powf(n2->coord[1] - n1->coord[1], 2) + powf(n2->coord[2] - n1->coord[2], 2));
}

// Calculate L2-norm between two sets of 2D coordinates
float norm(const float p1_x, const float p1_y, const float p2_x, const float p2_y)
{
	return sqrtf(powf(p2_x - p1_x, 2) + powf(p2_y - p1_y, 2));
}

// Calculate L2-norm between two sets of positions
float norm(const float p1_x, const float p1_y, const float p1_z, const float p2_x, const float p2_y, const float p2_z)
{
	return sqrtf(powf(p2_x - p1_x, 2) + powf(p2_y - p1_y, 2) + powf(p2_z - p1_z, 2));
}

// Create world
void create_world(const int n)
{
	// Two columns
	if (n == 0){

		// Starting node
		w.start = new_node(15.0f, 4.0f, 5.0f, PI, 0, 0, 0, 0.0f, 0.0f, NULL);

		// Environment boundaries
		w.lims[0] = 50.0f;
		w.lims[1] = 25.0f;
		w.lims[2] = 10.0f;

		// Obstacles
		w.n_obs = 2;
		float obs_arr[2][6] = { { 0.0f, 8.0f, 0.0f, 40.0f, 2.0f, 10.0f },
		{ 10.0f, 16.0f, 0.0f, 40.0f, 2.0f, 10.0f } };

		w.obs = (float**)malloc(w.n_obs * sizeof(float**));
		for (int i = 0; i < w.n_obs; i++){
			w.obs[i] = (float*)malloc(6 * sizeof(float*));
		}

		for (int i = 0; i < w.n_obs; ++i){
			for (int j = 0; j < 6; ++j){
				w.obs[i][j] = obs_arr[i][j];
			}
		}

		// Goals (intermediate and final)
		w.n_goals = 9;
		float goal_arr[9][5] = { { 45.0f, 8.5f, 5.0f, PI / 2.0f, 1.5f },
		{ 30.0f, 13.0f, 5.0f, PI, 1.5f },
		{ 20.0f, 13.0f, 5.0f, PI, 1.5f },
		{ 10.0f, 13.0f, 5.0f, PI, 1.5f },
		{ 5.0f, 16.5f, 5.0f, PI / 2.0f, 1.5f },
		{ 10.0f, 21.0f, 5.0f, 0.0f, 1.5f },
		{ 20.0f, 21.0f, 5.0f, 0.0f, 1.5f },
		{ 30.0f, 21.0f, 5.0f, 0.0f, 1.5f },
		{ 48.0f, 21.0f, 5.0f, 0.0f, 2.0f } };

		w.goals = (float**)malloc(w.n_goals * sizeof(float**));
		for (int i = 0; i < w.n_goals; i++){
			w.goals[i] = (float*)malloc(5 * sizeof(float*));
		}

		for (int i = 0; i < w.n_goals; i++){
			for (int j = 0; j < 5; j++){
				w.goals[i][j] = goal_arr[i][j];
			}
		}
	}

	// Two corridors
	else if (n == 1){

		w.start = new_node(3.0f, 3.0f, 5.0f, 0.0f, 0, 0, 0, 0.0f, 0.0f, NULL);

		w.lims[0] = 50.0f;
		w.lims[1] = 25.0f;
		w.lims[2] = 10.0f;

		w.n_obs = 2;
		float obs_arr[2][6] = { { 10.0f, 10.0f, 0.0f, 10.0f, 10.0f, 10.0f },
		{ 30.0f, 13.0f, 0.0f, 10.0f, 10.0f, 10.0f } };

		w.obs = (float**)malloc(w.n_obs * sizeof(float**));
		for (int i = 0; i < w.n_obs; i++){
			w.obs[i] = (float*)malloc(6 * sizeof(float*));
		}

		for (int i = 0; i < w.n_obs; ++i){
			for (int j = 0; j < 6; ++j){
				w.obs[i][j] = obs_arr[i][j];
			}
		}

		w.n_goals = 1;
		float goal_arr[1][5] = { 48.0f, 23.0f, 5.0f, PI / 2.0f, 2.0f };

		w.goals = (float**)malloc(w.n_goals * sizeof(float**));
		for (int i = 0; i < w.n_goals; i++){
			w.goals[i] = (float*)malloc(5 * sizeof(float*));
		}

		for (int i = 0; i < w.n_goals; i++){
			for (int j = 0; j < 5; j++){
				w.goals[i][j] = goal_arr[i][j];
			}
		}
	}

	// Wall with passage
	else if (n == 2) {

		w.start = new_node(3.0f, 3.0f, 5.0f, 0.0f, 0, 0, 0, 0.0f, 0.0f, NULL);

		w.lims[0] = 50.0f;
		w.lims[1] = 25.0f;
		w.lims[2] = 10.0f;

		w.n_obs = 2;
		float obs_arr[2][6] = { { 24.0f, 0.0f, 0.0f, 2.0f, 10.0f, 10.0f },
		{ 24.0f, 15.0f, 0.0f, 2.0f, 10.0f, 10.0f } };

		w.obs = (float**)malloc(w.n_obs * sizeof(float**));
		for (int i = 0; i < w.n_obs; i++){
			w.obs[i] = (float*)malloc(6 * sizeof(float*));
		}

		for (int i = 0; i < w.n_obs; ++i){
			for (int j = 0; j < 6; ++j){
				w.obs[i][j] = obs_arr[i][j];
			}
		}

		w.n_goals = 2;
		float goal_arr[2][5] = { { 25.0f, 12.5f, 5.0f, 0.0f, 1.0f },
		{ 48.0f, 23.0f, 5.0f, 0.0f, 2.0f } };

		w.goals = (float**)malloc(w.n_goals * sizeof(float**));
		for (int i = 0; i < w.n_goals; i++){
			w.goals[i] = (float*)malloc(5 * sizeof(float*));
		}

		for (int i = 0; i < w.n_goals; i++){
			for (int j = 0; j < 5; j++){
				w.goals[i][j] = goal_arr[i][j];
			}
		}
	}

	// S walls
	else {

		w.start = new_node(3.0f, 3.0f, 5.0f, PI / 4, 0, 0, 0, 0.0f, 0.0f, NULL);

		w.lims[0] = 50.0f;
		w.lims[1] = 25.0f;
		w.lims[2] = 10.0f;

		w.n_obs = 2;
		float obs_arr[2][6] = { { 15.0f, 0.0f, 0.0f, 5.0f, 20.0f, 10.0f },
		{ 30.0f, 5.0f, 0.0f, 5.0f, 20.0f, 10.0f } };

		w.obs = (float**)malloc(w.n_obs * sizeof(float**));
		for (int i = 0; i < w.n_obs; i++){
			w.obs[i] = (float*)malloc(6 * sizeof(float*));
		}

		for (int i = 0; i < w.n_obs; ++i){
			for (int j = 0; j < 6; ++j){
				w.obs[i][j] = obs_arr[i][j];
			}
		}

		w.n_goals = 3;
		float goal_arr[3][5] = { { 17.5f, 22.5f, 5.0f, 0.0f, 2.5f },
		{ 32.5f, 2.5f, 5.0f, 0.0f, 2.5f },
		{ 48.0f, 23.0f, 5.0f, 0.0f, 2.0f } };

		w.goals = (float**)malloc(w.n_goals * sizeof(float**));
		for (int i = 0; i < w.n_goals; i++){
			w.goals[i] = (float*)malloc(5 * sizeof(float*));
		}

		for (int i = 0; i < w.n_goals; i++){
			for (int j = 0; j < 5; j++){
				w.goals[i][j] = goal_arr[i][j];
			}
		}
	}
}

// Check if goal region has been reached
bool goal_reached(node* n, node* goal, int gn)
{
	if (n == NULL) return false;
	if (norm(n, goal) <= w.goals[gn][4]) { // If in region
		//n->cost = (float)gn * (-1000.0f);
		return true;
	}
	if (goal_reached(n->next, goal, gn)) return true; // Recursively iterate
	return goal_reached(n->child, goal, gn);
}

void rotate_arrvec(ptr_to_DCM DCM, float arr_vec[3])
{
	float tmp[3] = { arr_vec[0], arr_vec[1], arr_vec[2] };

	arr_vec[0] = (*DCM)[0][0] * tmp[0] + (*DCM)[0][1] * tmp[1] + (*DCM)[0][2] * tmp[2];
	arr_vec[1] = (*DCM)[1][0] * tmp[0] + (*DCM)[1][1] * tmp[1] + (*DCM)[1][2] * tmp[2];
	arr_vec[2] = (*DCM)[2][0] * tmp[0] + (*DCM)[2][1] * tmp[1] + (*DCM)[2][2] * tmp[2];
}

void limit_angle(float &angle)
{
	angle = fmod(angle, 2 * PI);
	if (angle > PI) angle = -2 * PI + angle;
	else if (angle < -PI) angle = 2 * PI + angle;
}

// Get heading from euler array when aircraft is in hover
float hover_hdg(const float& phi, const float& the, const float& psi)
{
	// Create DCM prime
	ptr_to_DCM DCM_p = (ptr_to_DCM)malloc(sizeof(*DCM_p));

	(*DCM_p)[0][0] = cosf(the)*cosf(psi);
	(*DCM_p)[1][0] = cosf(the)*sinf(psi);
	(*DCM_p)[2][0] = -sinf(the);

	(*DCM_p)[0][1] = -cosf(phi)*sinf(psi) + sinf(phi)*sinf(the)*cosf(psi);
	(*DCM_p)[1][1] = cosf(phi)*cosf(psi) + sinf(phi)*sinf(the)*sinf(psi);
	(*DCM_p)[2][1] = sinf(phi)*cosf(the);

	(*DCM_p)[0][2] = sinf(phi)*sinf(psi) + cosf(phi)*sinf(the)*cosf(psi);
	(*DCM_p)[1][2] = -sinf(phi)*cosf(psi) + cosf(phi)*sinf(the)*sinf(psi);
	(*DCM_p)[2][2] = cosf(phi)*cosf(the);

	float temp[3] = { 0.0f, 0.0f, 1.0f };
	rotate_arrvec(DCM_p, temp);

	float hdg = atan2f(temp[1], temp[0]);

	free(DCM_p);
	DCM_p = NULL;

	return hdg;
}

bool intersection(const float p0_x, const float p0_y, const float p1_x, const float p1_y,
	const float p2_x, const float p2_y, const float p3_x, const float p3_y)
{
	float s1_x, s1_y, s2_x, s2_y;
	s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
	s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

	float den = (-s2_x * s1_y + s1_x * s2_y);

	if (fabsf(den) < 0.0001f) return false;

	float s, t;
	s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / den;
	t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / den;

	if (s >= 0 && s <= 1 && t >= 0 && t <= 1){
		return true; // Collision
	}

	return false; // No collision
}

void update_tree_for_new_obstacles(node** root)
{
	while (!new_obs.empty()){
		float d = norm((*root)->coord[0], (*root)->coord[1], new_obs.front()[0], new_obs.front()[1]);
		float d2 = norm((*root)->coord[0], (*root)->coord[1], new_obs.front()[2], new_obs.front()[3]);
		if (d2 > d) {
			d = d2;
		}

		prune_new_obs_collisions(root, root, d);
		found_obs.push_back(new_obs.front());
		new_obs.pop();
	}
}

void prune_new_obs_collisions(node** n, node** root, float d)
{
	if (*n == NULL){
		return;
	}

	float dn = norm((*root)->coord[0], (*root)->coord[1], (*n)->coord[0], (*n)->coord[1]);
	bool past_obs = (dn > d) ? true : false;

	if (!past_obs && (*n)->child != NULL){
		if (intersection((*n)->coord[0], (*n)->coord[1], (*n)->child->coord[0], (*n)->child->coord[1],
			new_obs.front()[0], new_obs.front()[1], new_obs.front()[2], new_obs.front()[3])) {
			free_tree(n);
		}
	}

	if (!past_obs){
		prune_new_obs_collisions(&((*n)->child), root, d);
	}
	prune_new_obs_collisions(&((*n)->next), root, d);
}