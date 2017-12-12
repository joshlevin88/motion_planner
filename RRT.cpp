#include "RRT_Prim.h"
#include "common.h"

const int max_tree_size = 10000; // Max tree size (to keep memory usage low)
const float lr_dist = 10.0f; // Laser range finder range

bool path_found = false; // Whether a feasible path has been found yet
world w; // Environment information
float start_coord[3]; // Starting position

node* RRT_loop(node* root); 

int main()
{
	// Initial pose (faux sensor measurements)
	float hdg_init = 45.0f*PI/180.0f;
	float p_init[3];
	p_init[0] = 1.0f;
	p_init[1] = 1.0f;
	p_init[2] = 5.0f;

	// User selected
	int nw = 1;

	node* root = initialize_world(nw, p_init, hdg_init);

	root = RRT_loop(root);

	cleanup_tree_and_world(&root);

	// TEMP: testing initialize_world_2 and add_commit_2 coordinate transformations
	/*
	node* root = initialize_world_2(nw, p_init, hdg_init);
	printf("Root pos: [%.1f, %.1f, %.1f], Root hdg: %.1f\n", 
		root->coord[0], root->coord[1], root->coord[2], root->hdg*180.0f/PI);
	add_to_commit_2(root);
	*/
}

node* RRT_loop(node* root)
{
	clock_t t1, t2;
	bool done = false;
	float elapsed;
	int iter = 0;
	int up_iter = 0;
	int gn = 0;
	int fo_count = 0;

	node* prim_root;
	node* goal = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0, 0.0f, 0.0f, NULL);
	goal->coord[0] = w.goals[gn][0]; goal->coord[1] = w.goals[gn][1]; goal->coord[2] = w.goals[gn][2];
	node* randn = new_node(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0, 0.0f, 0.0f, NULL);
	srand((int)time(NULL));

	while (!done){

		// Start timer
		t1 = clock();
		bool done_iter = false;

		while (!done_iter){

			// Keep track of real-time computation interval
			t2 = clock();
			elapsed = (float)(t2 - t1) / static_cast<float>(CLOCKS_PER_SEC);
			if (elapsed >= t_int){
				done_iter = true;
				break;
			}

			if (tree_size(root) < max_tree_size){

				// Generate a random node (change coordinates of random node)
				rand_node_coord(randn, goal, iter);

				// Extend tree
				prim_root = extend_tree(root, randn, goal, gn);

				// Check if current goal has been reached
				if (goal_reached(prim_root, goal, gn)){
					// If intermediate goal, update goal node
					if (gn < (w.n_goals - 1)){
						gn++;
						goal->coord[0] = w.goals[gn][0];
						goal->coord[1] = w.goals[gn][1];
						goal->coord[2] = w.goals[gn][2];
						printf("Intermediate goal %d of %d reached!\n", gn, w.n_goals - 1);
					}
					// If final goal, feasible path has been found
					else if (!path_found){
						path_found = true;
						printf("Feasible path found!\n");
					}
				}

			}

			iter++;
		}

		// Print number of iterations and size of tree
		printf("Number of iterations: %d,  Size of tree: %d\n", iter, tree_size(root));
		
		/*
		if (fo_count % 3 == 0 && fo_count <= 9) {
			float d_rad = 0.1f;
			float x1 = root->coord[0] + lr_dist*cosf(root->hdg + d_rad);
			float y1 = root->coord[1] + lr_dist*sinf(root->hdg + d_rad);
			float x2 = root->coord[0] + lr_dist*cosf(root->hdg - d_rad);
			float y2 = root->coord[1] + lr_dist*sinf(root->hdg - d_rad);
			std::vector<float> temp_vec{ x1, y1, x2, y2 };
			new_obs.push(temp_vec);
		}
		++fo_count;
		*/
		
		// Update tree in accordance with aircraft's real-time motion
		done = update_tree(&root, goal);
		up_iter++;
		if (up_iter % 5 == 0){
			printf("Freeing tree\n");
			free_tree(&(root->child));
		}
	}

	// Garbage collection
	free(randn);
	free(goal);

	return root;
}
