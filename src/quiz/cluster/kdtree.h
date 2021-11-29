/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert_At(Node** current_node,std::vector<float> point,int level, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
			unsigned int compare_index = level % 2;
			if (*current_node == NULL){
				*current_node = new Node(point,id);
			}
			else {
				if (point[compare_index] < ((*current_node)->point[compare_index])){
				
				insert_At( &((*current_node)->left), point, compare_index+1, id);
				}
				else {
				
				insert_At( &((*current_node)->right), point, compare_index+1, id);
				}
			}

	}

	void insert(std::vector<float> point, int id)
	{
		// insert a new point into the tree
		// create a new node and place correctly with in the root 
		insert_At(&root,point,0,id);

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_in(root,0,target,distanceTol,ids);


		return ids;
	}
	
	void search_in(Node* current_node,int level,std::vector<float> target, float distanceTol,std::vector<int>& ids)
	{
		
      
		if (current_node!=NULL){

					
			float max_x = target[0] + distanceTol;
			float min_x = target[0] - distanceTol;

			float max_y = target[1] + distanceTol;
			float min_y = target[1] - distanceTol;

			
			unsigned int s_level = level% 2;

			if ( ((current_node->point[0]>=min_x) && (current_node->point[0]<=max_x)) && ((current_node->point[1]>=min_y) && (current_node->point[1]<=max_y)) ){
				
				float dist = sqrt((target[0] - current_node->point[0]) * (target[0] - current_node->point[0]) + (target[1] - current_node->point[1]) * (target[1] - current_node->point[1]));
				
				if (dist <= distanceTol)
						
					ids.push_back(current_node->id);

				
					
			}
			

			if ( (target[s_level] - distanceTol) < current_node->point[s_level] )
				search_in( current_node->left,level+1,target,distanceTol,ids );
			if ( (target[s_level] + distanceTol) > current_node->point[s_level] )
				search_in( current_node->right,level+1,target,distanceTol,ids );
		}
		

	}

};




