
// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}
};

struct KdTree
{
	Node *root;

	KdTree()
		: root(NULL)
	{
	}

	void search(Node **node, std::vector<float> target, std::vector<int> &ids,
				float distanceTol, int depth, int component1, int component2, int component3)
	{
		if (*node != NULL)
		{
			if ((*node)->point[component1] > target[component1] + distanceTol)
			{
				searchNearbyId3d(&(*node)->left, target, ids, distanceTol, depth + 1);
			}
			else if ((*node)->point[component1] < target[component1] - distanceTol)
			{
				searchNearbyId3d(&(*node)->right, target, ids, distanceTol, depth + 1);
			}
			else
			{
				if ((*node)->point[component2] < target[component2] + distanceTol && (*node)->point[component2] > target[component2] - distanceTol &&
					(*node)->point[component3] < target[component3] + distanceTol && (*node)->point[component3] > target[component3] - distanceTol)
				{
					ids.push_back((*node)->id);
				}
				searchNearbyId3d(&(*node)->left, target, ids, distanceTol, depth + 1);
				searchNearbyId3d(&(*node)->right, target, ids, distanceTol, depth + 1);
			}
		}
	}

	void searchNearbyId3d(Node **node, std::vector<float> target, std::vector<int> &ids,
						  float distanceTol, int depth)
	{
		if (depth % 3 == 0)
		{
			// compare x component
			if (*node != NULL)
			{
				search(node, target, ids, distanceTol, depth, 0, 1, 2);
			}
		}
		else if (depth % 3 == 1)
		{
			// compare y component
			if (*node != NULL)
			{
				search(node, target, ids, distanceTol, depth, 1, 0, 2);
			}
		}
		else
		{
			// compare z component
			if (*node != NULL)
			{
				search(node, target, ids, distanceTol, depth, 2, 0, 1);
			}
		}
	}

	void compare(Node **node, std::vector<float> point, int id, int depth, int component)
	{
		if (*node == NULL)
			*node = new Node(point, id);
		else if (point[component] < (*node)->point[component])
			insertNode3D(&(*node)->left, point, id, depth + 1);
		else
			insertNode3D(&(*node)->right, point, id, depth + 1);
	}

	void insertNode3D(Node **node, std::vector<float> point, int id, int depth)
	{
		if (depth % 3 == 0)
		{
			// compare x component
			compare(node, point, id, depth, 0);
		}
		else if (depth % 3 == 1)
		{
			// compare y component
			compare(node, point, id, depth, 1);
		}
		else
		{
			// compare z component
			compare(node, point, id, depth, 2);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// insert a new point into the tree
		// the function creates a new node and place correctly with in the root
		int depth = 0;
		insertNode3D(&root, point, id, depth);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		int depth = 0;
		searchNearbyId3d(&root, target, ids, distanceTol, depth);

		return ids;
	}
};
