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
};


constexpr size_t MAX_NODE=100000;
Node node_pool[MAX_NODE];

int node_cnt;

Node* new_node(std::vector<float> point, int id){
    node_pool[node_cnt].id = id;
    node_pool[node_cnt].point = point;
    node_pool[node_cnt].left = nullptr;
    node_pool[node_cnt].right = nullptr;
    return &node_pool[node_cnt++];
}

class KdTree
{

public:
    KdTree() = default;
	Node* root;

    void init(){
        node_cnt=0;
        root = nullptr;
    }
	void insert(std::vector<float> point, int id)
	{
        if(root == nullptr) root = new_node(point, id);
        else{
            root = insert_rec(root, point, id, 0);
        }
	}

    void insert_cloud(

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        search_rec(target, root, 0, distanceTol, ids);
		return ids;
	}

private:
    Node* insert_rec(Node* node, std::vector<float> point, int id, int depth) const{
        if(node == nullptr){
            return new_node(point, id);
        }

        if(point[depth%node->point.size()] > node->point[depth%node->point.size()]){
            node->right = insert_rec(node->right, point, id, depth+1);
        }
        else if(point[depth%node->point.size()] < node->point[depth%node->point.size()]){
            node->left = insert_rec(node->left, point, id, depth+1);
        }
        else{
            if(point[(depth+1)%node->point.size()] > node->point[(depth+1)%node->point.size()]){
                node->right = insert_rec(node->right, point, id, depth+1);
            }
            else if(point[(depth+1)%node->point.size()] < node->point[(depth+1)%node->point.size()]){
                node->left = insert_rec(node->left, point, id, depth+1);
            }
        }
        return node;
    }

    void search_rec(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
    {
        if (node==nullptr) return;
        bool flag=true;
        for(int i=0; i<node->point.size(); ++i){
            if(node->point[i] - target[i] >= -distanceTol && node->point[i] - target[i] <= distanceTol)
                continue;
            flag=false;
        }

        if(flag){
            float distance=0;
            for(int i=0; i<node->point.size(); ++i){
                distance += (node->point[i] - target[i]) * (node->point[i] - target[i]);
            }
            distance = sqrt(distance);
            if(distance <= distanceTol)
                ids.push_back(node->id);
        }

        if(node->point[depth%node->point.size()] - target[depth%node->point.size()] >= -distanceTol)
            search_rec(target, node->left, depth+1, distanceTol, ids);
        if(node->point[depth%node->point.size()] - target[depth%node->point.size()] <= distanceTol)
            search_rec(target, node->right, depth+1, distanceTol, ids);
    }
};

