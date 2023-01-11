#include <algorithm>

constexpr size_t MAX_NODE=100000;

enum var{
    X, Y, Z
};

template<typename PointT>
class KdTree
{

struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;
};

public:
    KdTree() = default;

    void init(){
        node_cnt=0;
        root = nullptr;
        return;
    }
	void insert(PointT point, int id)
	{
        if(root == nullptr) root = new_node(point, id);
        else{
            root = insert_rec(root, point, id, 0);
        }
        return;
	}

    void insert_cloud(typename pcl::PointCloud<PointT>::Ptr cloud)
    {
        int id = 0;

        for(PointT point: cloud->points)
        {
            insert(point, id++);

        }
        return;
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(typename pcl::PointCloud<PointT>::Ptr cloud, int idx, float distanceTol)
	{
		std::vector<int> ids;
        search_rec(cloud, idx, root, 0, distanceTol, ids);
		return ids;
	}

private:
    Node* root;
    int node_cnt;
    Node node_pool[MAX_NODE];

    Node* new_node(PointT point, int id){
        node_pool[node_cnt].id = id;
        node_pool[node_cnt].point = point;
        node_pool[node_cnt].left = nullptr;
        node_pool[node_cnt].right = nullptr;
        return &node_pool[node_cnt++];
    }

    Node* insert_rec(Node* node, PointT point, int id, int depth){
        if(node == nullptr)
            return new_node(point, id);

        int type = depth % 3;
        float _point, _node_point;

        for(int i=0; i<3; ++i){
            switch(type+i){
                case X:
                    _point = point.x, _node_point = node->point.x;
                    break;
                case Y:
                    _point = point.y, _node_point = node->point.y;
                    break;
                case Z:
                    _point = point.z, _node_point = node->point.z;
                    break;
                default:
                    break;
            }


            if(_point > _node_point){
                node->right = insert_rec(node->right, point, id, depth+1);
                break;
            }
            else if(_point < _node_point){
                node->left = insert_rec(node->left, point, id, depth+1);
                break;
            }
            else continue;
        }
        return node;
    }

    struct pointX{
        bool operator()(const PointT& a, const PointT&b){
            return (a.x < b.x);
        }
    };

    struct pointY{
        bool operator()(const PointT& a, const PointT&b){
            return (a.y < b.y);
        }
    };
    struct pointZ{
        bool operator()(const PointT& a, const PointT&b){
            return (a.z < b.z);
        }
    };

    void search_rec(typename pcl::PointCloud<PointT>::Ptr cloud,int idx, Node* node, int depth, float distanceTol, std::vector<int>& ids)
    {
        if (node==nullptr) return;
        bool flag=false;

        if(node->point.x - cloud->points[idx].x >= -distanceTol && node->point.x - cloud->points[idx].x <= distanceTol && 
            node->point.y - cloud->points[idx].y >= -distanceTol && node->point.y - cloud->points[idx].y <= distanceTol &&
            node->point.z - cloud->points[idx].z >= -distanceTol && node->point.z - cloud->points[idx].z <= distanceTol)
            flag=true;

        if(flag){
            float distance=0;
            distance=0;
            distance += (node->point.x - cloud->points[idx].x) * (node->point.x - cloud->points[idx].x);
            distance += (node->point.y - cloud->points[idx].y) * (node->point.y - cloud->points[idx].y);
            distance += (node->point.z - cloud->points[idx].z) * (node->point.z - cloud->points[idx].z);
            distance = sqrt(distance);

            if(distance <= distanceTol)
                ids.push_back(node->id);
        }

        int type = depth % 3;
        float _point, _node_point;

        switch(type){
            case X:
                _point = cloud->points[idx].x, _node_point = node->point.x;
                break;
            case Y:
                _point = cloud->points[idx].y, _node_point = node->point.y;
                break;
            case Z:
                _point = cloud->points[idx].z, _node_point = node->point.z;
                break;
            default:
                break;
        }

        if(_node_point - _point >= -distanceTol)
            search_rec(cloud, idx, node->left, depth+1, distanceTol, ids);
        if(_node_point - _point <= distanceTol)
            search_rec(cloud, idx, node->right, depth+1, distanceTol, ids);
    }
};
