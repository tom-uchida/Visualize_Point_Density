#ifndef __octree
#define __octree

#include <vector>
#include "create_octree.h"

// 8分木のノードを表す構造体
// struct octreeNode {
//   vector <int> pInd;
//   double c[3];
//   octreeNode *cOctreeNode[2][2][2];
// };

class octree {

private:
    std::vector<int> pInd;

public:
    octreeNode *octreeRoot;
    octree(float _points[], size_t _num_of_points, double _range[], int _num_of_min_node);
};

#endif