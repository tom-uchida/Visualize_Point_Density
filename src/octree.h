#pragma once

#include <vector>
#include "create_octree.h"

const int MIN_NODE = 15;

class octree {

private:
    std::vector<int> pIdx;

public:
    octreeNode *octreeRoot;
    octree( float _points[], const size_t _nPoints, double _range[], const int _nMinNode );
};