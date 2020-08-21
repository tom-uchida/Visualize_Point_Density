#pragma once

#include <vector>
#include "create_octree.h"

class octree {

private:
    std::vector<int> pIdx;

public:
    octreeNode *octreeRoot;
    octree( float _points[], size_t _nPoints, double _range[], int _nMinNode );
};