#pragma once

#include <vector>

struct octreeNode {
    std::vector<size_t> pIdx;
    double c[3];
    octreeNode *cOctreeNode[2][2][2];
};

void create_octree( octreeNode *_node, float _points[], int _nMin,
                    double _xMin, double _xMax, double _yMin, double _yMax,
                    double _zMin, double _zMax);

void search_points( double _p[], double _R, float _points[],
                    octreeNode *_Node, std::vector<size_t> *_nearIdxPtr,
                    std::vector<double> *_dist );