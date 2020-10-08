#pragma once

#include <vector>

const int INTERVAL = 1e6;

struct octreeNode {
    std::vector<size_t> pointIdx;
    double centerCoords[3];
    octreeNode *childOctreeNode[2][2][2];
};

void create_octree( octreeNode *_node, float _points[], const int _nMinNode,
                    const double _xMin, const double _xMax,
                    const double _yMin, const double _yMax,
                    const double _zMin, const double _zMax  );

void search_points( octreeNode *_node, float _points[],
                    const double _searchRadius, double _point[],
                    std::vector<size_t> *_nearIdxPtr,
                    std::vector<double> *_dist  );