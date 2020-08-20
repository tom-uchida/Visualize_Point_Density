#include "octree.h"

octree::octree(float _points[], size_t _num_of_points, double _range[], int _num_of_min_node)
{

    octreeRoot = new octreeNode;

    for ( size_t i = 0; i < _num_of_points; i++ ) {
        octreeRoot->pInd.push_back(i);
    }

    create_octree(  octreeRoot, _points, _num_of_min_node,
                    _range[0], _range[1], _range[2], _range[3], _range[4], _range[5]);

}
