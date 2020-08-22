#include "octree.h"

octree::octree( float _points[], size_t _nPoints, double _range[], int _nMinNode )
{
    octreeRoot = new octreeNode;

    for ( size_t i = 0; i < _nPoints; i++ ) {
        octreeRoot->pointIdx.push_back( i );
    }

    create_octree(  octreeRoot,
                    _points,    _nMinNode,
                    _range[0],  _range[1],
                    _range[2],  _range[3],
                    _range[4],  _range[5] );
}