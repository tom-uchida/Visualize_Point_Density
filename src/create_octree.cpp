#include <iostream>
#include <vector>
#include <cmath>
#include "create_octree.h"
#include "vec_ops.h"

void create_octree( octreeNode *_node,
                    float _points[],    int _nMinNode,
                    double _xMin,       double _xMax, 
                    double _yMin,       double _yMax,
                    double _zMin,       double _zMax  ) {
    // Assign _points in the _node to its children
    size_t i, j, k, i_point, nPoints, idx;

    nPoints = _node->pointIdx.size();

    if ( nPoints > _nMinNode ) {

        // Create 8 cells
        for ( i = 0; i < 2; i++ ) {
            for ( j = 0; j < 2; j++ ) {
                for ( k = 0; k < 2; k++ ) {
                    _node->childOctreeNode[i][j][k] = new octreeNode;
                    _node->childOctreeNode[i][j][k]->pointIdx.clear();
                }
            }
        } // end for

        // Calculate center coordinates of the 3D space range
        _node->centerCoords[0] = ( _xMin + _xMax ) * 0.5;
        _node->centerCoords[1] = ( _yMin + _yMax ) * 0.5;
        _node->centerCoords[2] = ( _zMin + _zMax ) * 0.5;

        // Assign all _points into children
        for ( i_point = 0; i_point < nPoints; i_point++ ) {
            // Get index of the point
            idx = _node->pointIdx[i_point] * 3;

            // Assign an appropriate cell, considering each axis
            i = ( _points[idx]     < _node->centerCoords[0] ) ? 0 : 1; // x
            j = ( _points[idx + 1] < _node->centerCoords[1] ) ? 0 : 1; // y
            k = ( _points[idx + 2] < _node->centerCoords[2] ) ? 0 : 1; // z

            // Add into child octree _node
            _node->childOctreeNode[i][j][k]->pointIdx.push_back( _node->pointIdx[i_point] );
        } // end for

        // Clear index-vector because it has been assigned
        _node->pointIdx.clear();


        // Assign child indices respectively
        // 1: [0][0][0]
        if ( _node->childOctreeNode[0][0][0]->pointIdx.size() == 0 ) {
            delete _node->childOctreeNode[0][0][0];
            _node->childOctreeNode[0][0][0] = NULL;

        } else {
            // Call recursively
            // Note that the target 3D space range has been updated
            create_octree(  _node->childOctreeNode[0][0][0], _points, _nMinNode,
                            _xMin, _node->centerCoords[0],
                            _yMin, _node->centerCoords[1],
                            _zMin, _node->centerCoords[2]  );
        } // end if

        // 2: [1][0][0]
        if ( _node->childOctreeNode[1][0][0]->pointIdx.size() == 0 ) {
            delete _node->childOctreeNode[1][0][0];
            _node->childOctreeNode[1][0][0] = NULL;

        } else {
            create_octree(  _node->childOctreeNode[1][0][0], _points, _nMinNode,
                            _node->centerCoords[0], _xMax,
                            _yMin, _node->centerCoords[1],
                            _zMin, _node->centerCoords[2]  );
        } // end if 

        // 3: [0][1][0]
        if ( _node->childOctreeNode[0][1][0]->pointIdx.size() == 0 ) {
            delete _node->childOctreeNode[0][1][0];
            _node->childOctreeNode[0][1][0] = NULL;

        } else {
            create_octree(  _node->childOctreeNode[0][1][0], _points, _nMinNode,
                            _xMin, _node->centerCoords[0],
                            _node->centerCoords[1], _yMax,
                            _zMin, _node->centerCoords[2]  );
        } // end if

        // 4: [1][1][0]
        if ( _node->childOctreeNode[1][1][0]->pointIdx.size() == 0 ) {
            delete _node->childOctreeNode[1][1][0];
            _node->childOctreeNode[1][1][0] = NULL;
        
        } else {
            create_octree(  _node->childOctreeNode[1][1][0], _points, _nMinNode,
                            _node->centerCoords[0], _xMax,
                            _node->centerCoords[1], _yMax,
                            _zMin, _node->centerCoords[2]  );
        } // end if

        // 5: [0][0][1]
        if ( _node->childOctreeNode[0][0][1]->pointIdx.size() == 0 ) {
            delete _node->childOctreeNode[0][0][1];
            _node->childOctreeNode[0][0][1] = NULL;
        
        } else {
            create_octree(  _node->childOctreeNode[0][0][1], _points, _nMinNode,
                            _xMin, _node->centerCoords[0],
                            _yMin, _node->centerCoords[1],
                            _node->centerCoords[2], _zMax  );
        } // end if

        // 6: [1][0][1]
        if ( _node->childOctreeNode[1][0][1]->pointIdx.size() == 0 ) {
            delete _node->childOctreeNode[1][0][1];
            _node->childOctreeNode[1][0][1] = NULL;

        } else {
            create_octree(  _node->childOctreeNode[1][0][1], _points, _nMinNode,
                            _node->centerCoords[0], _xMax,
                            _yMin, _node->centerCoords[1],
                            _node->centerCoords[2], _zMax  );
        } // end if

        // 7: [0][1][1]
        if ( _node->childOctreeNode[0][1][1]->pointIdx.size() == 0 ) {
            delete _node->childOctreeNode[0][1][1];
            _node->childOctreeNode[0][1][1] = NULL;
        
        } else {
            create_octree(  _node->childOctreeNode[0][1][1], _points, _nMinNode,
                            _xMin, _node->centerCoords[0],
                            _node->centerCoords[1], _yMax,
                            _node->centerCoords[2], _zMax  );
        } // end if

        // 8: [1][1][1]
        if ( _node->childOctreeNode[1][1][1]->pointIdx.size() == 0 ) {
            delete _node->childOctreeNode[1][1][1];
            _node->childOctreeNode[1][1][1] = NULL;
        
        } else {
            create_octree(  _node->childOctreeNode[1][1][1], _points, _nMinNode,
                            _node->centerCoords[0], _xMax,
                            _node->centerCoords[1], _yMax,
                            _node->centerCoords[2], _zMax  );
        } // end if

    } else { // if ( nPoints > _nMinNode ) {

        // Don't create childe further
        for ( i = 0; i < 2; i++ )
            for ( j = 0; j < 2; j++ )
                for ( k = 0; k < 2; k++ )
                    _node->childOctreeNode[i][j][k] = NULL;
        // end for

    } // end if

} // End create_octree()


void search_node(   octreeNode *_node, float _points[], double _point[],
                    std::vector<size_t> *_nearIdxPtr, std::vector<double> *_dist,
                    double _xLeft, double _xRight,
                    double _yLeft, double _yRight,
                    double _zLeft, double _zRight,
                    double _R2  ) {

    if ( _node->pointIdx.size() == 0 ) {

        // If node has children
        if ( _xLeft <= _node->centerCoords[0] ) {
            
            // Search [0][?][?]
            if ( _yLeft <= _node->centerCoords[1] ) {

                // Search [0][0][?]
                if ( _zLeft <= _node->centerCoords[2] ) {

                    // Search [0][0][0]
                    if ( _node->childOctreeNode[0][0][0] != NULL ) {
                        search_node(    _node->childOctreeNode[0][0][0], _points, _point, 
                                        _nearIdxPtr, _dist,
                                        _xLeft, _xRight,
                                        _yLeft, _yRight,
                                        _zLeft, _zRight,
                                        _R2  );
                    }

                } // end if

                if ( _zRight >= _node->centerCoords[2] ) {

	                // Search [0][0][1]
                    if ( _node->childOctreeNode[0][0][1] != NULL ) {
                        search_node(    _node->childOctreeNode[0][0][1], _points, _point, 
                                        _nearIdxPtr, _dist,
                                        _xLeft, _xRight,
                                        _yLeft, _yRight,
                                        _zLeft, _zRight,
                                        _R2  );
                    }
                
                } // end if

            } // end if

            if ( _yRight >= _node->centerCoords[1] ) {
                
                // Search [0][1][?]
                if ( _zLeft <= _node->centerCoords[2] ) {
                    
                    // Search [0][1][0]
                    if ( _node->childOctreeNode[0][1][0] != NULL ) {
                        search_node(    _node->childOctreeNode[0][1][0], _points, _point, 
                                        _nearIdxPtr, _dist,
                                        _xLeft, _xRight,
                                        _yLeft, _yRight,
                                        _zLeft, _zRight,
                                        _R2  );
                    }

                } // end if

                if ( _zRight >= _node->centerCoords[2] ) {
                    
                    // Search [0][1][1]
                    if ( _node->childOctreeNode[0][1][1] != NULL ) {
                        search_node(    _node->childOctreeNode[0][1][1], _points, _point, 
                                        _nearIdxPtr, _dist,
                                        _xLeft, _xRight,
                                        _yLeft, _yRight,
                                        _zLeft, _zRight,
                                        _R2  );
                    }

                } // end if

            } // end if

        } // end if

        if ( _xRight >= _node->centerCoords[0] ) {
            
            // Search [1][?][?]
            if ( _yLeft <= _node->centerCoords[1] ) {
                
                // Search [1][0][?]
                if ( _zLeft <= _node->centerCoords[2] ) {
                    
                    // Search [1][0][0]
                    if ( _node->childOctreeNode[1][0][0] != NULL ) {
                        search_node(    _node->childOctreeNode[1][0][0], _points, _point, 
                                        _nearIdxPtr, _dist,
                                        _xLeft, _xRight,
                                        _yLeft, _yRight,
                                        _zLeft, _zRight,
                                        _R2  );
                    }
                
                } // end if

                if ( _zRight >= _node->centerCoords[2] ) {
                    
                    // Search [1][0][1]
                    if ( _node->childOctreeNode[1][0][1] != NULL ) {
                        search_node(    _node->childOctreeNode[1][0][1], _points, _point, 
                                        _nearIdxPtr, _dist,
                                        _xLeft, _xRight,
                                        _yLeft, _yRight,
                                        _zLeft, _zRight,
                                        _R2  );
                    }

                } // end if

            } // end if

            if ( _yRight >= _node->centerCoords[1] ) {
                
                // Search [1][1][?]
                if ( _zLeft <= _node->centerCoords[2] ) {
                    
                    // Search [1][1][0]
                    if ( _node->childOctreeNode[1][1][0] != NULL ) {
                        search_node(    _node->childOctreeNode[1][1][0], _points, _point, 
                                        _nearIdxPtr, _dist,
                                        _xLeft, _xRight,
                                        _yLeft, _yRight,
                                        _zLeft, _zRight,
                                        _R2  );
                    }

                } // end if

                if ( _zRight >= _node->centerCoords[2] ) {
                    
                    // Search [1][1][1]
                    if ( _node->childOctreeNode[1][1][1] != NULL ) {
                        search_node(    _node->childOctreeNode[1][1][1], _points, _point, 
                                        _nearIdxPtr, _dist,
                                        _xLeft, _xRight,
                                        _yLeft, _yRight,
                                        _zLeft, _zRight,
                                        _R2  );
                    }

                } // end if

            } // end if

        } // end if
    
    } else {

        // If _node is a leaf
        size_t nPoints = _node->pointIdx.size();

        for ( size_t i = 0; i < nPoints; i++ ) {
            double point[3] = { (double)_points[ _node->pointIdx[i] * 3 ],      
                                (double)_points[ _node->pointIdx[i] * 3 + 1 ],  
                                (double)_points[ _node->pointIdx[i] * 3 + 2 ]   };
            double dist = dist2( _point, point );

            if ( dist < _R2 ) {
                //if ( dist2( _point, &( _points[ _node->pointIdx[i] * 3 ]) ) < _R2 ) {
                _nearIdxPtr->push_back( _node->pointIdx[i] );
                _dist->push_back( sqrt( dist ) );
            } 
        } // end for

    } // end if

} // End search_node()


void search_points( octreeNode *_node, float _points[],
                    double _searchRadius, double _point[],
                    std::vector<size_t> *_nearIdxPtr,
                    std::vector<double> *_dist   ) {

    double xLeft   = _point[0] - _searchRadius;
    double xRight  = _point[0] + _searchRadius;
    double yLeft   = _point[1] - _searchRadius;
    double yRight  = _point[1] + _searchRadius;
    double zLeft   = _point[2] - _searchRadius;
    double zRight  = _point[2] + _searchRadius;
    double R2      = _searchRadius * _searchRadius;

    search_node(    _node,  _points, _point, 
                    _nearIdxPtr, _dist,
                    xLeft,  xRight,
                    yLeft,  yRight,
                    zLeft,  zRight,
                    R2  );

} // End search_points()