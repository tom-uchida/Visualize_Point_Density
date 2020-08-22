#include <iostream>
#include <vector>
#include <cmath>
#include "create_octree.h"
#include "vec_ops.h"

void create_octree( octreeNode *_node, float _points[], int _nMinNode,
                    double _xMin, double _xMax, 
                    double _yMin, double _yMax,
                    double _zMin, double _zMax  ) {
    // Assign points in the node to its children
    size_t i, j, k, i_point, nPoints, idx;

    nPoints = _node->pIdx.size();

    if ( nPoints > _nMinNode ) {
        // Create 8 cells
        for ( i = 0; i < 2; i++ ) {
            for ( j = 0; j < 2; j++ ) {
                for ( k = 0; k < 2; k++ ) {
                    _node->childOctreeNode[i][j][k] = new octreeNode;
                    _node->childOctreeNode[i][j][k]->pIdx.clear();
                }
            }
        } // end for

        // Calculate center coordinates of the 3D space range
        _node->centerCoords[0] = ( _xMin + _xMax ) * 0.5;
        _node->centerCoords[1] = ( _yMin + _yMax ) * 0.5;
        _node->centerCoords[2] = ( _zMin + _zMax ) * 0.5;

        // Assign all points into children
        for ( i_point = 0; i_point < nPoints; i_point++ ) {
            idx = _node->pIdx[i_point] * 3;
            i = ( ( _points[idx] < _node->centerCoords[0] ) ? 0: 1 );
            j = ( ( _points[idx + 1] < _node->centerCoords[1] ) ? 0: 1 );
            k = ( ( _points[idx + 2] < _node->centerCoords[2] ) ? 0: 1 );
            _node->childOctreeNode[i][j][k]->pIdx.push_back( _node->pIdx[i_point] );
        } // end for

        _node->pIdx.clear();

        // Assigh childe indices respectively
        // [0][0][0]
        if ( _node->childOctreeNode[0][0][0]->pIdx.size() == 0 ) {
            delete _node->childOctreeNode[0][0][0];
            _node->childOctreeNode[0][0][0] = NULL;

        } else {
            create_octree(  _node->childOctreeNode[0][0][0], _points, _nMinNode,
                            _xMin, _node->centerCoords[0],
                            _yMin, _node->centerCoords[1],
                            _zMin, _node->centerCoords[2]  );
        } // end if

        // [1][0][0]
        if ( _node->childOctreeNode[1][0][0]->pIdx.size() == 0 ) {
            delete _node->childOctreeNode[1][0][0];
            _node->childOctreeNode[1][0][0] = NULL;

        } else {
            create_octree(  _node->childOctreeNode[1][0][0], _points, _nMinNode,
                            _node->centerCoords[0], _xMax,
                            _yMin, _node->centerCoords[1],
                            _zMin, _node->centerCoords[2]  );
        } // end if 

        // [0][1][0]
        if ( _node->childOctreeNode[0][1][0]->pIdx.size() == 0 ) {
            delete _node->childOctreeNode[0][1][0];
            _node->childOctreeNode[0][1][0] = NULL;

        } else {
            create_octree(  _node->childOctreeNode[0][1][0], _points, _nMinNode,
                            _xMin, _node->centerCoords[0],
                            _node->centerCoords[1], _yMax,
                            _zMin, _node->centerCoords[2]  );
        } // end if

        // [1][1][0]
        if ( _node->childOctreeNode[1][1][0]->pIdx.size() == 0 ) {
            delete _node->childOctreeNode[1][1][0];
            _node->childOctreeNode[1][1][0] = NULL;
        
        } else {
            create_octree(  _node->childOctreeNode[1][1][0], _points, _nMinNode,
                            _node->centerCoords[0], _xMax,
                            _node->centerCoords[1], _yMax,
                            _zMin, _node->centerCoords[2]  );
        } // end if

        // [0][0][1]
        if ( _node->childOctreeNode[0][0][1]->pIdx.size() == 0 ) {
            delete _node->childOctreeNode[0][0][1];
            _node->childOctreeNode[0][0][1] = NULL;
        
        } else {
            create_octree(  _node->childOctreeNode[0][0][1], _points, _nMinNode,
                            _xMin, _node->centerCoords[0],
                            _yMin, _node->centerCoords[1],
                            _node->centerCoords[2], _zMax  );
        } // end if

        // [1][0][1]
        if ( _node->childOctreeNode[1][0][1]->pIdx.size() == 0 ) {
            delete _node->childOctreeNode[1][0][1];
            _node->childOctreeNode[1][0][1] = NULL;

        } else {
            create_octree(  _node->childOctreeNode[1][0][1], _points, _nMinNode,
                            _node->centerCoords[0], _xMax,
                            _yMin, _node->centerCoords[1],
                            _node->centerCoords[2], _zMax  );
        } // end if

        // [0][1][1]
        if ( _node->childOctreeNode[0][1][1]->pIdx.size() == 0 ) {
            delete _node->childOctreeNode[0][1][1];
            _node->childOctreeNode[0][1][1] = NULL;
        
        } else {
            create_octree(  _node->childOctreeNode[0][1][1], _points, _nMinNode,
                            _xMin, _node->centerCoords[0],
                            _node->centerCoords[1], _yMax,
                            _node->centerCoords[2], _zMax  );
        } // end if

        // [1][1][1]
        if ( _node->childOctreeNode[1][1][1]->pIdx.size() == 0 ) {
            delete _node->childOctreeNode[1][1][1];
            _node->childOctreeNode[1][1][1] = NULL;
        
        } else {
            create_octree(  _node->childOctreeNode[1][1][1], _points, _nMinNode,
                            _node->centerCoords[0], _xMax,
                            _node->centerCoords[1], _yMax,
                            _node->centerCoords[2], _zMax  );
        } // end if

    } else {

        // Don't create childe further
        for ( i = 0; i < 2; i++ ) {
            for ( j = 0; j < 2; j++ ) {
                for ( k = 0; k < 2; k++ ) {
                    _node->childOctreeNode[i][j][k] = NULL;
                }
            }
        } // end for

    } // end if

} // End create_octree()


void search_node(   double p[], double R2, octreeNode *node, float points[],
                    double xleft, double xright,
                    double yleft, double yright,
                    double zleft, double zright,
		            std::vector<size_t> *nearIdxPtr,
                    std::vector<double> *dist   ) {
    size_t i, pNum;

    if ( node->pIdx.size() == 0 ) {

        // If node has children
        if ( xleft <= node->centerCoords[0] ) {
            
            // Search [0][?][?]
            if ( yleft <= node->centerCoords[1] ) {

                // Search [0][0][?]
                if ( zleft <= node->centerCoords[2] ) {

                    // Search [0][0][0]
                    if ( node->childOctreeNode[0][0][0] != NULL ) {
                        search_node(    p, R2, node->childOctreeNode[0][0][0], points,
                                        xleft, xright, yleft, yright, zleft, zright,
                                        nearIdxPtr, dist    );
                    }

                } // end if

                if ( zright >= node->centerCoords[2] ) {

	                // Search [0][0][1]
                    if ( node->childOctreeNode[0][0][1] != NULL ) {
                        search_node(    p, R2, node->childOctreeNode[0][0][1], points,
                                        xleft, xright, yleft, yright, zleft, zright,
                                        nearIdxPtr, dist    );
                    }
                
                } // end if

            } // end if

            if ( yright >= node->centerCoords[1] ) {
                
                // Search [0][1][?]
                if ( zleft <= node->centerCoords[2] ) {
                    
                    // Search [0][1][0]
                    if ( node->childOctreeNode[0][1][0] != NULL ) {
                        search_node(    p, R2, node->childOctreeNode[0][1][0], points,
                                        xleft, xright, yleft, yright, zleft, zright,
                                        nearIdxPtr, dist    );
                    }

                } // end if

                if ( zright >= node->centerCoords[2] ) {
                    
                    // Search [0][1][1]
                    if ( node->childOctreeNode[0][1][1] != NULL ) {
                        search_node(    p, R2, node->childOctreeNode[0][1][1], points,
                                        xleft, xright, yleft, yright, zleft, zright,
                                        nearIdxPtr, dist    );
                    }

                } // end if

            } // end if

        } // end if

        if ( xright >= node->centerCoords[0] ) {
            
            // Search [1][?][?]
            if ( yleft <= node->centerCoords[1] ) {
                
                // Search [1][0][?]
                if ( zleft <= node->centerCoords[2] ) {
                    
                    // Search [1][0][0]
                    if ( node->childOctreeNode[1][0][0] != NULL ) {
                        search_node(    p, R2, node->childOctreeNode[1][0][0], points,
                                        xleft, xright, yleft, yright, zleft, zright,
                                        nearIdxPtr, dist    );
                    }
                
                } // end if

                if ( zright >= node->centerCoords[2] ) {
                    
                    // Search [1][0][1]
                    if ( node->childOctreeNode[1][0][1] != NULL ) {
                        search_node(    p, R2, node->childOctreeNode[1][0][1], points,
                                        xleft, xright, yleft, yright, zleft, zright,
                                        nearIdxPtr, dist    );
                    }

                } // end if

            } // end if

            if ( yright >= node->centerCoords[1] ) {
                
                // Search [1][1][?]
                if ( zleft <= node->centerCoords[2] ) {
                    
                    // Search [1][1][0]
                    if ( node->childOctreeNode[1][1][0] != NULL ) {
                        search_node(    p, R2, node->childOctreeNode[1][1][0], points,
                                        xleft, xright, yleft, yright, zleft, zright,
                                        nearIdxPtr, dist    );
                    }

                } // end if

                if ( zright >= node->centerCoords[2] ) {
                    
                    // Search [1][1][1]
                    if ( node->childOctreeNode[1][1][1] != NULL ) {
                        search_node(    p, R2, node->childOctreeNode[1][1][1], points,
                                        xleft, xright, yleft, yright, zleft, zright,
                                        nearIdxPtr, dist    );
                    }

                } // end if

            } // end if

        } // end if
    
    } else {

        // If node is a leaf
        pNum = node->pIdx.size();

        for ( i = 0; i < pNum; i++ ) {
            double pt[3] = {    (double)points[node->pIdx[i] * 3],      
                                (double)points[node->pIdx[i] * 3 + 1],  
                                (double)points[node->pIdx[i] * 3 + 2]   };
            double d0 = dist2( p, pt );

            if ( d0 < R2 ) {
                //if (dist2(p, &(points[node->pIdx[i] * 3])) < R2) {
                nearIdxPtr->push_back( node->pIdx[i] );
                dist->push_back( sqrt( d0 ) );
            } 
        } // end for

    } // end if

} // End search_node()


void search_points( double p[], double R, float points[],
                    octreeNode *node, std::vector <size_t> *nearIdxPtr,
                    std::vector<double> *dist   ) {

    double xleft, xright, yleft, yright, zleft, zright, R2;

    xleft   = p[0] - R;
    xright  = p[0] + R;
    yleft   = p[1] - R;
    yright  = p[1] + R;
    zleft   = p[2] - R;
    zright  = p[2] + R;
    R2      = R * R;

    search_node(    p, R2, node, points,
                    xleft, xright, yleft, yright, zleft, zright,
                    nearIdxPtr, dist    );

} // End search_points()