#include <iostream>
#include <vector>
#include <cmath>
#include "create_octree.h"
#include "vec_ops.h"

void create_octree(octreeNode *node, float points[], int nMin,
		   double xMin, double xMax, double yMin, double yMax,
		   double zMin, double zMax) {

  // Assign points in the node to its children
  size_t i, j, k, ip, np, ind;

  np = node->pInd.size();

  if (np > nMin) {
    // Create each cell
    for (i = 0; i < 2; i++) {
      for (j = 0; j < 2; j++) {
	for (k = 0; k < 2; k++) {
	  node->cOctreeNode[i][j][k] = new octreeNode;
	  node->cOctreeNode[i][j][k]->pInd.clear();
	}
      }
    }

    node->c[0] = (xMin + xMax) * 0.5;
    node->c[1] = (yMin + yMax) * 0.5;
    node->c[2] = (zMin + zMax) * 0.5;

    // Assign points into children
    for (ip = 0; ip < np; ip++) {
      ind = node->pInd[ip] * 3;
      i = ((points[ind] < node->c[0]) ? 0: 1);
      j = ((points[ind + 1] < node->c[1]) ? 0: 1);
      k = ((points[ind + 2] < node->c[2]) ? 0: 1);
      node->cOctreeNode[i][j][k]->pInd.push_back(node->pInd[ip]);
    }

    node->pInd.clear();

    // Assigh childe indices respectively
    // [0][0][0]
    if (node->cOctreeNode[0][0][0]->pInd.size() == 0) {
      delete node->cOctreeNode[0][0][0];
      node->cOctreeNode[0][0][0] = NULL;
    }
    else {
      create_octree(node->cOctreeNode[0][0][0], points, nMin,
		    xMin, node->c[0], yMin, node->c[1],
		    zMin, node->c[2]);
    }

    // [1][0][0]
    if (node->cOctreeNode[1][0][0]->pInd.size() == 0) {
      delete node->cOctreeNode[1][0][0];
      node->cOctreeNode[1][0][0] = NULL;
    }
    else {
      create_octree(node->cOctreeNode[1][0][0], points, nMin,
		    node->c[0], xMax, yMin, node->c[1],
		    zMin, node->c[2]);
    }

    // [0][1][0]
    if (node->cOctreeNode[0][1][0]->pInd.size() == 0) {
      delete node->cOctreeNode[0][1][0];
      node->cOctreeNode[0][1][0] = NULL;
    }
    else {
      create_octree(node->cOctreeNode[0][1][0], points, nMin,
		    xMin, node->c[0], node->c[1], yMax,
		    zMin, node->c[2]);
    }

    // [1][1][0]
    if (node->cOctreeNode[1][1][0]->pInd.size() == 0) {
      delete node->cOctreeNode[1][1][0];
      node->cOctreeNode[1][1][0] = NULL;
    }
    else {
      create_octree(node->cOctreeNode[1][1][0], points, nMin,
		    node->c[0], xMax, node->c[1], yMax,
		    zMin, node->c[2]);
    }

    // [0][0][1]
    if (node->cOctreeNode[0][0][1]->pInd.size() == 0) {
      delete node->cOctreeNode[0][0][1];
      node->cOctreeNode[0][0][1] = NULL;
    }
    else {
      create_octree(node->cOctreeNode[0][0][1], points, nMin,
		    xMin, node->c[0], yMin, node->c[1],
		    node->c[2], zMax);
    }

    // [1][0][1]
    if (node->cOctreeNode[1][0][1]->pInd.size() == 0) {
      delete node->cOctreeNode[1][0][1];
      node->cOctreeNode[1][0][1] = NULL;
    }
    else {
      create_octree(node->cOctreeNode[1][0][1], points, nMin,
		    node->c[0], xMax, yMin, node->c[1],
		    node->c[2], zMax);
    }

    // [0][1][1]
    if (node->cOctreeNode[0][1][1]->pInd.size() == 0) {
      delete node->cOctreeNode[0][1][1];
      node->cOctreeNode[0][1][1] = NULL;
    }
    else {
      create_octree(node->cOctreeNode[0][1][1], points, nMin,
		    xMin, node->c[0], node->c[1], yMax,
		    node->c[2], zMax);
    }

    // [1][1][1]
    if (node->cOctreeNode[1][1][1]->pInd.size() == 0) {
      delete node->cOctreeNode[1][1][1];
      node->cOctreeNode[1][1][1] = NULL;
    }
    else {
      create_octree(node->cOctreeNode[1][1][1], points, nMin,
		    node->c[0], xMax, node->c[1], yMax,
		    node->c[2], zMax);
    }

  }
  else {

    // Don't create childe further
    for (i = 0; i < 2; i++) {
      for (j = 0; j < 2; j++) {
	for (k = 0; k < 2; k++) {
	  node->cOctreeNode[i][j][k] = NULL;
	}
      }
    }

  }
		    
  return;
}


void search_node(double p[], double R2, octreeNode *node, float points[],
		 double xleft, double xright, double yleft, double yright,
		 double zleft, double zright,
		 std::vector<size_t> *nearIndPtr,
                 std::vector<double> *dist ) {

  size_t i, pNum;

  if (node->pInd.size() == 0) {
    // If node has children

    if (xleft <= node->c[0]) {
      // Search [0][?][?]

      if (yleft <= node->c[1]) {
	// Search [0][0][?]

	if (zleft <= node->c[2]) {
	  // Search [0][0][0]
	  if (node->cOctreeNode[0][0][0] != NULL) {
	    search_node(p, R2, node->cOctreeNode[0][0][0], points,
			xleft, xright, yleft, yright, zleft, zright,
			nearIndPtr, dist);
	  }
	}

	if (zright >= node->c[2]) {
	  // Search [0][0][1]
	  if (node->cOctreeNode[0][0][1] != NULL) {
	    search_node(p, R2, node->cOctreeNode[0][0][1], points,
			xleft, xright, yleft, yright, zleft, zright,
			nearIndPtr, dist );
	  }
	}

      }

      if (yright >= node->c[1]) {
	// Search [0][1][?]

	if (zleft <= node->c[2]) {
	  // Search [0][1][0]
	  if (node->cOctreeNode[0][1][0] != NULL) {
	    search_node(p, R2, node->cOctreeNode[0][1][0], points,
			xleft, xright, yleft, yright, zleft, zright,
			nearIndPtr, dist);
	  }
	}

	if (zright >= node->c[2]) {
	  // Search [0][1][1]
	  if (node->cOctreeNode[0][1][1] != NULL) {
	    search_node(p, R2, node->cOctreeNode[0][1][1], points,
			xleft, xright, yleft, yright, zleft, zright,
			nearIndPtr, dist );
	  }
	}

      }

    }

    if (xright >= node->c[0]) {
      // Search [1][?][?]

      if (yleft <= node->c[1]) {
	// Search [1][0][?]

	if (zleft <= node->c[2]) {
	  // Search [1][0][0]
	  if (node->cOctreeNode[1][0][0] != NULL) {
	    search_node(p, R2, node->cOctreeNode[1][0][0], points,
			xleft, xright, yleft, yright, zleft, zright,
			nearIndPtr, dist);
	  }
	}

	if (zright >= node->c[2]) {
	  // Search [1][0][1]
	  if (node->cOctreeNode[1][0][1] != NULL) {
	    search_node(p, R2, node->cOctreeNode[1][0][1], points,
			xleft, xright, yleft, yright, zleft, zright,
			nearIndPtr, dist);
	  }
	}

      }

      if (yright >= node->c[1]) {
	// Search [1][1][?]

	if (zleft <= node->c[2]) {
	  // Search [1][1][0]
	  if (node->cOctreeNode[1][1][0] != NULL) {
	    search_node(p, R2, node->cOctreeNode[1][1][0], points,
			xleft, xright, yleft, yright, zleft, zright,
			nearIndPtr, dist);
	  }
	}

	if (zright >= node->c[2]) {
	  // Search [1][1][1]
	  if (node->cOctreeNode[1][1][1] != NULL) {
	    search_node(p, R2, node->cOctreeNode[1][1][1], points,
			xleft, xright, yleft, yright, zleft, zright,
			nearIndPtr, dist);
	  }
	}

      }

    }

  }

  else {
    // If node is a leaf
    pNum = node->pInd.size();
    for (i = 0; i < pNum; i++) {
      double pt[3] = { (double)points[node->pInd[i] * 3],      
                       (double)points[node->pInd[i] * 3 + 1],  
                       (double)points[node->pInd[i] * 3 + 2] };
      double d0 = dist2( p, pt );
      if( d0 < R2 ) {
      //if (dist2(p, &(points[node->pInd[i] * 3])) < R2) {
	nearIndPtr->push_back(node->pInd[i]);
        dist->push_back( sqrt( d0 ) );
      }
    }

  }


  return;
}


void search_points(double p[], double R, float points[],
		   octreeNode *node, std::vector <size_t> *nearIndPtr,
                   std::vector<double> *dist) {

  double xleft, xright, yleft, yright, zleft, zright, R2;

  xleft = p[0] - R;
  xright = p[0] + R;
  yleft = p[1] - R;
  yright = p[1] + R;
  zleft = p[2] - R;
  zright = p[2] + R;
  R2 = R * R;

  search_node(p, R2, node, points,
	      xleft, xright, yleft, yright, zleft, zright,
	      nearIndPtr, dist);

  return;
}
