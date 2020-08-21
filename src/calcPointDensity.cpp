#include "calcPointDensity.h"

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h> 
#include <time.h>
#include <fstream>
#include <cstdlib>

#include <numeric>
#include "octree.h"

// #define CREATE_HISTOGRAM

const int INTERVAL_POINTS = 5e6;
const int MIN_NODE = 15;

calcPointDensity::calcPointDensity():
    m_max_point_num( -1 ),
    m_min_point_num( 1e7 ),
    m_max_avg_dist( -1.0f ),
    m_min_avg_dist( 1e7 )
{}

void calcPointDensity::setSearchType( const SearchType _search_type ) {
    m_search_type = _search_type;
}

void calcPointDensity::setSearchRadius( const double _distance ) {
    m_searchRadius = _distance;
}

void calcPointDensity::setSearchRadius( const double _divide_value, const kvs::Vector3f _bbmin, const kvs::Vector3f _bbmax ) {
    kvs::Vector3f bb = _bbmax - _bbmin;
    m_searchRadius   = bb.length() / _divide_value;

    std::cout << "> search radius: " << m_searchRadius;
    std::cout << " (= " << bb.length() << "/" << _divide_value << ")" << std::endl;
}

void calcPointDensity::setNearestK( const int _k ) {
    m_nearestK = _k;
}

void calcPointDensity::calcWithOctree( const kvs::PolygonObject* _ply ) {
    kvs::ValueArray<kvs::Real32> coords = _ply->coords();
    float *points = coords.data();
    m_number_of_points  = _ply->numberOfVertices();
    kvs::Vector3f minBB = _ply->minObjectCoord();
    kvs::Vector3f maxBB = _ply->maxObjectCoord();

    double *rangeBB = new double[6];
    rangeBB[0] = (double)minBB.x();
    rangeBB[1] = (double)maxBB.x();
    rangeBB[2] = (double)minBB.y();
    rangeBB[3] = (double)maxBB.y();
    rangeBB[4] = (double)minBB.z();
    rangeBB[5] = (double)maxBB.z();

    // Create octree
    octree *myTree = new octree(    points,             /* float _points[] */
                                    m_number_of_points, /* size_t _Points */
                                    rangeBB,            /* double _range[] */
                                    MIN_NODE            /* int _nMinNode */ );

    std::cout << "\n";
    std::cout << "Now Octree Searching... " << std::endl;
    clock_t start = clock(); // Start time count
    for ( size_t i = 0; i < m_number_of_points; i++ ) {
        if ( i == m_number_of_points ) --i;

        double point[3] = { coords[3 * i],
                            coords[3 * i + 1],
                            coords[3 * i + 2] };

        // Search neighborhood points
        std::vector<size_t> neighborhoodIdx;
        std::vector<double> dist;
        search_points(  point,              /* double p[] */
                        m_searchRadius,     /* double R */
                        points,             /* float points[] */
                        myTree->octreeRoot, /* octreeNode *Node */
                        &neighborhoodIdx,   /* std::vector<size_t> *nearIndPtr */
                        &dist               /* std::vector<double> *dist */ );
        int num_of_neighborhood_points = (int)neighborhoodIdx.size();

        // for ( size_t j = 0; j < num_of_neighborhood_points; j++ ) {
        //     double x = coords[3 * neighborhoodIdx[j]];
        //     double y = coords[3 * neighborhoodIdx[j] + 1];
        //     double z = coords[3 * neighborhoodIdx[j] + 2];
        // }

        // Count the searched neighborhood points
        m_point_densities.push_back( num_of_neighborhood_points );

        // Display progress
        if ( !((i + 1) % INTERVAL_POINTS) )
            std::cout << i + 1 << ", " << num_of_neighborhood_points << " (points)\n";
    } // end for

    // End time clock
    clock_t end = clock();
    std::cout << "Done Octree Search! (" << (double)(end - start) / CLOCKS_PER_SEC / 60.0 << " [minute])" << std::endl;
}

void calcPointDensity::calcWithPCL( const kvs::PolygonObject* _ply ) {
    std::vector<pcl::PointXYZ>  points4pcl;
    std::vector<float>          normal;

    kvs::ValueArray<kvs::Real32> coords  = _ply->coords(); 
    kvs::ValueArray<kvs::Real32> normals = _ply->normals();
    size_t point_num = _ply->numberOfVertices();
    m_number_of_points = point_num;
    
    bool hasNormal = false;
    if ( point_num == _ply->numberOfNormals() ) hasNormal = true;
    
    // kvs::ValueArray<kvs::Real32> to pcl::PointXYZ( x, y, z )
    for ( size_t i = 0; i < point_num; i++ ) {
        float x     = coords[3*i];
        float y     = coords[3*i+1];
        float z     = coords[3*i+2];
        float nx    = 0.0;
        float ny    = 0.0;
        float nz    = 0.0;    
        if ( hasNormal ) {
            nx = normals[3*i];
            ny = normals[3*i+1];
            nz = normals[3*i+2];
        }

        points4pcl.push_back( pcl::PointXYZ( x, y, z ) );
        normal.push_back( nx );
        normal.push_back( ny );
        normal.push_back( nz );      
    }

    // Calculate point density
    exec( points4pcl );
} // End calc( kvs::PolygonObject* _ply )

void calcPointDensity::exec( std::vector<pcl::PointXYZ> &_points ) {
    // http://pointclouds.org/documentation/tutorials/basic_structures.php#basic-structures
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);  
    cloud->width  = m_number_of_points; 
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);
    // Copy "std::vector<pcl::PointXYZ>" to "pcl::PointCloud<pcl::PointXYZ>::Ptr"
    std::copy(_points.begin(), _points.end(), cloud->points.begin());
    
    // Set KdTree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud);
    pcl::PointXYZ searchPoint;
    std::vector<int>    pointIndex;
    std::vector<float>  pointSquaredDistance;

    // Search nearest points by using kdTree
    clock_t start = clock(); // Start time count
    std::cout << "\nNow searching and calculating ..." << std::endl;
    int display_interval = 1e06;
    for ( size_t i = 0; i < m_number_of_points; i++ ) {
        // Processing ratio
        double processing_ratio = 100.0 * (double)i / (double)m_number_of_points;

        // Display messages
        if ( !(i % display_interval) && i > 0 ) { 
            std::cout << "*** Num. of processed points: " << i;
            std::cout << " [" << processing_ratio << " %]" << std::endl;
        }

        searchPoint.x = _points[i].x;
        searchPoint.y = _points[i].y;
        searchPoint.z = _points[i].z;
            
        // Search nearest points
        // http://pointclouds.org/documentation/tutorials/kdtree_search.php#kdtree-search

        // RadiusSearch
        int num_of_neighborhood=0;
        if ( m_search_type == PCL_RadiusSearch ) {
            num_of_neighborhood = kdtree.radiusSearch ( searchPoint, 
                                                        m_searchRadius,
                                                        pointIndex, 
                                                        pointSquaredDistance );

            // Count the number of points in the search sphere
            m_point_densities.push_back(num_of_neighborhood);
        // end RadiusSearch

        // NearestKSearch
        } else if ( m_search_type == PCL_NearestKSearch ) {
            num_of_neighborhood = kdtree.nearestKSearch (   searchPoint,
                                                            m_nearestK,
                                                            pointIndex, 
                                                            pointSquaredDistance );

            // Calc point density
            double distance_sum = 0.0f;
            for ( int j = 0; j < m_nearestK; j++ ) {
                double distanceXYZ[3];
                distanceXYZ[0] = _points[i].x - _points[ pointIndex[j] ].x;
                distanceXYZ[1] = _points[i].y - _points[ pointIndex[j] ].y;
                distanceXYZ[2] = _points[i].z - _points[ pointIndex[j] ].z;
            
                double distance = sqrt( distanceXYZ[0]*distanceXYZ[0] + 
                                        distanceXYZ[1]*distanceXYZ[1] + 
                                        distanceXYZ[2]*distanceXYZ[2] );
                distance_sum += distance;
            }
            double distance_avg = distance_sum / (num_of_neighborhood - 1.0);

            m_point_densities.push_back(distance_avg);
        } // end if
        // end NearestKSearch
    } // end for

    // End time clock
    clock_t end = clock();
    std::cout << "Done calculating point densities! (" << (double)(end - start) / CLOCKS_PER_SEC / 60.0 << " [minute])" << std::endl;
} // End exec( std::vector<pcl::PointXYZ> &_points )

void calcPointDensity::calcMinMax4PointDensities() {

    if ( m_search_type == Octree || m_search_type == PCL_RadiusSearch ) {
        m_min_point_num = *std::min_element( m_point_densities.begin(), m_point_densities.end() );
        m_max_point_num = *std::max_element( m_point_densities.begin(), m_point_densities.end() );

        m_min_value = (double)m_min_point_num;
        m_max_value = (double)m_max_point_num;

    } else if ( m_search_type == PCL_NearestKSearch ) {
        m_min_avg_dist = *std::min_element( m_point_densities.begin(), m_point_densities.end() );
        m_max_avg_dist = *std::max_element( m_point_densities.begin(), m_point_densities.end() );

        m_min_value = m_min_avg_dist;
        m_max_value = m_max_avg_dist;
    }

    std::cout << "\n";
    std::cout << "Min value: " << m_min_value << std::endl;
    std::cout << "Max value: " << m_max_value << std::endl;

} // End calcMaxMin4PointDensities()

void calcPointDensity::removeOutlier4PointDensities( const int _sigma_section4outlier ) {

    // Calc. average and standard deviation
    double avg = 0.0, var = 0.0, std = 0.0;
    for ( const double &i : m_point_densities ) {
        avg += i;
        var += i * i;
    }
    avg = avg / m_point_densities.size();
    var = var / m_point_densities.size() - avg * avg;
    std = sqrt(var);
    std::cout << "Average: " << avg << std::endl;
    // std::cout << "Variance: " << var << std::endl;
    std::cout << "Standard Deviation: " << std << std::endl;

    // Remove outlier
    if ( _sigma_section4outlier != 0 ) {
        int threshold4outlier;

        if ( _sigma_section4outlier == 1 ) {
            threshold4outlier = avg + 1 * std;

        } else if ( _sigma_section4outlier == 2 ) {
            threshold4outlier = avg + 2 * std;

        } else if ( _sigma_section4outlier == 3 ) {
            threshold4outlier = avg + 3 * std;
        } // end if

        for ( size_t i = 0; i < m_point_densities.size(); i++ ) {
            if ( m_point_densities[i] > threshold4outlier )
                m_point_densities[i] = threshold4outlier;
        } // end for

        // Update max point density
        if ( m_search_type == Octree || m_search_type == PCL_RadiusSearch ) {
            m_max_point_num = threshold4outlier;
            m_max_value = m_max_point_num;
            std::cout << "\n";
            std::cout << "Removed outliers for point density vector." << std::endl;
            std::cout << "New max value: " << m_max_point_num << std::endl;
        
        } else if ( m_search_type == PCL_NearestKSearch ) {
            m_max_avg_dist = threshold4outlier;
            m_max_value = m_max_avg_dist;
        } // end if

    }// end if

} // End adjustPointDensity()

void calcPointDensity::normalizePointDensities() {
#ifdef CREATE_HISTOGRAM
    std::ofstream fout_before( "SPBR_DATA/tmp/norm_before.csv" );
    // std::ofstream fout_after( "SPBR_DATA/tmp/norm_after.csv" );
#endif

#ifdef CREATE_HISTOGRAM
    std::cout << "\n";
    std::cout << "Writing csv file ..." << std::endl;
#endif
    for ( size_t i = 0; i < m_point_densities.size(); i++ ) {
#ifdef CREATE_HISTOGRAM
        fout_before << m_point_densities[i] << std::endl;
#endif
        // Normalize
        if ( m_search_type == Octree || m_search_type == PCL_RadiusSearch ) {
            m_point_densities[i] -= m_min_value;
            m_point_densities[i] /= (m_max_value - m_min_value);

        } else if ( m_search_type == PCL_NearestKSearch ) {
            m_point_densities[i] /= m_max_avg_dist;
        }

#ifdef CREATE_HISTOGRAM
        fout_after << m_point_densities[i] << std::endl;
#endif

    } // end for
#ifdef CREATE_HISTOGRAM
    std::cout << "Done writing csv file!" << std::endl;
#endif

    std::cout << "\n";
    std::cout << "Normalized point density vector." << std::endl;
    std::cout << "Updated min and max value." << std::endl;
    m_min_value = *std::min_element( m_point_densities.begin(), m_point_densities.end() );
    m_max_value = *std::max_element( m_point_densities.begin(), m_point_densities.end() );
    std::cout << "Min value: " << m_min_value << std::endl;
    std::cout << "Max value: " << m_max_value << std::endl;
} // End normalizePointDencity()