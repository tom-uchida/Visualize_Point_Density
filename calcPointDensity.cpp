#include "calcPointDensity.h"

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h> 
#include <time.h>
#include <fstream>
#include <cstdlib>

#define HISTOGRAM_MODE

calcPointDensity::calcPointDensity( kvs::PolygonObject* _ply ):
    m_max_point_num( -1 ),
    m_min_point_num( 1e06 ),
    m_max_avg_dist( -1.0f ),
    m_min_avg_dist( 1e05 )
{}

void calcPointDensity::setSearchType( SearchType _type) {
    m_type = _type;
}

void calcPointDensity::setSearchRadius( double _distance ) {
    m_searchRadius = _distance;
}

void calcPointDensity::setSearchRadius( double _divide_value, kvs::Vector3f _bbmin, kvs::Vector3f _bbmax  ) {
    kvs::Vector3f bb        = _bbmax - _bbmin;  // Diagonal vector
    double diagonal_length  = bb.length();      // Diagonal length
    m_searchRadius          = diagonal_length / _divide_value; // Diagonal length / divide
    std::cout << "> search_radius : " << m_searchRadius;
    std::cout << " (= " << diagonal_length << " / " << _divide_value << ")" << std::endl;
}

void calcPointDensity::setNearestK( int _k) {
    m_nearestK = _k;
}

void calcPointDensity::calc( kvs::PolygonObject* _ply ) {
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

    // Start time count
    std::cout << "\nClock start" << std::endl;
    clock_t start = clock();
    std::cout << "Now searching and calculating ..." << std::endl;
    
    // Search nearest points by using kdTree
    int display_interval = 1e06;
    for ( size_t i = 0; i < m_number_of_points; i++ ) {
        // Processing ratio
        double processing_ratio = 100.0 * (double)i / (double)m_number_of_points;

        // Display messages
        if ( !(i % display_interval) && i > 0 ) { 
            std::cout << "*** Num. of processed points : " << i;
            std::cout << " [" << processing_ratio << " %]" << std::endl;
        }

        searchPoint.x = _points[i].x;
        searchPoint.y = _points[i].y;
        searchPoint.z = _points[i].z;
            
        // Search nearest points
        // http://pointclouds.org/documentation/tutorials/kdtree_search.php#kdtree-search

        // RadiusSearch
        int num_of_neighborhood=0;
        if ( m_type == RadiusSearch ) {
            num_of_neighborhood = kdtree.radiusSearch ( searchPoint, 
                                                        m_searchRadius,
                                                        pointIndex, 
                                                        pointSquaredDistance );

            // Count the number of points in the search sphere
            m_point_densities.push_back(num_of_neighborhood);
        // end RadiusSearch

        // NearestKSearch
        } else if ( m_type == NearestKSearch ) {
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
    std::cout << "Done! " << (double)(end - start) / CLOCKS_PER_SEC / 60.0 << " (minute)" << std::endl;

    if ( m_type == RadiusSearch ) {
        // Show result
        m_max_point_num = *std::max_element(m_point_densities.begin(), m_point_densities.end());
        m_min_point_num = *std::min_element(m_point_densities.begin(), m_point_densities.end());
        std::cout << "\nMax num. of points";
        std::cout << " : " << m_max_point_num    << std::endl;
        std::cout << "Min num. of points";
        std::cout << " : " << m_min_point_num    << std::endl;

    } else if ( m_type == NearestKSearch ) {
         // Show result
        m_max_avg_dist = *std::max_element(m_point_densities.begin(), m_point_densities.end());
        m_min_avg_dist = *std::min_element(m_point_densities.begin(), m_point_densities.end());
        std::cout << "\nMax avg. distance";
        std::cout << " : " << m_max_avg_dist    << std::endl;
        std::cout << "Min avg. distance";
        std::cout << " : " << m_min_avg_dist    << std::endl;
    } // end if
} // End exec( std::vector<pcl::PointXYZ> &_points )

void calcPointDensity::normalizePointDensities() {
    std::ofstream fout_before( "SPBR_DATA/norm_before.csv" );
    std::ofstream fout_after( "SPBR_DATA/norm_after.csv" );

    for (int i = 0; i < m_point_densities.size(); i++) {
#ifdef HISTOGRAM_MODE
        fout_before << m_point_densities[i] << std::endl;
#endif
        // Normalize
        if ( m_type == RadiusSearch ) {
            m_point_densities[i] = m_point_densities[i] / m_max_point_num;
        } else if ( m_type == NearestKSearch ) {
            m_point_densities[i] = m_point_densities[i] / m_max_avg_dist;
        }

#ifdef HISTOGRAM_MODE
        fout_after << m_point_densities[i] << std::endl;
#endif
    }
} // End normalizePointDencities()