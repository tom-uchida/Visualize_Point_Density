#pragma once

#include <kvs/PolygonObject>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h> 
#include <vector>

class calcPointDensity {
public:
    enum SearchType {
        Octree              = 0,
        PCL_RadiusSearch    = 1,
        PCL_NearestKSearch  = 2
    };

public:
    calcPointDensity( void );

    void    setSearchType( const SearchType _search_type ) { m_search_type = _search_type; }
    void    setSearchRadius( const double _distance ) { m_searchRadius = _distance; }
    void    setSearchRadius( const double _divide_value, const kvs::Vector3f _bbmin, const kvs::Vector3f _bbmax );
    void    setNearestK( const int _k ) { m_nearestK = _k; }
    void    calcWithOctree( const kvs::PolygonObject* _ply );
    void    calcWithPCL( const kvs::PolygonObject* _ply );
    void    calcMinMax4PointDensities( void );
    void    removeOutlier4PointDensities( const int _sigma_section4outlier );
    void    normalizePointDensities( void );

    std::vector<double> getPointDensities( void ) { return m_point_densities; }
    double  getMinValue( void ) const { return m_min_value; }
    double  getMaxValue( void ) const { return m_max_value; }
    int     getMinPointNum( void ) const { return m_min_point_num; }
    int     getMaxPointNum( void ) const { return m_max_point_num; }
    double  getMinAvgDistance( void ) const { return m_min_avg_dist; }
    double  getMaxAvgDistance( void ) const { return m_max_avg_dist; }

private:
    SearchType           m_search_type;
    size_t               m_number_of_points;
    std::vector<double>  m_point_densities;
    double               m_searchRadius;
    int                  m_nearestK;
    double               m_max_value;
    double               m_min_value;
    int                  m_max_point_num;
    int                  m_min_point_num;
    double               m_max_avg_dist;
    double               m_min_avg_dist;

private:
    void exec( std::vector<pcl::PointXYZ> &_point );
};