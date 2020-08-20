#ifndef _calcPointDensity_H__
#define _calcPointDensity_H__

#include <kvs/PolygonObject>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h> 
#include <vector>

class calcPointDensity {
public:
    enum SearchType {
        Octree = 0,
        PCL_RadiusSearch = 1,
        PCL_NearestKSearch = 2,
    };

public:
    calcPointDensity( void );
    calcPointDensity( kvs::PolygonObject* _ply );

    void    setSearchType( const SearchType _type );
    void    setSearchRadius( const double _distance );
    void    setSearchRadius( const double _divide_value, const kvs::Vector3f _bbmin, const kvs::Vector3f _bbmax );
    void    setNearestK( const int _k );
    void    calcWithOctree( const kvs::PolygonObject* _ply );
    void    calcWithPCL( const kvs::PolygonObject* _ply );
    void    calcMinMax4PointDensities( void );
    void    removeOutlier4PointDensities( const int _sigma_section4outlier );
    void    normalizePointDensities( void );

    int     getMaxPointNum( void ) { return m_max_point_num; }
    int     getMinPointNum( void ) { return m_min_point_num; }
    double  getMaxAvgDistance( void ) { return m_max_avg_dist; }
    double  getMinAvgDistance( void ) { return m_min_avg_dist; }
    std::vector<double> getPointDensities( void ) { return m_point_densities; }

private:
    SearchType           m_type;
    size_t               m_number_of_points;
    std::vector<double>  m_point_densities;
    double               m_searchRadius;
    int                  m_nearestK;
    int                  m_max_point_num;
    int                  m_min_point_num;
    double               m_max_avg_dist;
    double               m_min_avg_dist;

private:
    void exec( std::vector<pcl::PointXYZ> &_point );
};

#endif