#ifndef _calcPointDensity_H__
#define _calcPointDensity_H__

#include <kvs/PolygonObject>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h> 
#include <vector>

class calcPointDensity {
public:
    enum SearchType {
        RadiusSearch = 0,
        NearestKSearch = 1,
    };

public:
    calcPointDensity( kvs::PolygonObject* _ply );

    void setSearchType( SearchType _type );
    void setSearchRadius( double _distance );
    void setSearchRadius( double _divide_value, kvs::Vector3f _bbmin, kvs::Vector3f _bbmax );
    void setNearestK( int _k );
    void calc( kvs::PolygonObject* _ply );
    void normalizePointDensities( void );
    int     getMaxPointNum( void ) { return m_max_point_num; }
    int     getMinPointNum( void ) { return m_min_point_num; }
    double  getMaxAvgDistance( void ) { return m_max_avg_dist; }
    double  getMinAvgDistance( void ) { return m_min_avg_dist; }
    std::vector<float> getPointDensities( void ) { return m_point_densities; }

private:
    SearchType          m_type;
    size_t              m_number_of_points;
    std::vector<float>  m_point_densities;
    double              m_searchRadius;
    int                 m_nearestK;
    int                 m_max_point_num;
    int                 m_min_point_num;
    double              m_max_avg_dist;
    double              m_min_avg_dist;

private:
    void exec( std::vector<pcl::PointXYZ> &_point );
};
#endif