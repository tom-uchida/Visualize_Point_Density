#pragma once

#include <kvs/PolygonObject>
#include <vector>

class calcPointDensity {

public:
    calcPointDensity( void );

    void    setSearchRadius( const double _divide_value, const kvs::Vector3f _bb_dia_vec );
    void    setNearestK( const int _k ) { m_nearestK = _k; }
    void    exec( const kvs::PolygonObject* _ply );
    void    calcMinMax( void );
    void    removeOutlier( const int _sigma_section4outlier );
    void    normalize( void );

    std::vector<double> getPointDensities( void ) const { return m_point_densities; }
    double  getMinValue( void ) const { return m_min_value; }
    double  getMaxValue( void ) const { return m_max_value; }
    int     getMinPointNum( void ) const { return m_min_point_num; }
    int     getMaxPointNum( void ) const { return m_max_point_num; }
    double  getMinAvgDistance( void ) const { return m_min_avg_dist; }
    double  getMaxAvgDistance( void ) const { return m_max_avg_dist; }

private:
    size_t               m_number_of_points;
    std::vector<double>  m_point_densities;
    double               m_search_radius;
    int                  m_nearestK;
    double               m_max_value;
    double               m_min_value;
    int                  m_max_point_num;
    int                  m_min_point_num;
    double               m_max_avg_dist;
    double               m_min_avg_dist;
};