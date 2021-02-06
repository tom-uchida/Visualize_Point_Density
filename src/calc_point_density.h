#pragma once

#include <kvs/PolygonObject>
#include <vector>

class calcPointDensity {

public:
    calcPointDensity( void );

    void    setSearchRadius( const double _divide_value, const kvs::Vector3f _bb_dia_vec );
    void    exec( const kvs::PolygonObject* _ply );
    void    calcMinMax( void );
    void    removeOutlier( const int _sigma_section4outlier );
    void    normalize( void );

    std::vector<double> getPointDensities( void ) const { return m_point_densities; }
    int     getMinPointDensity( void ) const { return m_min_point_density; }
    int     getMaxPointDensity( void ) const { return m_max_point_density; }

private:
    size_t               m_number_of_points;
    std::vector<double>  m_point_densities;
    double               m_search_radius;
    int                  m_max_point_density;
    int                  m_min_point_density;
};