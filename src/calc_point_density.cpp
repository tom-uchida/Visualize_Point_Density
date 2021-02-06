#include "calc_point_density.h"

#include <vector>
#include <time.h>
#include <fstream>
#include <cstdlib>

#include <numeric>
#include "octree.h"

calcPointDensity::calcPointDensity():
    m_max_point_density( -1 ),
    m_min_point_density( 1e7 )
{

}

void calcPointDensity::setSearchRadius( const double _divide_value, const kvs::Vector3f _bb_dia_vec ) {
    m_search_radius = _bb_dia_vec.length() / _divide_value;

    std::cout << " search radius: " << m_search_radius;
    std::cout << " ( = " << _bb_dia_vec.length() << " / " << _divide_value << " )" << std::endl;
}

void calcPointDensity::exec( const kvs::PolygonObject* _ply ) {
    m_number_of_points = _ply->numberOfVertices();
    const kvs::Vector3f BB_min = _ply->minObjectCoord();
    const kvs::Vector3f BB_max = _ply->maxObjectCoord();
    kvs::ValueArray<kvs::Real32> coords = _ply->coords();
    float *points = coords.data();
    
    double *BB_range = new double[6];
    BB_range[0] = (double)BB_min.x();
    BB_range[1] = (double)BB_max.x();
    BB_range[2] = (double)BB_min.y();
    BB_range[3] = (double)BB_max.y();
    BB_range[4] = (double)BB_min.z();
    BB_range[5] = (double)BB_max.z();

    // Create Octree
    octree *my_tree = new octree(
        points,             /* float    _points[] */
        m_number_of_points, /* size_t   _nPoints  */
        BB_range,           /* double   _range[]  */
        MIN_NODE            /* int      _nMinNode */
    );

    // Octree search
    std::cout << "\n";
    std::cout << "Now Octree Searching...\n";
    const clock_t start = clock();
    for ( size_t i = 0; i < m_number_of_points; i++ ) {
        if ( i == m_number_of_points ) --i;

        double point[3] = {
            coords[ 3 * i ],
            coords[ 3 * i + 1 ],
            coords[ 3 * i + 2 ]
        };

        // Search neighborhood points
        std::vector<size_t> neighborhood_index;
        std::vector<double> dist;
        search_points( 
            my_tree->octreeRoot, /* octreeNode           *_node          */
            points,              /* float                _points[]       */
            m_search_radius,     /* const double         _searchRadius   */
            point,               /* double               _point[]        */
            &neighborhood_index, /* std::vector<size_t>  *_nearIndPtr    */
            &dist                /* std::vector<double>  *_dist          */
        );

        // for ( size_t j = 0; j < num_of_neighborhood_points; j++ ) {
        //     double x = coords[3 * neighborhood_index[j]    ];
        //     double y = coords[3 * neighborhood_index[j] + 1];
        //     double z = coords[3 * neighborhood_index[j] + 2];
        // }

        // Save the number of neighborhood points
        const int num_of_neighborhood_points = (int)neighborhood_index.size();
        m_point_densities.push_back( num_of_neighborhood_points );

        // Display progress
        if ( !( (i + 1) % INTERVAL ) )
            std::cout << " " << i + 1 << ", " << num_of_neighborhood_points << " (points)\n";
    } // end for

    const clock_t end = clock();
    std::cout << "Done Octree search! ( " << (double)(end - start) / CLOCKS_PER_SEC << " [sec] )\n";
} // End exec()

void calcPointDensity::calcMinMax() {
    // Get min and max point density
    m_min_point_density = *std::min_element( m_point_densities.begin(), m_point_densities.end() );
    m_max_point_density = *std::max_element( m_point_densities.begin(), m_point_densities.end() );

    std::cout << "\n";
    std::cout << "Point density statistics:\n";
    std::cout << " Min: " << m_min_point_density << "\n";
    std::cout << " Max: " << m_max_point_density << "\n";
} // End calcMinMax()

void calcPointDensity::removeOutlier( const int _sigma_section4outlier ) {
    // Calc. average and standard deviation for point density
    double avg = 0.0, var = 0.0, std = 0.0;
    for ( const double &i : m_point_densities ) {
        avg += i;
        var += i * i;
    }
    avg = avg / m_point_densities.size();
    var = var / m_point_densities.size() - avg * avg;
    std = sqrt(var);
    std::cout << " Avg: " << avg << "\n";
    std::cout << " Std: " << std << "\n";

    // Remove outlier
    if ( _sigma_section4outlier != 0 ) {
        int threshold4outlier;

        // Define a threshold for outliers
        if ( _sigma_section4outlier == 1 )
            threshold4outlier = avg + 1 * std;
        else if ( _sigma_section4outlier == 2 )
            threshold4outlier = avg + 2 * std;
        else if ( _sigma_section4outlier == 3 )
            threshold4outlier = avg + 3 * std;
        // end if

        // Remove outliers
        for ( size_t i = 0; i < m_point_densities.size(); i++ ) {
            if ( m_point_densities[i] > threshold4outlier )
                m_point_densities[i] = threshold4outlier;
        } // end for

        // Update max point density
        m_max_point_density = threshold4outlier;
        std::cout << "\n";
        std::cout << "Outliers were removed:\n";
        std::cout << " Threshold: " << threshold4outlier << "\n";
        
        // std::cout << "New max point density: " << m_max_point_num << "\n";
    }// end if

} // End removeOutlier()

void calcPointDensity::normalize() {
    // Normalize point densities
    for ( size_t i = 0; i < m_point_densities.size(); i++ )
        m_point_densities[i] /= m_max_point_density;

    std::cout << "\n";
    std::cout << "Point densities were normalized:" << "\n";
    m_min_point_density = *std::min_element( m_point_densities.begin(), m_point_densities.end() );
    m_max_point_density = *std::max_element( m_point_densities.begin(), m_point_densities.end() );
    std::cout << " Min: " << m_min_point_density << "\n";
    std::cout << " Max: " << m_max_point_density << "\n";
} // End normalize()