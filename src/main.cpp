// ==================================
//      Visualize Point Density
//          Tomomasa Uchida
//            2021/02/06
// ==================================

#include <iostream>
#include <cstring> 
#include <cstdlib>
#include <vector>
#include "import_point_clouds.h"
#include "calc_point_density.h"
#include "colormap_option.h"
#include "write_spbr.h"

#include <kvs/Application> 
#include <kvs/Screen>
#include <kvs/Camera>
#include <kvs/PolygonObject>
#include <kvs/PointObject>
#include <kvs/PointRenderer> 
#include <kvs/Coordinate> 
#include <kvs/ColorMap>

const char OUT_FILE[] = "SPBR_DATA/output_vpd.spbr";

inline void message() {
    std::cout << "\n";
    std::cout << "=================================" << std::endl;
    std::cout << "     Visualize Point Density"      << std::endl;
    std::cout << "         Tomomasa Uchida"          << std::endl;
    std::cout << "           2021/02/06"             << std::endl;
    std::cout << "=================================" << std::endl;
    std::cout << "\n";
}

inline void display_usage( const char* _argv0 ) {
    std::cout   << "  USAGE:\n  "
                << _argv0 
                << " [input_file] [output_file] [max_value_adjustment_ratio] [colormap_type]\n\n"
                << "  EXAMPLE:\n  "
                << _argv0 
                << " input.ply output.spbr 0.8 -v\n\n"
                << "   [max_value_adjustment_ratio]\n"
                << "    0.8: new_max_value = max_value * 0.8\n"
                << "    1.0: no change.\n\n"
                << "   [colormap_type]\n"
                << "    -v: Viridis\n"
                << "    -p: Plasma\n"
                << "    -i: Inferno\n"
                << "    -m: Magma\n"
                << "    -c: Cividis\n\n";
}

int main( int argc, char** argv ) {
    char outSPBRfile[128];
    strcpy( outSPBRfile, OUT_FILE ); 
    message();

    if ( argc != 5 ) {
        display_usage( argv[0] );
        exit(1);
    } else {
        strcpy( outSPBRfile, argv[2] );
    } // end if
    
    // Import the input point cloud
    ImportPointClouds *ply = new ImportPointClouds( argv[1] );
    ply->updateMinMaxCoords();
    const size_t number_of_points = ply->numberOfVertices();
    const kvs::Vector3f BB_min = ply->minObjectCoord();
    const kvs::Vector3f BB_max = ply->maxObjectCoord();
    const kvs::Vector3f BB_dia_vec = BB_min - BB_max;
    std::cout << "\n";
    std::cout << "Bounding Box:\n";
    std::cout << " Min: " << BB_min << "\n";
    std::cout << " Max: " << BB_max << "\n\n";
    std::cout << "Number of points:\n " << number_of_points << "\n\n";

    // Set colormap type
    kvs::ColorMap cmap;
    for ( size_t i = 1; i < argc; i++ ) {
        if ( !strncmp( VIRIDIS_OPTION, argv[i], strlen( VIRIDIS_OPTION ) ) ) {
            cmap = kvs::ColorMap::Viridis( 256 );
            std::cout << "ColorMap:\n Viridis\n\n"; i++;
        } else if ( !strncmp( PLASMA_OPTION,    argv[i], strlen( PLASMA_OPTION ) ) ) {
            cmap = kvs::ColorMap::Plasma( 256 );
            std::cout << "ColorMap:\n Plasma\n\n";  i++;
        } else if ( !strncmp( INFERNO_OPTION,   argv[i], strlen( INFERNO_OPTION ) ) ) {
            cmap = kvs::ColorMap::Inferno( 256 );
            std::cout << "ColorMap:\n Inferno\n\n"; i++;
        } else if ( !strncmp( MAGMA_OPTION,     argv[i], strlen( MAGMA_OPTION ) ) ) {
            cmap = kvs::ColorMap::Magma( 256 );
            std::cout << "ColorMap:\n Magma\n\n";   i++;
        } else if ( !strncmp( CIVIDIS_OPTION,   argv[i], strlen( CIVIDIS_OPTION ) ) ) {
            cmap = kvs::ColorMap::Cividis( 256 );
            std::cout << "ColorMap:\n Cividis\n\n"; i++;
        } // end if
    } // end for
    
    // Set search radius
    int divide_value = 100;
    std::cout << "Diagonal length of BB:\n " << BB_dia_vec.length() << "\n\n";
    std::cout << "Input divide value";
    std::cout << " ( search radius = diagonal length / divide value ): ";
    std::cin  >> divide_value;

    // Calculate point density
    calcPointDensity *cpd = new calcPointDensity();
    cpd->setSearchRadius( divide_value, BB_dia_vec );
    cpd->exec( ply );
    cpd->calcMinMax();
    // cpd->removeOutlier( atoi(argv[3]) );
    cpd->adjustMaxValue( atof( argv[3] ) );
    cpd->normalize();
    const std::vector<double> point_densities = cpd->getPointDensities();

    // Apply colormap
    cmap.setRange( 0.0f, 1.0f );
    kvs::ValueArray<kvs::UInt8> colors( number_of_points * 3 );
    for ( size_t i = 0; i < number_of_points; i++ ) {
        const kvs::RGBColor color( cmap.at( point_densities[i] ) );

        colors[ 3 * i + 0 ] = color.r();
        colors[ 3 * i + 1 ] = color.g();
        colors[ 3 * i + 2 ] = color.b();
    }
    ply->setColors( colors );

    // Write to spbr file
    const WritingDataType type = Ascii;
    writeSPBR(
        ply,          /* kvs::PolygonObject *_ply        */  
        outSPBRfile,  /* char*              _filename    */  
        type          /* WritingDataType    _type        */
    );       

    // Convert "kvs::PolygonObject" to "kvs::PointObject"
    kvs::PointObject* object = new kvs::PointObject( *ply );
    object->setSize( 1 );
    object->updateMinMaxCoords(); 

    // Exec. SPBR
    const std::string output_file( outSPBRfile );
    std::string EXEC( "spbr " );
    EXEC += output_file;
    system( EXEC.c_str() );

    return 0;
}