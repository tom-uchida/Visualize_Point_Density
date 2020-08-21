// ==================================
//      Visualize Point Density
// ==================================

#include <iostream>
#include <cstring> 
#include <cstdlib>
#include <vector>
#include "importPointClouds.h"
#include "calcPointDensity.h"
#include "colormap_option.h"
#include "writeSPBR.h"

#include <kvs/Application> 
#include <kvs/Screen>
#include <kvs/Camera>
#include <kvs/PolygonObject>
#include <kvs/PointObject>
#include <kvs/PointRenderer> 
#include <kvs/Coordinate> 
#include <kvs/ColorMap>

#define OCTREE_MODE
// #define PCL_MODE

const char OUT_FILE[] = "SPBR_DATA/output_vpd.spbr";

void message() {
    std::cout << "=================================" << std::endl;
    std::cout << "     Visualize Point Density"      << std::endl;
    std::cout << "         Tomomasa Uchida"          << std::endl;
    std::cout << "           2020/08/21"             << std::endl;
    std::cout << "=================================" << std::endl;
    std::cout << "\n";
}

int main( int argc, char** argv ) {
    char outSPBRfile[128];
    strcpy( outSPBRfile, OUT_FILE ); 
    message();

    if ( argc != 5 ) {
        std::cout   << "  USAGE:\n  ";
        std::cout   << argv[0] << " [input.spbr] [output.spbr] [sigma_section_for_outlier] [colormap_option]";
        std::cout   << "\n\n  EXAMPLE:\n  ";
        std::cout   << argv[0] << " input.ply output.spbr 2 -v"
                    << "\n\n"
                    << "   [sigma_section_for_outlier]\n"
                    << "    0: No Outlier\n"
                    << "    1: 1σ < Outlier\n"
                    << "    2: 2σ < Outlier\n"
                    << "    3: 3σ < Outlier\n\n"
                    << "   [colormap_option]\n"
                    << "    -v: Viridis\n"
                    << "    -p: Plasma\n"
                    << "    -i: Inferno\n"
                    << "    -m: Magma\n"
                    << "    -c: Cividis\n";
        exit(1);
    } else {
        strcpy( outSPBRfile, argv[2] );
    }
    
    ImportPointClouds *ply = new ImportPointClouds( argv[1] );
    ply->updateMinMaxCoords();
    kvs::Vector3f bb_min = ply->minObjectCoord();
    kvs::Vector3f bb_max = ply->maxObjectCoord();
    kvs::Vector3f bb_dia_vec = bb_min - bb_max;
    std::cout << "\n";
    std::cout << "PLY Min, Max Coords:\n";
    std::cout << "Min : " << bb_min << "\n";
    std::cout << "Max : " << bb_max << "\n";
    std::cout << "Number of points: " << ply->numberOfVertices() << "\n";
    std::cout << "Diagonal length of BB: " << bb_dia_vec.length() << "\n\n";

    // Start Calculate Point Density
    calcPointDensity *cpd = new calcPointDensity();

    // Set colormap type
    kvs::ColorMap cmap;
    for ( size_t i = 1; i < argc; i++ ) {
        if ( !strncmp( VIRIDIS_OPTION, argv[i], strlen( VIRIDIS_OPTION ) ) ) {
            cmap = kvs::ColorMap::Viridis( 256 );
            std::cout << "ColorMap: Viridis\n"; i++;
        } else if ( !strncmp( PLASMA_OPTION,    argv[i],    strlen( PLASMA_OPTION ) ) ) {
            cmap = kvs::ColorMap::Plasma( 256 );
            std::cout << "ColorMap: Plasma\n";  i++;
        } else if ( !strncmp( INFERNO_OPTION,   argv[i],    strlen( INFERNO_OPTION ) ) ) {
            cmap = kvs::ColorMap::Inferno( 256 );
            std::cout << "ColorMap: Inferno\n"; i++;
        } else if ( !strncmp( MAGMA_OPTION,     argv[i],    strlen( MAGMA_OPTION ) ) ) {
            cmap = kvs::ColorMap::Magma( 256 );
            std::cout << "ColorMap: Magma\n";   i++;
        } else if ( !strncmp( CIVIDIS_OPTION,   argv[i],    strlen( CIVIDIS_OPTION ) ) ) {
            cmap = kvs::ColorMap::Cividis( 256 );
            std::cout << "ColorMap: Cividis\n"; i++;
        } // end if
    } // end for

#ifdef OCTREE_MODE
    cpd->setSearchType( calcPointDensity::Octree );
    
    // Set radius
    int divide = 0;
    std::cout << "\n";
    std::cout << "Set divide value. ";
    std::cout << "(search radius = diagonal length / divide value): ";
    std::cin >> divide;
    cpd->setSearchRadius( divide, bb_min, bb_max );

    cpd->calcWithOctree( ply );
#endif

#ifdef PCL_MODE
    //============================//
    // STEP 1: Select search type //
    //============================//
    int search_type = -1;
    std::cout << "\nSelect search type. ";
    std::cout << "(0: RadiusSearch or 1: NearestKSearch): ";
    std::cin >> search_type;
    if ( search_type == 0 ) {
        cpd->setSearchType( calcPointDensity::PCL_RadiusSearch );
        std::cout << "> RadiusSearch" << std::endl;

        //====================//
        // STEP 2: Set radius //
        //====================//
        int divide = 0;
        std::cout << "\n";
        std::cout << "Set divide value. ";
        std::cout << "(search radius = diagonal length / divide value): ";
        std::cin >> divide;
        cpd->setSearchRadius( divide, ply->minObjectCoord(), ply->maxObjectCoord() );
        //std::cout << "> " << divide << std::endl;

    } else if ( search_type == 1 ) {
        cpd->setSearchType( calcPointDensity::PCL_NearestKSearch );
        std::cout << "> NearestKSearch" << std::endl;

        //=======================//
        // STEP 2: Set nearest K //
        //=======================//
        int K = 0;
        std::cout << "\nSet nearest K: ";
        std::cin >> K;
        cpd->setNearestK(K);
        std::cout << "> " << K << std::endl;

    } else {
        exit(1);
    }

    cpd->calcWithPCL( ply );
#endif

    cpd->calcMinMax4PointDensities();
    cpd->removeOutlier4PointDensities( atoi(argv[3]) );
    cpd->normalizePointDensities();
    const std::vector<double> point_densities = cpd->getPointDensities();
    // End Calculate Point Density

    // Start Apply Colormap
    cmap.setRange( cpd->getMinValue(), cpd->getMaxValue() );
    kvs::ValueArray<kvs::UInt8> colors( ply->numberOfVertices() * 3 );
    for ( size_t i = 0; i < ply->numberOfVertices(); i++ ) {
        const kvs::RGBColor color( cmap.at( point_densities[i] ) );

        colors[ 3 * i + 0 ] = color.r();
        colors[ 3 * i + 1 ] = color.g();
        colors[ 3 * i + 2 ] = color.b();
    }
    ply->setColors( colors );
    // End Apply Color

    // Write to .spbr file
    const WritingDataType type = Ascii;
    writeSPBR(  ply,          /* kvs::PolygonObject *_ply        */  
                outSPBRfile,  /* char*              _filename    */  
                type          /* WritingDataType    _type        */ );       

    // Convert "kvs::PolygonObject" to "kvs::PointObject"
    kvs::PointObject* object = new kvs::PointObject( *ply );
    object->setSize( 1 );
    object->updateMinMaxCoords(); 

    // Exec. SPBR
    const std::string out_noised_spbr( outSPBRfile );
    std::string EXEC("spbr ");
    EXEC += out_noised_spbr;
    system( EXEC.c_str() );

    return 0;
}