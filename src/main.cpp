// ==================================
//      Visualize Point Density
// ==================================

#include <iostream>
#include <cstring> 
#include <cstdlib>
#include <vector>
#include "importPointClouds.h"
#include "writeSPBR.h"
#include "calcPointDensity.h"
#include "tinycolormap.h"

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
    std::cout << "           2020/08/20"             << std::endl;
    std::cout << "=================================" << std::endl;
    std::cout << "\n";
}

int main( int argc, char** argv ) {
    char outSPBRfile[128];
    strcpy( outSPBRfile, OUT_FILE ); 
    message();

    if ( argc != 4 ) {
        std::cout   << "  USAGE:\n  ";
        std::cout   << argv[0] << " [input.spbr] [sigma_section_for_outlier] [output.spbr]";
        std::cout   << "\n\n  EXAMPLE:\n  ";
        std::cout   << argv[0] << " input.ply 2 output.spbr"
                    << "\n\n"
                    << "   [sigma_section_for_outlier]\n"
                    << "    0: No Outlier\n"
                    << "    1: 1σ < Outlier\n"
                    << "    2: 2σ < Outlier\n"
                    << "    3: 3σ < Outlier\n";
        exit(1);
    } else {
        strcpy( outSPBRfile, argv[3] );
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
    std::cout << "Diagonal length of BB: " << bb_dia_vec.length() << "\n";

    // Calculate Point Density
    // calcPointDensity *cpd = new calcPointDensity( /* kvs::PolygonObject* */ ply );
    calcPointDensity *cpd = new calcPointDensity();

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
    cpd->removeOutlier4PointDensities( atoi(argv[2]) );
    cpd->normalizePointDensities();
    const std::vector<double> normalized_point_densities = cpd->getPointDensities();
    // End Calculate Point Density

    // Apply Color
    std::vector<unsigned char>  colors;
    kvs::ValueArray<kvs::UInt8> original_colors = ply->colors();
    for ( size_t i = 0; i < ply->numberOfVertices(); i++ ) {
        // Get viridis color
        tinycolormap::Color color = tinycolormap::GetColor(normalized_point_densities[i], tinycolormap::ColormapType::Viridis);

        colors.push_back( color.r()*255 );
        colors.push_back( color.g()*255 );
        colors.push_back( color.b()*255 );
    }
    ply->setColors( kvs::ValueArray<kvs::UInt8>( colors ) );
    // End Apply Color

    // Write to .spbr file
    WritingDataType type = Ascii;
    writeSPBR(  ply,          /* kvs::PolygonObject *_ply        */  
                outSPBRfile,  /* char*              _filename    */  
                type          /* WritingDataType    _type        */ );       

    // Convert "kvs::PolygonObject" to "kvs::PointObject"
    kvs::PointObject* object = new kvs::PointObject( *ply );
    object->setSize( 1 );
    object->updateMinMaxCoords(); 

    // Exec. SPBR
    std::string out_noised_spbr( outSPBRfile );
    std::string EXEC("spbr ");
    EXEC += out_noised_spbr;
    system( EXEC.c_str() );

    return 0;
}