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

#define ADJUST_POINT_DENSITIES_MODE

const char OUT_FILE[] = "SPBR_DATA/output_vpd.spbr";

void message() {
    std::cout << "================================="    << std::endl;
    std::cout << "     Visualize Point Density "        << std::endl;
    std::cout << "         Tomomasa Uchida "            << std::endl;
    std::cout << "           2020/08/20 "               << std::endl;
    std::cout << "================================="    << std::endl;
    std::cout << std::endl;
}

int main( int argc, char** argv ) {
    char outSPBRfile[128];
    strcpy( outSPBRfile, OUT_FILE ); 
    message();

    if ( argc != 3 ) {
        std::cerr << "USAGE: $ ./vpd [input.spbr] [output.spbr]" << std::endl;
        exit(1);
    } else {
        strcpy( outSPBRfile, argv[2] );
    }
    
    ImportPointClouds *ply = new ImportPointClouds( argv[1] );
    ply->updateMinMaxCoords();
    std::cout << "\nPLY Min, Max Coords:" << "\n";
    std::cout << "Min : " << ply->minObjectCoord() << "\n";
    std::cout << "Max : " << ply->maxObjectCoord() << "\n";
    std::cout << "Number of points: " << ply->numberOfVertices() << "\n";

    // Calculate Point Dencity
    calcPointDensity *cpd = new calcPointDensity( /* kvs::PolygonObject* */ ply );
    //============================//
    // STEP 1: Select search type //
    //============================//
    int search_type = -1;
    std::cout << "\nSelect search type. ";
    std::cout << "(0: RadiusSearch or 1: NearestKSearch): ";
    std::cin >> search_type;
    if ( search_type == 0 ) {
        cpd->setSearchType( calcPointDensity::RadiusSearch );
        std::cout << "> RadiusSearch" << std::endl;

        //====================//
        // STEP 2: Set radius //
        //====================//
        int divide = 0;
        kvs::Vector3f bb_dia_vec = ply->maxObjectCoord() - ply->minObjectCoord();  // Diagonal vector
        std::cout << "\ndiagonal_length = " << bb_dia_vec.length() << std::endl;
        std::cout << "Set divide value. ";
        std::cout << "(search_radius = diagonal_length / divide_value): ";
        std::cin >> divide;
        cpd->setSearchRadius( divide, ply->minObjectCoord(), ply->maxObjectCoord() );
        //std::cout << "> " << divide << std::endl;

    } else if ( search_type == 1 ) {
        cpd->setSearchType( calcPointDensity::NearestKSearch );
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

    cpd->calc( ply );
    cpd->calcMaxMin4PointDensities();
#ifdef ADJUST_POINT_DENSITIES_MODE
    cpd->adjustPointDensities();
#endif
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

    // Write to SPBR file
    WritingDataType type = Ascii;
    writeSPBR(  ply,          /* kvs::PolygonObject *_ply        */  
                outSPBRfile,  /* char*              _filename    */  
                type          /* WritingDataType    _type        */
                );       

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