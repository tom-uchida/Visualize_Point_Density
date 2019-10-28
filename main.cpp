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

#include <kvs/PolygonObject>
#include <kvs/PointObject>
#include <kvs/glut/Application> 
#include <kvs/glut/Screen>
#include <kvs/Camera>
#include <kvs/PointRenderer> 
#include <kvs/Coordinate> 

const char OUT_FILE[] = "SPBR_DATA/output.spbr";

int main( int argc, char** argv ) {
    char outSPBRfile[128];
    strcpy( outSPBRfile, OUT_FILE ); 

    // Message                                                                     
    std::cout << "================================="    << std::endl;
    std::cout << "     Visualize Point Density "        << std::endl;
    std::cout << "         Tomomasa Uchida "            << std::endl;
    std::cout << "           2019/10/28 "               << std::endl;
    std::cout << "================================="    << std::endl;
    std::cout << std::endl;

    if ( argc != 3 ) {
        std::cerr << "USAGE: ./vpd input.spbr output.spbr" << std::endl;
        exit(1);
    } else {
        strcpy( outSPBRfile, argv[2] );
    }
    
    // Import "point cloud data（.ply, argv[1]）" that user selected
    ImportPointClouds *ply = new ImportPointClouds( argv[1] );
    ply->updateMinMaxCoords();
    std::cout << "\nPLY Min, Max Coords:" << std::endl;
    std::cout << "Min : " << ply->minObjectCoord() << std::endl;
    std::cout << "Max : " << ply->maxObjectCoord() << std::endl;
    std::cout << "Number of points : " << ply->numberOfVertices() << std::endl;

    // Calculate Point Dencity
    calcPointDensity *cpd = new calcPointDensity( /* kvs::PolygonObject* */ ply );
    //============================//
    // STEP 1: Select search type //
    //============================//
    int search_type = -1;
    std::cout << "\nSelect search type ";
    std::cout << "( 0: RadiusSearch or 1: NearestKSearch ) : ";
    std::cin >> search_type;
    if ( search_type == 0 ) {
        cpd->setSearchType( calcPointDensity::RadiusSearch );
        std::cout << "> RadiusSearch" << std::endl;

        //====================//
        // STEP 2: Set radius //
        //====================//
        int divide = 0;
        std::cout << "\nSet divide value ";
        std::cout << "( search_radius = diagonal_length / divide_value ) : ";
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
        std::cout << "\nSet nearest K : ";
        std::cin >> K;
        cpd->setNearestK(K);
        std::cout << "> " << K << std::endl;

    } else {
        exit(1);
    }

    cpd->calc( ply );
    cpd->normalizePointDensities();
    const std::vector<float> normalized_point_densities = cpd->getPointDensities();
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