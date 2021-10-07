/**
 * @file Modules/Modeling/NearBallNavigationField/NearBallNavigationFieldProvider.cpp
 *
 * This file implements a module that provides a model of an Artificial Potential
 * Field for the striker to navigate around the ball, avoiding obstacles
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include <iostream>
#include "NearBallNavigationFieldProvider.h"
#include "Platform/Time.h"

NearBallNavigationFieldProvider::NearBallNavigationFieldProvider() :
    last_attractive_field_update(0.f),
    last_repulsive_field_update(0.f),
    last_potential_field_update(0.f)
{}

void NearBallNavigationFieldProvider::update(NearBallNavigationField& nearBallNavigationField)
{
    //std::cout<<"NearBallNavigationFieldProvider"<<std::endl;
    if(theGameInfo.state == STATE_PLAYING && theRole.role == Role::striker)
    {
        nearBallNavigationField.graphical_debug = (GRAPHICAL_DEBUG==1 ? true : false);
        nearBallNavigationField.show_tiles = (SHOW_TILES==1 ? true : false);

        nearBallNavigationField.graphical_norm_factor = GRAPHICAL_NORM_FACTOR;
        nearBallNavigationField.graphical_potential_upper_bound = GRAPHICAL_POTENTIAL_UPPER_BOUND;
        nearBallNavigationField.graphical_potential_lower_bound = GRAPHICAL_POTENTIAL_LOWER_BOUND;
        //nearBallNavigationField.graphical_draw_radius = GRAPHICAL_DRAW_RADIUS;

        nearBallNavigationField.graphical_min_mesh_height = GRAPHICAL_MAX_MESH_HEIGHT;
        nearBallNavigationField.graphical_max_mesh_height = GRAPHICAL_MIN_MESH_HEIGHT;

        nearBallNavigationField.graphical_arrow_length_as_norm = (GRAPHICAL_ARROW_LENGTH_AS_NORM==1 ? true : false);
        nearBallNavigationField.graphical_arrow_length = GRAPHICAL_ARROW_LENGTH;

        nearBallNavigationField.angle_based_color = (ANGLE_BASED_COLOR==1 ? true : false);
        
        nearBallNavigationField.field_border_offset = FIELD_BORDER_OFFSET;

        nearBallNavigationField.max_cell_size = MAXIMUM_CELL_SIZE;
        nearBallNavigationField.min_cell_size = MINIMUM_CELL_SIZE;
        
        nearBallNavigationField.min_field_radius = theBallCarrierModel.minimumApproachDistance;
        nearBallNavigationField.max_field_radius = theBallCarrierModel.maximumApproachDistance;

        nearBallNavigationField.use_dynamic_approach_point = (USE_DYNAMIC_APPROACH_POINT == 1 ? true : false); /** Use the ball carrier dynamic approach point as an attractor, otherwise use the static one */
        
        nearBallNavigationField.RO = RO;
        nearBallNavigationField.Kap = Kap;
        nearBallNavigationField.Kbp = Kbp;
        nearBallNavigationField.Kr = Kr;
        nearBallNavigationField.TEAMMATE_CO = TEAMMATE_CO;
        nearBallNavigationField.ETA = ETA;
        nearBallNavigationField.GAMMA = GAMMA;
        nearBallNavigationField.POW = POW;
        
        float current_time = Time::getCurrentSystemTime(); //in milliseconds

        if(last_potential_field_update == 0.f || current_time - last_potential_field_update > POTENTIAL_FIELD_DELAY)
        {    
            //std::cout<<"Initialize potential field"<<std::endl;

            //nearBallNavigationField.current_field_source = theRobotPose.translation;
            nearBallNavigationField.current_field_source = theBallCarrierModel.staticApproachPoint().translation;
            //std::cout<<"\tnearBallNavigationField.current_field_center: ("<<nearBallNavigationField.current_field_center.x()<<", "<<nearBallNavigationField.current_field_center.y()<<")"<<std::endl;
            
            //Potential field radius is inversely proportional to distance from field borders  
            //float radius_x = theLibCheck.mapToInterval(std::abs(nearBallNavigationField.current_field_center.x()), 0.f, theFieldDimensions.xPosOpponentGroundline, 0.f, nearBallNavigationField.max_field_radius - nearBallNavigationField.min_field_radius);
            //float radius_y = theLibCheck.mapToInterval(std::abs(nearBallNavigationField.current_field_center.y()), 0.f, theFieldDimensions.yPosLeftSideline, 0.f, nearBallNavigationField.max_field_radius - nearBallNavigationField.min_field_radius);
            //std::cout<<"radius_x: "<<radius_x<<std::endl;
            //std::cout<<"radius_y: "<<radius_y<<std::endl;
            
            nearBallNavigationField.current_field_center = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
            nearBallNavigationField.current_field_radius = nearBallNavigationField.max_field_radius;
            //nearBallNavigationField.current_field_radius = nearBallNavigationField.max_field_radius;
            //std::cout<<"\tnearBallNavigationField.current_field_radius: "<<nearBallNavigationField.current_field_radius<<std::endl;

            //Potential field cell size is inversely proportional to distance from field borders
            float cell_size_x = theLibCheck.mapToInterval(std::abs(nearBallNavigationField.current_field_center.x()), 0.f, theFieldDimensions.xPosOpponentGroundline, 0.f, nearBallNavigationField.max_cell_size - nearBallNavigationField.min_cell_size);
            float cell_size_y = theLibCheck.mapToInterval(std::abs(nearBallNavigationField.current_field_center.y()), 0.f, theFieldDimensions.yPosLeftSideline, 0.f, nearBallNavigationField.max_cell_size - nearBallNavigationField.min_cell_size);
            nearBallNavigationField.current_cell_size = nearBallNavigationField.min_cell_size;
            //std::cout<<"\tnearBallNavigationField.current_cell_size: "<<nearBallNavigationField.current_cell_size<<std::endl;
            
            nearBallNavigationField.potential_field = theLibPotentialFields.initializePFAroundPoint(nearBallNavigationField.current_cell_size, nearBallNavigationField.current_field_center, nearBallNavigationField.current_field_radius, nearBallNavigationField.field_border_offset);

            Vector2f approachPoint;
            if(nearBallNavigationField.use_dynamic_approach_point)
            {
                approachPoint = theBallCarrierModel.dynamicApproachPoint().translation;
            }
            else
            {
                approachPoint = theBallCarrierModel.staticApproachPoint().translation;
            }

            //std::cout<<"Compute attractive field"<<std::endl;
            nearBallNavigationField.attractive_field = theLibPotentialFields.computeStrikerAttractivePF(nearBallNavigationField.potential_field, approachPoint, RO, Kap, Kbp, Kr, TEAMMATE_CO, ETA, GAMMA);
            last_attractive_field_update = Time::getCurrentSystemTime();

            //std::cout<<"Compute repulsive field"<<std::endl;
            nearBallNavigationField.repulsive_field = theLibPotentialFields.computeStrikerRepulsivePF(nearBallNavigationField.potential_field, nearBallNavigationField.current_field_source, true, RO, Kap, Kbp, Kr, TEAMMATE_CO, ETA, GAMMA, POW);
            last_repulsive_field_update = Time::getCurrentSystemTime();

            //std::cout<<"Compute potential field"<<std::endl;
            //Potential field using the goalTarget with shootASAP mode set to false
            nearBallNavigationField.potential_field = theLibPotentialFields.computePFAroundPoint(nearBallNavigationField.potential_field, nearBallNavigationField.attractive_field, nearBallNavigationField.repulsive_field);
            last_potential_field_update = Time::getCurrentSystemTime();

//TODO Modify the field such that arrows point out radially when too near to the ball

            /*std::cout<<nearBallNavigationField.potential_field.size()<<std::endl;
            for(const auto node: nearBallNavigationField.potential_field)
            {
                std::cout<<"Node: Position: ("<<node.position.x() <<", "<<node.position.y()<<")\nPotential: ("<<node.potential.x() <<", "<<node.potential.y()<<")"<<std::endl;
            }
            std::cout<<std::endl;*/
        }
    }

}

MAKE_MODULE(NearBallNavigationFieldProvider, modeling)