/**
 * @file LibPotentialFieldsProvider.cpp
 *
 *
 * @author Emanuele Musumeci (used previous implementation by Vincenzo Suriani for potential fields)
 */
#include "Platform/Nao/SoundPlayer.h"
#include "LibPotentialFieldsProvider.h"
#include <iostream>
#include <algorithm>
#define SQ(x) x*x

MAKE_MODULE(LibPotentialFieldsProvider, behaviorControl);

LibPotentialFieldsProvider::LibPotentialFieldsProvider()
{
    SPQR::ConfigurationParameters();
}

void LibPotentialFieldsProvider::update(LibPotentialFields& libCheck)
{
  reset();
  libCheck.initializePFAllField = [&](float cell_size, float FIELD_BORDER_OFFSET) -> std::vector<NodePF>
  {
    return initializePFAllField(cell_size, FIELD_BORDER_OFFSET);
  };

  libCheck.initializePFAroundPoint = [&](float cell_size, Vector2f field_center, float field_radius, float FIELD_BORDER_OFFSET = 300.f) -> std::vector<NodePF>
  {
    return initializePFAroundPoint(cell_size, field_center, field_radius, FIELD_BORDER_OFFSET);
  };

  libCheck.computeStrikerAttractivePF = [&] (std::vector<NodePF>& potential_field, Vector2f goal, float RO = 1000.f,
                                                    float Kap = 0.1f, float Kbp = 100.f, float Kr = 100.f,
                                                    float TEAMMATE_CO = 500.f, float ETA = 1000.f, float GAMMA = 2.f) -> std::vector<NodePF>
  {
    return computeStrikerAttractivePF(potential_field, goal, RO, Kap, Kbp, Kr, TEAMMATE_CO, ETA, GAMMA);
  };

  libCheck.computeStrikerRepulsivePF = [&](std::vector<NodePF>& potential_field, Vector2f source_pos, bool navigateAroundBall = false, float RO = 1000.f, float Kap = 0.1f, float Kbp = 100.f, float Kr = 100.f,
                                                    float TEAMMATE_CO = 500.f, float ETA = 1000.f, float GAMMA = 2.f, float POW = 1000.f) -> std::vector<NodePF>
  {
    return computeStrikerRepulsivePF(potential_field, source_pos, navigateAroundBall, RO, Kap, Kbp, Kr, TEAMMATE_CO, ETA, GAMMA);
  };

  libCheck.computeStrikerRepulsivePFWithCustomObstacles = [&](std::vector<NodePF>& potential_field, Vector2f source_pos, std::vector<Vector2f>& repulsive_obstacles, float RO = 1000.f, float Kap = 0.1f, float Kbp = 100.f, float Kr = 100.f,
                                                    float TEAMMATE_CO = 500.f, float ETA = 1000.f, float GAMMA = 2.f, float POW = 1000.f) -> std::vector<NodePF>
  {
    return computeStrikerRepulsivePFWithCustomObstacles(potential_field, source_pos, repulsive_obstacles, RO, Kap, Kbp, Kr, TEAMMATE_CO, ETA, GAMMA);
  };

  libCheck.computePFAllField = [&](std::vector<NodePF>& potential_field, std::vector<NodePF> attractive_field, std::vector<NodePF> repulsive_field) -> std::vector<NodePF>
  {
    return computePFAllField(potential_field, attractive_field, repulsive_field);
  };

  libCheck.computePFAroundPoint = [&](std::vector<NodePF>& potential_field, std::vector<NodePF> attractive_field, std::vector<NodePF> repulsive_field) -> std::vector<NodePF>
  {
    return computePFAroundPoint(potential_field, attractive_field, repulsive_field);
  };

  libCheck.mapToInterval = [&](float value, float fromIntervalMin, float fromIntervalMax, float toIntervalMin, float toIntervalMax) -> float
  {
    return mapToInterval(value, fromIntervalMin, fromIntervalMax, toIntervalMin, toIntervalMax);
  };
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void LibPotentialFieldsProvider::reset()
{}

float LibPotentialFieldsProvider::distance(float x1, float y1, float x2, float y2) const{

  return sqrt(pow(x1-x2,2)+pow(y1-y2,2));

}

float LibPotentialFieldsProvider::distance(Pose2f p1, Pose2f p2) const{

  return static_cast<float>( std::sqrt( std::pow(p2.translation.x() - p1.translation.x(), 2) +
    std::pow(p2.translation.y() - p1.translation.y(), 2) ) );

}

float LibPotentialFieldsProvider::mapToInterval(float value, float fromIntervalMin, float fromIntervalMax, float toIntervalMin, float toIntervalMax) {
  float fromIntervalSize = fromIntervalMax - fromIntervalMin;
  float toIntervalSize = toIntervalMax - toIntervalMin;
  if(value > fromIntervalMax) return toIntervalMax;
  else if (value<fromIntervalMin) return toIntervalMin;
  else return toIntervalMin + (value - fromIntervalMin) * toIntervalSize / fromIntervalSize;
}

Pose2f LibPotentialFieldsProvider::glob2Rel(float x, float y)
{
    Vector2f result;
    float theta = 0;
    float tempX = x - theRobotPose.translation.x();
    float tempY = y - theRobotPose.translation.y();

    result.x() = (float)(tempX * cos(theRobotPose.rotation) + tempY * sin(theRobotPose.rotation));
    result.y() = (float)(-tempX * sin(theRobotPose.rotation) + tempY * cos(theRobotPose.rotation));

    return Pose2f(theta /*deg*/, result.x(),result.y());
}

Pose2f LibPotentialFieldsProvider::rel2Glob(float x, float y)
{
    Vector2f result;
    float rho = (float)(sqrt((x * x) + (y * y)));

    result.x() = (float)(theRobotPose.translation.x() + (rho * cos(theRobotPose.rotation + atan2(y,x))));
    result.y() = (float)(theRobotPose.translation.y() + (rho * sin(theRobotPose.rotation + atan2(y,x))));

    return Pose2f(result.x(),result.y());
}
float LibPotentialFieldsProvider::radiansToDegree(float x)
{
  return (float)((x*180)/3.14159265358979323846);
}

bool LibPotentialFieldsProvider::isValueBalanced(float currentValue, float target, float bound) {
    float minErr = currentValue - (target - bound);
    float maxErr = currentValue - (target + bound);

    if( std::abs(minErr) < bound*1.2 && std::abs(maxErr) < bound*1.2 )
        return true;
    else
        return false;
}

float LibPotentialFieldsProvider::angleToTarget(float x, float y)
{
    //gets the relative position of the point to go for the robot
    Pose2f relativePosition = glob2Rel(x,y);
    // std::cerr << "y relativa: "<< relativePosition.translation.y() << ", x relativa: "<<relativePosition.translation.x() << std::endl;
    //return radiansToDegree(atan2f(relativePosition.translation.y(), relativePosition.translation.x()));

    return (atan2f(relativePosition.translation.y(), relativePosition.translation.x()));

    //return glob2Rel(x, y).translation.angle();
}

std::vector<NodePF> LibPotentialFieldsProvider::initializePFAroundPoint(float cell_size, Vector2f field_center, float field_radius, float FIELD_BORDER_OFFSET)
{
    //std::cout<<"initializePFAroundPoint";
    std::vector<NodePF> potential_field;
    //init obstacle model
    float d = cell_size;
    for(float i=SPQR::FIELD_DIMENSION_X; i>-SPQR::FIELD_DIMENSION_X; i-= d) //TODO parametrize
    {
        for(float j=SPQR::FIELD_DIMENSION_Y; j>-SPQR::FIELD_DIMENSION_Y; j-= d)
        {
          //Skip cells that are further than the field radius
          if(distance(i-d/2,j-d/2,field_center.x(),field_center.y())>field_radius) continue;

          //Only spawn a field cell if it is inside the field border
            if(i < SPQR::FIELD_DIMENSION_X - FIELD_BORDER_OFFSET &&
               i > -SPQR::FIELD_DIMENSION_X + FIELD_BORDER_OFFSET &&
               j < SPQR::FIELD_DIMENSION_Y - FIELD_BORDER_OFFSET &&
               j > -SPQR::FIELD_DIMENSION_Y + FIELD_BORDER_OFFSET
              )
                potential_field.push_back( NodePF(Vector2f( i-d/2,j-d/2 ), Vector2f(), cell_size) );
            else
            {
              //If it is near or on the field border, spawn it exactly at field border with cell_size -1 (it means that this cell is to be considered off limits)
              if(i==SPQR::FIELD_DIMENSION_X - FIELD_BORDER_OFFSET) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f(), -1) );
              if(i== -SPQR::FIELD_DIMENSION_X + FIELD_BORDER_OFFSET) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f(), -1) );
              if(j== SPQR::FIELD_DIMENSION_Y - FIELD_BORDER_OFFSET) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f(), -1) );
              if(j== -SPQR::FIELD_DIMENSION_Y + FIELD_BORDER_OFFSET) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f(), -1) );
            }

            //SPECIAL CASE: Penalty area PF -> Spawn a normal cell
            //if(i>=SPQR::FIELD_DIMENSION_X-(SPQR::FIELD_DIMENSION_X/50) && j<=800 && j >= -800) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f(), cell_size) );
        }
    }
    return potential_field;
}

std::vector<NodePF> LibPotentialFieldsProvider::initializePFAllField(float cell_size, float FIELD_BORDER_OFFSET)
{
    //std::cout<<"initializePFAllField";
    std::vector<NodePF> potential_field;
    //init obstacle model
    int d = cell_size;
    for(int i=SPQR::FIELD_DIMENSION_X; i>-SPQR::FIELD_DIMENSION_X; i-= d) //TODO parametrize
    {
        for(int j=SPQR::FIELD_DIMENSION_Y; j>-SPQR::FIELD_DIMENSION_Y; j-= d)
        {
            potential_field.push_back( NodePF(Vector2f( i-d/2,j-d/2 ), Vector2f(), cell_size) );
            if(i==SPQR::FIELD_DIMENSION_X - FIELD_BORDER_OFFSET) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f(), cell_size) );
            if(i== -SPQR::FIELD_DIMENSION_X + FIELD_BORDER_OFFSET) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f(), cell_size) );
            if(j== SPQR::FIELD_DIMENSION_Y - FIELD_BORDER_OFFSET) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f(), cell_size) );
            if(j== -SPQR::FIELD_DIMENSION_Y + FIELD_BORDER_OFFSET) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f(), cell_size) );
            if(i>=SPQR::FIELD_DIMENSION_X-(SPQR::FIELD_DIMENSION_X/50) && j<=800 && j >= -800) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f(), cell_size) );
        }
    }
    return potential_field;
}

std::vector<NodePF> LibPotentialFieldsProvider::computeStrikerAttractivePF(std::vector<NodePF>& potential_field, Vector2f goal, float RO, float Kap, float Kbp, float Kr, float TEAMMATE_CO, float ETA, float GAMMA)
{
  //std::cout<<"computeStrikerAttractivePF";
  //Attractive potential field toward the center of the soccer field + Repulsive potential field written as an opposite-attractive component
  std::vector<NodePF> attractive_field(potential_field.size());

  for(unsigned int i=0; i<attractive_field.size(); ++i)
  {
    Vector2f current_cell_pos = potential_field.at(i).position;

    Vector2f tmp_err = goal - current_cell_pos;

    if( tmp_err.norm() > RO)
    {
      //Linear potential field: Kap multiplied by the distance (vector) from the goal
      Vector2f conic_field = Vector2f(Kap * tmp_err.x(), Kap * tmp_err.y());
      attractive_field.at(i) = NodePF(Vector2f(current_cell_pos.x(), current_cell_pos.y()),conic_field, potential_field.at(i).cell_size);
    }
    else
    {
      Vector2f quadratic_field = Vector2f((Kbp/tmp_err.norm()) * tmp_err.x(), (Kbp/tmp_err.norm()) * tmp_err.y());
      attractive_field.at(i) = NodePF(Vector2f(current_cell_pos.x(), current_cell_pos.y()),quadratic_field, potential_field.at(i).cell_size);
    }
  }

  return attractive_field;
}

std::vector<NodePF> LibPotentialFieldsProvider::computeStrikerRepulsivePF(std::vector<NodePF>& potential_field, Vector2f source_pos, bool navigateAroundBall,
                                                                          float RO, float Kap, float Kbp, float Kr, float TEAMMATE_CO, float ETA, float GAMMA, float POW) 
{
  //std::cout<<"computeStrikerRepulsivePF";
  //Vector2f my_pos = theRobotPose.translation;

  //Build a vector out of all obstacles, including OPPONENTS and TEAMMATES, excluding POLES
  std::vector<Vector2f> repulsive_obstacles;
  for(auto obs : theTeamPlayersModel.obstacles)
  {
    /*NOTICE: poles are added statically (a priori) by the vision system
      so the 4 goal poles will always be in the obstacle list*/
    switch(obs.type)
    {
      case Obstacle::Type::goalpost:
      {
        //Don't consider poles as repulsors
        break;
      }
      default:
      {
        repulsive_obstacles.push_back(obs.center);
        //std::cout<<"\nObs: ("<<obs.center.x()<<", "<<obs.center.y()<<")"<<std::endl;
        break;
      }
    }
  }
  

  if(navigateAroundBall)
  {
    Vector2f globalBall = rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
    repulsive_obstacles.push_back(globalBall);
  }

  //Repulsive potential field from other robots and/or regions of the field
  std::vector<NodePF> repulsive_field(potential_field.size());

  for(unsigned int i=0; i<repulsive_field.size(); ++i)
  {
      Vector2f current_cell_pos = potential_field.at(i).position;

      repulsive_field.at(i) = NodePF(Vector2f(current_cell_pos.x(),current_cell_pos.y()), Vector2f(0,0), potential_field.at(i).cell_size);

      for(unsigned int r=0; r<repulsive_obstacles.size(); ++r)
      {

          //if(r == (uint) theRobotInfo.number-1) continue;
          if (repulsive_obstacles.at(r).x() == 0 || repulsive_obstacles.at(r).y() == 0) continue;

          //Vector2f tmp_err = repulsive_obstacles.at(r) - current_cell_pos;
          Vector2f tmp_err = current_cell_pos - source_pos;
          //Vector2f tmp_rep = repulsive_obstacles.at(r) - source_pos;
          Vector2f tmp_rep = repulsive_obstacles.at(r) - current_cell_pos;
          if(tmp_rep.norm() < ETA) //If the cell is further away from the obstacle than the cutoff radius ETA, use linear influence instead of quadratic
          {
              //std::cout<<"\nObs: ("<<repulsive_obstacles.at(r).x()<<", "<<repulsive_obstacles.at(r).y()<<")"<<std::endl;
              //std::cout<<"QUADRATIC"<<std::endl;
              float tmp_eta = tmp_rep.norm();

              repulsive_field.at(i).potential = repulsive_field.at(i).potential + Vector2f( -tmp_rep * (Kr/GAMMA) * pow(POW*(1/tmp_eta)-(1/ETA),GAMMA-1) * (1/tmp_err.norm()));
              /// original else bbody
              /// repulsive_field.at(i)  += (Kr/GAMMA) * pow(1000*(1/tmp_eta)-(1/ETA),GAMMA-1) * (-tmp_rep*(1/tmp_err.translation.absFloat()));

          }
          else
          {
              //std::cout<<"\nObs: ("<<repulsive_obstacles.at(r).x()<<", "<<repulsive_obstacles.at(r).y()<<")"<<std::endl;
              //std::cout<<"LINEAR"<<std::endl;
              //WE USED (0.0, 0.0) AS A POTENTIAL TERM FOR OBSTACLES FURTHER THAN ETA
              //repulsive_field.at(i).potential += Vector2f(0.f, 0.f);

              //NOW WE'RE USING A LINEAR REPULSION FOR OBSTACLES FURTHER THAN ETA
              repulsive_field.at(i).potential = repulsive_field.at(i).potential + tmp_rep;
          }
      }

      //if(navigateAroundBall)
      //{
      //  Vector2f ballRep = rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation - current_cell_pos;
      //  repulsive_field.at(i).potential = repulsive_field.at(i).potential - ballRep;
      //}
  }

  return repulsive_field;
}

std::vector<NodePF> LibPotentialFieldsProvider::computeStrikerRepulsivePFWithCustomObstacles(std::vector<NodePF>& potential_field, Vector2f source_pos, std::vector<Vector2f>& repulsive_obstacles, float RO, float Kap, float Kbp, float Kr, float TEAMMATE_CO, float ETA, float GAMMA, float POW) 
{
  //std::cout<<"computeStrikerRepulsivePF";
  //Vector2f my_pos = theRobotPose.translation;

  //Repulsive potential field from other robots and/or regions of the field
  std::vector<NodePF> repulsive_field(potential_field.size());

  for(unsigned int i=0; i<repulsive_field.size(); ++i)
  {
      Vector2f current_cell_pos = potential_field.at(i).position;

      repulsive_field.at(i) = NodePF(Vector2f(current_cell_pos.x(),current_cell_pos.y()), Vector2f(0,0), potential_field.at(i).cell_size);

      for(unsigned int r=0; r<repulsive_obstacles.size(); ++r)
      {

          //if(r == (uint) theRobotInfo.number-1) continue;
          if (repulsive_obstacles.at(r).x() == 0 || repulsive_obstacles.at(r).y() == 0) continue;

          Vector2f tmp_err = repulsive_obstacles.at(r) - current_cell_pos;
          //Vector2f tmp_err = current_cell_pos - source_pos;
          Vector2f tmp_rep = repulsive_obstacles.at(r) - source_pos;
          //if( (tmp_err).norm() < ETA)
          //{
              //std::cout<<"\nObs: ("<<repulsive_obstacles.at(r).x()<<", "<<repulsive_obstacles.at(r).y()<<")"<<std::endl;
              //std::cout<<"QUADRATIC"<<std::endl;
              float tmp_eta = tmp_rep.norm();

              repulsive_field.at(i).potential = repulsive_field.at(i).potential + Vector2f( -tmp_rep * (Kr/GAMMA) * pow(POW*(1/tmp_eta)-(1/ETA),GAMMA-1) * (1/tmp_err.norm()));
              /// original else bbody
              /// repulsive_field.at(i)  += (Kr/GAMMA) * pow(1000*(1/tmp_eta)-(1/ETA),GAMMA-1) * (-tmp_rep*(1/tmp_err.translation.absFloat()));

          //}
          //else
          //{
              //std::cout<<"\nObs: ("<<repulsive_obstacles.at(r).x()<<", "<<repulsive_obstacles.at(r).y()<<")"<<std::endl;
              //std::cout<<"LINEAR"<<std::endl;
              //WE USED (0.0, 0.0) AS A POTENTIAL TERM FOR OBSTACLES FURTHER THAN ETA
              //repulsive_field.at(i).potential += Vector2f(0.f, 0.f);

              //NOW WE'RE USING A LINEAR REPULSION FOR OBSTACLES FURTHER THAN ETA
              //repulsive_field.at(i).potential = -tmp_rep;
          //}
      }
  }

  return repulsive_field;
}

std::vector<NodePF> LibPotentialFieldsProvider::computePFAllField(std::vector<NodePF>& potential_field, std::vector<NodePF> attractive_field, std::vector<NodePF> repulsive_field)
{
  std::cout<<"computePFAllField";
  //std::vector<NodePF> potential_field = initializePFAllField(cell_size);

  ///////////////////////////////
  //NOTICE: can be parallelized//
  ///////////////////////////////

  for(unsigned int p=0; p<potential_field.size(); ++p)
  {
    potential_field.at(p).potential = (attractive_field.at(p).potential + repulsive_field.at(p).potential);
  }

  //attractive_field.clear();
  //repulsive_field.clear();
  //potential_field.clear();

  return potential_field;
}

std::vector<NodePF> LibPotentialFieldsProvider::computePFAroundPoint(std::vector<NodePF>& potential_field, std::vector<NodePF> attractive_field, std::vector<NodePF> repulsive_field)
{
  //std::cout<<"computePFAroundPoint";
  //std::vector<NodePF> potential_field = initializePFAroundPoint(cell_size, field_center, field_radius, FIELD_BORDER_OFFSET);

  ///////////////////////////////
  //NOTICE: can be parallelized//
  ///////////////////////////////

  for(unsigned int p=0; p<potential_field.size(); ++p)
  {
    potential_field.at(p).potential = (attractive_field.at(p).potential + repulsive_field.at(p).potential);
  }

  //attractive_field.clear();
  //repulsive_field.clear();
  //potential_field.clear();

  return potential_field;
}