/**
 * @file Kick.cpp
 *
 * This file implements the implementation of the Kick skill.
 *
 * @author Arne Hasselbring
 */

#define USE_DYNAMIC_POINTS
//#undef USE_DYNAMIC_POINTS

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/KickRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Platform/SystemCall.h"
#include "Tools/RobotParts/Limbs.h"

#include <algorithm>
#include <iostream>
#include <string>

SKILL_IMPLEMENTATION(KickImpl,
{,
  IMPLEMENTS(Kick),
  REQUIRES(GetUpEngineOutput),
  REQUIRES(KickEngineOutput),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  REQUIRES(FieldBall),
  MODIFIES(MotionRequest),
});

class KickImpl : public KickImplBase
{
  option(Kick)
  {
    initial_state(launch)
    {
      transition
      {
        if(theMotionInfo.motion == MotionRequest::kick)
          goto execute;
        if(theMotionInfo.motion == MotionRequest::fall)
          goto fallen;
      }
      action
      {
        setRequest(p, true);
      }
    }

    state(execute)
    {
      transition
      {
        if(theKickEngineOutput.isLeavingPossible)
          goto finished;
        if(theMotionInfo.motion == MotionRequest::fall)
          goto fallen;
      }
      action
      {
        setRequest(p, true);
      }
    }

    target_state(finished)
    {
      action
      {
        setRequest(p, false);
      }
    }

    aborted_state(fallen)
    {
      transition
      {
        if(theMotionInfo.motion != MotionRequest::fall && (theMotionInfo.motion != MotionRequest::getUp || theGetUpEngineOutput.isLeavingPossible))
          goto launch;
      }
      action
      {
        theMotionRequest.motion = MotionRequest::getUp;
        theLibCheck.inc(LibCheck::motionRequest);
      }
    }
  }


  void setRequest(const Kick& p, bool requestKick)
  {

    if(requestKick)
      theMotionRequest.motion = MotionRequest::kick;
    else
      theMotionRequest.motion = MotionRequest::stand;
          	
    theMotionRequest.kickRequest.mirror = p.mirror;
    theMotionRequest.kickRequest.armsBackFix = p.armsBackFix;
    theMotionRequest.kickRequest.dynPoints.clear();
    theMotionRequest.kickRequest.kickMotionType = KickRequest::strongKick;

    #ifdef TARGET_ROBOT
     #ifdef USE_DYNAMIC_POINTS     
      float keyPhase=1, keyPhaseDuration=250;

      if(p.length <=6000.f) {
        //std::cout<<"KICK CASE :"<<p.length<<" € [0 , 6000]\t";
        float kick_length = p.length;
        if(kick_length<2000) kick_length = 2000.f; // the NonInWalkKick is used only if the distance is at least 2 meters
        
        // The kick is always strongKick
        theMotionRequest.kickRequest.kickMotionType = KickRequest::strongKick;
        
        /* Interpolation function for keyPhaseDuration: 
         * if 6000 is the target, the duration phase is 250. 
         * if 2000 is the target, the duration phase is 350.
         * if the target belongs to (2000,6000), then it is interpolated (e.g. 4000 implies duration 300)
         * NOTE: TO BE TESTED WITH DIFFERENT DISTANCES IN THE FIELD.
         */  
        keyPhaseDuration = 350.f - ((100.f/4000.f)*(kick_length-2000.f));
        DynPoint dynTemp(Limbs::footRight,keyPhase,Vector3f::Zero(),keyPhaseDuration);
        theMotionRequest.kickRequest.dynPoints.push_back(dynTemp);
      }
      
      // From 6000 onwards, we still use strongKick with 250. Next pushes of the code will update this part.
      else if(p.length >6000 && p.length <7000) {
        //std::cout<<"KICK CASE :"<<"6500\n";
      	theMotionRequest.kickRequest.kickMotionType = KickRequest::strongKick;
      }
      else if(p.length >=7000 && p.length <8000) {
        //std::cout<<"KICK CASE :"<<"7500\n";
      	theMotionRequest.kickRequest.kickMotionType = KickRequest::strongKick;
      }
      else if(p.length >=8000) {
        //std::cout<<"KICK CASE :"<<"GOAL\n";
      	theMotionRequest.kickRequest.kickMotionType = KickRequest::ottoMetriKick;
      }
     #endif
   #endif

   #ifndef TARGET_ROBOT
    #ifdef USE_DYNAMIC_POINTS     

      if(p.length < 1000){
        //std::cout<<"PASS CASE :"<<"1000\n";
      	theMotionRequest.kickRequest.kickMotionType = KickRequest::kick_0_1;
      } 
      else if(p.length >=1000 && p.length <2000) {
        //std::cout<<"PASS CASE :"<<"1500\n";
      	theMotionRequest.kickRequest.kickMotionType = KickRequest::kick_1_2;
      } 
      else if(p.length >=2000.f && p.length <2500) {
        //std::cout<<"PASS CASE :"<<"2500---3000\n";
      	theMotionRequest.kickRequest.kickMotionType = KickRequest::kick_2_25;
      }
      else if(p.length >=2500 && p.length <3000) {
        //std::cout<<"PASS CASE :"<<"2500---3000\n";
      	theMotionRequest.kickRequest.kickMotionType = KickRequest::kick_25_3;
      }
      else if(p.length >=3000 && p.length <4000) {
        //std::cout<<"PASS CASE :"<<"3500\n";
      	theMotionRequest.kickRequest.kickMotionType = KickRequest::kick_3_4;
      }
      else if(p.length >=4000 && p.length <5000) {
        //std::cout<<"PASS CASE :"<<"4500\n";
      	theMotionRequest.kickRequest.kickMotionType = KickRequest::kick_4_5;
      }
      else if(p.length >=5000 && p.length <6000) {
        //std::cout<<"PASS CASE :"<<"5500\n";
      	theMotionRequest.kickRequest.kickMotionType = KickRequest::kick_5_6;
      }
      else if(p.length >6000 && p.length <7000) {
        //std::cout<<"PASS CASE :"<<"6500\n";
      	theMotionRequest.kickRequest.kickMotionType = KickRequest::kick_6_7;
      }
      else if(p.length >=7000) {
        //std::cout<<"PASS OR KICK CASE :"<<">7500\n";
      	theMotionRequest.kickRequest.kickMotionType = KickRequest::kick_7_9;
      }

    #endif
   #endif

    theLibCheck.inc(LibCheck::motionRequest);
  }
};

MAKE_SKILL_IMPLEMENTATION(KickImpl);
