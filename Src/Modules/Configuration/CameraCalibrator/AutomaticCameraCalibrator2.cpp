/**
 * @file AutomaticCameraCalibrator.cpp
 *
 * This file implements a module that can provide an automatic camera calibration.
 *
 * @author Amila Sikalo
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#pragma GCC diagnostic ignored "-Wimplicit-int-float-conversion"
#pragma GCC diagnostic ignored "-Wreorder-ctor"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"

#include "AutomaticCameraCalibrator2.h"
#include "Modules/Configuration/JointCalibrator/JointCalibrator.h"
#include "Platform/Time.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Approx.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Settings.h"
#include "Sample2.h"
#include <limits>

#include <iostream>

#include "CameraCalibratorNewMain.h"

#pragma GCC diagnostic pop

MAKE_MODULE(AutomaticCameraCalibrator2, infrastructure)

AutomaticCameraCalibrator2::AutomaticCameraCalibrator2() : functor(*this), currentState(Idle)
{
    std::cout << "AutomaticCameraCalibrator2 Constructor!\n";
    current_operation = std::bind(&AutomaticCameraCalibrator2::idle, this);
    //currentState = State::Idle;
}

void AutomaticCameraCalibrator2::idle()
{
    std::cout << "Idling...\n";
    currentState = State(CameraCalibratorNewMain::getCurrentCameraCalibratorState());
    switch(currentState)
    {
        case Idle:
            current_operation = std::bind(&AutomaticCameraCalibrator2::idle, this);
        break;

        case Init:
            current_operation = std::bind(&AutomaticCameraCalibrator2::init, this);
        break;

         case MoveHead:
            current_operation = std::bind(&AutomaticCameraCalibrator2::moveHead, this);
        break;
    }
}

void AutomaticCameraCalibrator2::init()
{
    std::cout << "Init\n";

    // ....
    currentState = MoveHead;
    current_operation = std::bind(&AutomaticCameraCalibrator2::moveHead, this);

}
void AutomaticCameraCalibrator2::moveHead()
{
    std::cout << "Moving head...\n";
}

/**
 * Automatically inverts the BodyRotationCorrection so the user does not have to do it.
 * @param the current cameracalibration
 */
void AutomaticCameraCalibrator2::invert(const CameraCalibration &cameraCalibration)
{
  Vector3a buffLowerCameraRotationCorrection = cameraCalibration.cameraRotationCorrections[CameraInfo::lower];
  Vector3a buffUpperCameraRotationCorrection = cameraCalibration.cameraRotationCorrections[CameraInfo::upper];
  Angle buffBodyRotationCorrectionX = cameraCalibration.bodyRotationCorrection.x() * -1;
  Angle buffBodyRotationCorrectionY = cameraCalibration.bodyRotationCorrection.y() * -1;
  CameraCalibration nextCalibration;
  nextCalibration.cameraRotationCorrections[CameraInfo::lower] = buffLowerCameraRotationCorrection;
  nextCalibration.cameraRotationCorrections[CameraInfo::upper] = buffUpperCameraRotationCorrection;
  nextCalibration.bodyRotationCorrection.x() = 0;
  nextCalibration.bodyRotationCorrection.y() = 0;
  nextCameraCalibration = nextCalibration;
  JointCalibrator::setOffsets(buffBodyRotationCorrectionX, buffBodyRotationCorrectionY);
}

void AutomaticCameraCalibrator2::abort()
{
    currentState = Idle;
    current_operation = std::bind(&AutomaticCameraCalibrator2::idle, this);
}

void AutomaticCameraCalibrator2::update(CameraCalibrationNext &cameraCalibrationNext) 
{
  AutomaticCameraCalibrator2::current_operation();
  const RobotCameraMatrix robotCameraMatrix(theRobotDimensions,
      theJointAngles.angles[Joints::headYaw],
      theJointAngles.angles[Joints::headPitch],
      theCameraCalibration,
      theCameraInfo.camera);
  theCameraMatrix.computeCameraMatrix(theTorsoMatrix, robotCameraMatrix, theCameraCalibration);

  nextCameraCalibration = theCameraCalibration;
  MODIFY_ONCE("module:AutomaticCameraCalibrator:robotPose", currentRobotPose);

  //Allow access to variables by other modules, required for the AutomaticCameraCalibratorHandler
  MODIFY("module:AutomaticCameraCalibrator:deletionPoint", unwantedPoint);
  MODIFY("module:AutomaticCameraCalibrator:deletionCurrentCamera", deletionOnCamera);
  MODIFY("module:AutomaticCameraCalibrator:insertionPoint", wantedPoint);
  MODIFY("module:AutomaticCameraCalibrator:insertionCurrentCamera", insertionOnCamera);
  MODIFY("module:AutomaticCameraCalibrator:setJointOffsets", setJointOffsets);

  processManualControls();
  current_operation();
  draw();

  std::cout << "And the state is " << (int) currentState << std::endl;

  cameraCalibrationNext.setNext(nextCameraCalibration);
}


void AutomaticCameraCalibrator2::update(CameraResolutionRequest &cameraResolutionRequest) 
{
  if(SystemCall::getMode() == SystemCall::Mode::physicalRobot)
    cameraResolutionRequest.resolutions[CameraInfo::lower] = (currentState == Idle) ? CameraResolutionRequest::Resolutions::w320h240
                                                                             : CameraResolutionRequest::Resolutions::w640h480;
}

void AutomaticCameraCalibrator2::update(HeadAngleRequest &headAngleRequest) 
{
  if(currentState != Init && currentState != Idle && currentState != MoveHead)
    headAngleRequest = nextHeadAngleRequest;
}

void AutomaticCameraCalibrator2::optimize()
{
  OUTPUT_TEXT("Optimizing...");
  if(!optimizer)
  {
    // since the parameters for the robot pose are correction parameters,
    // an empty RobotPose is used instead of theRobotPose
    optimizationParameters = pack(theCameraCalibration, Pose2f());
    optimizer = std::make_unique<GaussNewtonOptimizer<numOfParameterTranslations>>(functor);
    successiveConvergations = 0;
    framesToWait = 0;
  }
  else
  {
    // only do an iteration after some frames have passed
    if(framesToWait <= 0)
    {
      framesToWait = numOfFramesToWait;
      const float delta = optimizer->iterate(optimizationParameters, Parameters::Constant(0.0001f));
      if(!std::isfinite(delta))
      {
        OUTPUT_TEXT("Restart optimize! An optimization error occured!");
        optimizer = nullptr;
        currentState = Accumulate;
      }
      OUTPUT_TEXT("AutomaticCameraCalibrator: delta = " << delta);

      // the camera calibration is refreshed from the current optimizer state
      Pose2f robotPoseCorrection;
      unpack(optimizationParameters, nextCameraCalibration, robotPoseCorrection);

      if(std::abs(delta) < terminationCriterion)
        ++successiveConvergations;
      else
        successiveConvergations = 0;
      if(successiveConvergations >= minSuccessiveConvergations)
      {
        //CameraCalibrationMain::setCurrentStateCalib(0);
        CameraCalibratorNewMain::setCurrentCameraCalibratorState(0);
        OUTPUT_TEXT("AutomaticCameraCalibrator: converged!");
        OUTPUT_TEXT("RobotPoseCorrection: " << robotPoseCorrection.translation.x() * 1000.0f
                    << " " << robotPoseCorrection.translation.y() * 1000.0f
                    << " " << robotPoseCorrection.rotation.toDegrees() << "deg");
        currentRobotPose.translation.x() += robotPoseCorrection.translation.x() * 1000.0f;
        currentRobotPose.translation.y() += robotPoseCorrection.translation.y() * 1000.0f;
        currentRobotPose.rotation = Angle::normalize(currentRobotPose.rotation + robotPoseCorrection.rotation);
        OUTPUT_TEXT("save representation:CameraCalibration");
        abort();

        if(setJointOffsets)
          invert(theCameraCalibration);
      }
    }
    --framesToWait;
  }
}

void AutomaticCameraCalibrator2::listen()
{
  OUTPUT_TEXT("Listening...");
  if(insertionValueExistant)
    insertSample(wantedPoint, insertionOnCamera);
  if(deletionValueExistant)
    deleteSample(unwantedPoint, deletionOnCamera);
  if(CameraCalibratorNewMain::getCurrentCameraCalibratorState() == 9)
    currentState = Optimize;
}

void AutomaticCameraCalibrator2::deleteSample(Vector2i point, CameraInfo::Camera camera)
{
  if(deletionOnCamera != theCameraInfo.camera || samples.empty())
    return;
  Vector2f interest = point.cast<float>();
  Vector2f pointOnField;//Needed for identification of the sample
  bool x = Transformation::imageToRobot(point.x(), point.y(), theCameraMatrix, theCameraInfo, pointOnField);
  if(!x)//Suppress warnings
    return;
  for(auto existingSample = samples.begin(); existingSample != samples.end();)
  {
    if(existingSample->cameraInfo.camera == camera)
    {
      Vector2f pointInImage;
      const RobotCameraMatrix robotCameraMatrix(
        theRobotDimensions, theJointAngles.angles[Joints::headYaw],
        theJointAngles.angles[Joints::headPitch],
        startingCameraCalibration, theCameraInfo.camera);
      const CameraMatrix cameraMatrix(theTorsoMatrix, robotCameraMatrix, startingCameraCalibration);
      if(Transformation::robotToImage(existingSample->getPointOnField(), cameraMatrix, theCameraInfo, pointInImage))
      {
        float distance = (interest - pointInImage).norm();
        if(distance <= deletionThreshold)
        {
          lastDeletedSample = *existingSample;
          alreadyRevertedDeletion = false;
          lastActionWasInsertion = false;
          existingSample = samples.erase(existingSample);
          deletionValueExistant = false;
          return;
        }
      }
    }
    ++existingSample;
  }
  return;
}

void AutomaticCameraCalibrator2::undo()
{

}

void AutomaticCameraCalibrator2::insertSample(Vector2i point, CameraInfo::Camera camera)
{
  if(insertionOnCamera != theCameraInfo.camera)
    return;
  Sample2 sample;
  if(!Transformation::imageToRobot(point.x(), point.y(), theCameraMatrix, theCameraInfo, sample.getPointOnField()))
  {
    OUTPUT_TEXT("MEEEK! Point not on field!" << (theCameraInfo.camera == CameraInfo::upper ? " Upper " : " Lower "));
    return;
  }
  sample.setPointInImage(point);
  sample.setTorsoMatrix(theTorsoMatrix);
  sample.setHeadYaw(theJointAngles.angles[Joints::headYaw]);
  sample.setHeadPitch(theJointAngles.angles[Joints::headPitch]);
  sample.cameraInfo = theCameraInfo;
  samples.push_back(sample);
  lastInsertedSample = sample;
  lastActionWasInsertion = true;
  alreadyRevertedInsertion = false;
  insertionValueExistant = false;
}

void AutomaticCameraCalibrator2::processManualControls()
{
  DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator:accumulate")
  {
    if(currentState == WaitForAccumulate)
      currentState = Accumulate;
  }
  DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator:start")
  {
    if(currentState == Idle || currentState == ManualManipulation)
      currentState = Init;
  }
  DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator:stop")
  {
    abort();
  }
  DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator:optimize")
  {
    if(currentState == WaitForOptimize || currentState == ManualManipulation || samples.size() > numOfParameterTranslations)
      currentState = Optimize;
  }
  DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator:deletePoint")
  {
    deletionValueExistant = true;
  }
  DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator:undo")
  {
    undo();
  }
  DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator:insertPoint")
  {
    insertionValueExistant = true;
  }
}

void AutomaticCameraCalibrator2::draw() const
{
  DEBUG_DRAWING("module:AutomaticCameraCalibrator:points", "drawingOnImage")
  {
    THREAD("module:AutomaticCameraCalibrator:points", theCameraInfo.camera == CameraInfo::upper ? "Upper" : "Lower");

    // #define CIRCLE(id, center_x, center_y, radius, penWidth, penStyle, penColor, brushStyle, brushColor) static_cast<void>(0)
    CIRCLE("module:AutomaticCameraCalibrator:drawSamples", -25, -25, 10, 2, Drawings::solidPen,
           theCameraInfo.camera == CameraInfo::upper ? ColorRGBA::blue : ColorRGBA::red,
           Drawings::solidBrush, theCameraInfo.camera == CameraInfo::upper ? ColorRGBA::blue : ColorRGBA::red);
    DRAWTEXT("module:AutomaticCameraCalibrator:points", 10, -10, 40,
             !(samples.size() > numOfParameterTranslations) ? ColorRGBA::red : ColorRGBA::green,
             "Points collected: " << static_cast<unsigned>(samples.size()));
  }

  DEBUG_DRAWING("module:AutomaticCameraCalibrator:drawFieldLines", "drawingOnImage") drawFieldLines();
  DEBUG_DRAWING("module:AutomaticCameraCalibrator:drawSamples", "drawingOnImage") drawSamples();
}

void AutomaticCameraCalibrator2::drawFieldLines() const
{
  THREAD("module:AutomaticCameraCalibrator:drawFieldLines", theCameraInfo.camera == CameraInfo::upper ? "Upper" : "Lower");

  const Pose2f robotPoseInv = currentRobotPose.inverse();
  for(FieldDimensions::LinesTable::Line lineOnField : theFieldDimensions.fieldLines.lines)
  {
    lineOnField.from = robotPoseInv * lineOnField.from;
    lineOnField.to = robotPoseInv * lineOnField.to;
    Geometry::Line lineInImage;
    if(projectLineOnFieldIntoImage(Geometry::Line(lineOnField.from, lineOnField.to - lineOnField.from),
                                   theCameraMatrix, theCameraInfo, lineInImage))
    {
      LINE("module:AutomaticCameraCalibrator:drawFieldLines", lineInImage.base.x(), lineInImage.base.y(),
           (lineInImage.base + lineInImage.direction).x(), (lineInImage.base + lineInImage.direction).y(), 1,
           Drawings::solidPen, ColorRGBA::black);
    }
  }
}

void AutomaticCameraCalibrator2::drawSamples() const
{
  THREAD("module:AutomaticCameraCalibrator:drawSamples", theCameraInfo.camera == CameraInfo::upper ? "Upper" : "Lower");

  const RobotCameraMatrix robotCameraMatrix(
    theRobotDimensions, theJointAngles.angles[Joints::headYaw],
    theJointAngles.angles[Joints::headPitch],
    startingCameraCalibration, theCameraInfo.camera);
  const CameraMatrix cameraMatrix(theTorsoMatrix, robotCameraMatrix, startingCameraCalibration);
  for(const Sample2& sample : samples)
  {
    ColorRGBA color = sample.cameraInfo.camera == CameraInfo::upper ? ColorRGBA::blue : ColorRGBA::red;
    Vector2f pointInImage;
    if(Transformation::robotToImage(sample.getPointOnField(), cameraMatrix, theCameraInfo, pointInImage))
    {
      CROSS("module:AutomaticCameraCalibrator:drawSamples",
            static_cast<int>(pointInImage.x() + 0.5), static_cast<int>(pointInImage.y() + 0.5),
            5, 1, Drawings::solidPen, color);
    }
  }
}

float AutomaticCameraCalibrator2::computeError(const Sample2& sample, const CameraCalibration& cameraCalibration,
    const Pose2f& robotPose, bool inImage) const
{
  // build camera matrix from sample and camera calibration
  const RobotCameraMatrix robotCameraMatrix(theRobotDimensions, sample.getHeadYaw(), sample.getHeadPitch(),
      cameraCalibration, sample.cameraInfo.camera);
  const CameraMatrix cameraMatrix(sample.getTorsoMatrix(), robotCameraMatrix, cameraCalibration);

  float minimum = std::numeric_limits<float>::max();
  if(inImage)
  {
    const Pose2f robotPoseInv = robotPose.inverse();
    for(FieldDimensions::LinesTable::Line lineOnField : theFieldDimensions.fieldLines.lines)
    {
      // transform the line in robot relative coordinates
      lineOnField.from = robotPoseInv * lineOnField.from;
      lineOnField.to = robotPoseInv * lineOnField.to;
      Geometry::Line lineInImage;
      float distance;
      if(!projectLineOnFieldIntoImage(Geometry::Line(lineOnField.from, lineOnField.to - lineOnField.from),
                                      cameraMatrix, sample.cameraInfo, lineInImage))
        distance = aboveHorizonError;
      else
        distance = Geometry::getDistanceToEdge(lineInImage, sample.getPointInImage().cast<float>());

      if(distance < minimum)
        minimum = distance;
    }
  }
  else // on ground
  {
    // project point in image onto ground
    Vector3f cameraRay(sample.cameraInfo.focalLength, sample.cameraInfo.opticalCenter.x() - sample.getPointInImage().x(),
                       sample.cameraInfo.opticalCenter.y() - sample.getPointInImage().y());
    cameraRay = cameraMatrix * cameraRay;
    if(cameraRay.z() >= 0) // above horizon
      return aboveHorizonError;

    const float scale = cameraMatrix.translation.z() / -cameraRay.z();
    cameraRay *= scale;
    Vector2f pointOnGround(cameraRay.x(), cameraRay.y()); // point on ground relative to the robot
    pointOnGround = robotPose * pointOnGround; // point on ground in absolute coordinates

    for(const FieldDimensions::LinesTable::Line& lineOnField : theFieldDimensions.fieldLines.lines)
    {
      const Geometry::Line line(lineOnField.from, lineOnField.to - lineOnField.from);
      const float distance = Geometry::getDistanceToEdge(line, pointOnGround);
      if(distance < minimum)
        minimum = distance;
    }
  }
  return minimum;
}

float AutomaticCameraCalibrator2::Functor2::operator()(const Parameters& params, size_t measurement) const
{
    CameraCalibration cameraCalibration = calibrator.nextCameraCalibration;
    Pose2f robotPose;
    calibrator.unpack(params, cameraCalibration, robotPose);

    // the correction parameters for the robot pose are added to theRobotPose
    // in the parameter space the robot pose translation unit is m to keep the order of
    // magnitude similar to the other parameters
    robotPose.translation *= 1000.0f;
    robotPose.translation += calibrator.currentRobotPose.translation;
    robotPose.rotation += calibrator.currentRobotPose.rotation;

    return calibrator.computeError(calibrator.samples[measurement], cameraCalibration, robotPose);

}

AutomaticCameraCalibrator2::Parameters AutomaticCameraCalibrator2::pack(const CameraCalibration& cameraCalibration, const Pose2f& robotPose) const
{
  Parameters params;
  params(lowerCameraRollCorrection) = cameraCalibration.cameraRotationCorrections[CameraInfo::lower].x();
  params(lowerCameraTiltCorrection) = cameraCalibration.cameraRotationCorrections[CameraInfo::lower].y();
  //params(lowerCameraPanCorrection) = cameraCalibration.lowerCameraRotationCorrection.z;

  params(upperCameraRollCorrection) = cameraCalibration.cameraRotationCorrections[CameraInfo::upper].x();
  params(upperCameraTiltCorrection) = cameraCalibration.cameraRotationCorrections[CameraInfo::upper].y();
  //params[upperCameraPanCorrection) = cameraCalibration.upperCameraRotationCorrection.z;

  params(bodyRollCorrection) = cameraCalibration.bodyRotationCorrection.x();
  params(bodyTiltCorrection) = cameraCalibration.bodyRotationCorrection.y();

  params(robotPoseXCorrection) = robotPose.translation.x();
  params(robotPoseYCorrection) = robotPose.translation.y();
  params(robotPoseRotationCorrection) = robotPose.rotation;
  return params;
}

void AutomaticCameraCalibrator2::unpack(const Parameters& params, CameraCalibration& cameraCalibration, Pose2f& robotPose) const
{
    cameraCalibration.cameraRotationCorrections[CameraInfo::lower].x() = params(lowerCameraRollCorrection);
  cameraCalibration.cameraRotationCorrections[CameraInfo::lower].y() = params(lowerCameraTiltCorrection);
  //cameraCalibration.lowerCameraRotationCorrection.z = params(lowerCameraPanCorrection);

  cameraCalibration.cameraRotationCorrections[CameraInfo::upper].x() = params(upperCameraRollCorrection);
  cameraCalibration.cameraRotationCorrections[CameraInfo::upper].y() = params(upperCameraTiltCorrection);
  //cameraCalibration.upperCameraRotationCorrection.z = params(upperCameraPanCorrection);

  cameraCalibration.bodyRotationCorrection.x() = params(bodyRollCorrection);
  cameraCalibration.bodyRotationCorrection.y() = params(bodyTiltCorrection);

  robotPose.translation.x() = params(robotPoseXCorrection);
  robotPose.translation.y() = params(robotPoseYCorrection);
  robotPose.rotation = params(robotPoseRotationCorrection);

}

bool AutomaticCameraCalibrator2::projectLineOnFieldIntoImage(const Geometry::Line& lineOnField,
    const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Geometry::Line& lineInImage) const
{
  const float& f = cameraInfo.focalLength;
  const Pose3f cameraMatrixInv = cameraMatrix.inverse();

  // start and end point of the line
  const Vector2f& p1 = lineOnField.base;
  const Vector2f p2 = p1 + lineOnField.direction;
  Vector3f p1Camera(p1.x(), p1.y(), 0);
  Vector3f p2Camera(p2.x(), p2.y(), 0);

  // points are transformed into camera coordinates
  p1Camera = cameraMatrixInv * p1Camera;
  p2Camera = cameraMatrixInv * p2Camera;

  // handle the case that points can lie behind the camera plane
  const bool p1Behind = p1Camera.x() < cameraInfo.focalLength;
  const bool p2Behind = p2Camera.x() < cameraInfo.focalLength;
  if(p1Behind && p2Behind)
    return false;
  else if(!p1Behind && !p2Behind)
  {
    // both rays can be simply intersected with the image plane
    p1Camera /= (p1Camera.x() / f);
    p2Camera /= (p2Camera.x() / f);
  }
  else
  {
    // if one point lies behind the camera and the other in front,
    // there must be an intersection of the connective line with the image plane
    const Vector3f direction = p1Camera - p2Camera;
    const float scale = (f - p1Camera.x()) / direction.x();
    const Vector3f intersection = p1Camera + direction * scale;
    if(p1Behind)
    {
      p1Camera = intersection;
      p2Camera /= (p2Camera.x() / f);
    }
    else
    {
      p2Camera = intersection;
      p1Camera /= (p1Camera.x() / f);
    }
  }
  const Vector2f p1Result(cameraInfo.opticalCenter.x() - p1Camera.y(), cameraInfo.opticalCenter.y() - p1Camera.z());
  const Vector2f p2Result(cameraInfo.opticalCenter.x() - p2Camera.y(), cameraInfo.opticalCenter.y() - p2Camera.z());
  lineInImage.base = p1Result;
  lineInImage.direction = p2Result - p1Result;
  return true;
}