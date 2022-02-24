/**
 * @file AutomaticCameraCalibratorNew.h
 *
 * File that implements the automatic camera calibration
 * (includes some code from the rest of the bhuman framework to process head motion requests)
 *
 * @author Amila Sikalo
 */

#pragma once

#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/CameraResolutionRequest.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/RingBufferWithSum.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Module/Module.h"
#include "Tools/Optimization/GaussNewtonOptimizer.h"
#include <algorithm>
#include <functional>

#include "Sample2.h"

MODULE(AutomaticCameraCalibrator2,
{,
  REQUIRES(CameraCalibration),
  REQUIRES(CameraInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(JointAngles),
  REQUIRES(LinesPercept),
  REQUIRES(RobotDimensions),
  REQUIRES(TorsoMatrix),
  REQUIRES(RobotPose),
  REQUIRES(HeadLimits),
  REQUIRES(HeadMotionRequest),
  REQUIRES(RobotCameraMatrix),
  REQUIRES(RobotModel),
  PROVIDES(CameraCalibrationNext),
  REQUIRES(CameraCalibrationNext),
  PROVIDES(CameraResolutionRequest),
  PROVIDES(HeadAngleRequest),
  DEFINES_PARAMETERS(
  {,
    (float)(0.001f) terminationCriterion, /**< The difference of two succesive parameter sets that are taken as a convergation */
    (float)(1000000.f) aboveHorizonError, /**< The error for a sample the error of which cannot be computed regularly */
    (unsigned)(10) numOfFramesToWait, /**< The number of frames to wait between two iterations (necessary to keep the debug connection alive) */
    (unsigned)(5) minSuccessiveConvergations, /**< The number of consecutive iterations that fulfil the termination criterion to converge */
    (float)(1.f) waitForUserTime, /** (in s) */
    (std::vector<float>)({2.1f, 1.0f, 0.1f, -1.0f, -2.1f}) headPans,
    (float)(0.38f) upperCameraHeadTilt,
    (float)(-0.25f) lowerCameraHeadTilt,
    (float)(0.5f) headSpeed, /** speed of headmovement*/
    (float)(0.5f) headMotionWaitTime,  /** time the robot has to wait to change the headposition (in s) */
    (unsigned)(300) minimumSampleDistance,  /** the minimum field distance of samples to each other */
    (float)(5.f) deletionThreshold,
    (unsigned)(100) maxIterations,
  }),
});


class AutomaticCameraCalibrator2 : public AutomaticCameraCalibrator2Base
{

public:
    AutomaticCameraCalibrator2();
    //~AutomaticCameraCalibrator2();

    ENUM(State,
    {,
        Idle,  
        Init, 
        MoveHead, 
        WaitForCamera, 
        WaitForHeadToReachPosition, 
        WaitForAccumulate,
        Accumulate, 
        WaitForUser, 
        WaitForOptimize, 
        Optimize, 
        ManualManipulation, 
    });

    /**
     * This enum is used to translate between the indices of the parameter vector used in the
     * optimizer and their actual meaning.
     */
    ENUM(ParameterTranslation,
    {,
      lowerCameraRollCorrection,
      lowerCameraTiltCorrection,
      //lowerCameraPanCorrection,

      upperCameraRollCorrection,
      upperCameraTiltCorrection,
      //upperCameraPanCorrection,

      bodyRollCorrection,
      bodyTiltCorrection,

      robotPoseXCorrection,
      robotPoseYCorrection,
      robotPoseRotationCorrection,
    });

private:
    using Parameters = GaussNewtonOptimizer<numOfParameterTranslations>::Vector;

    struct Functor2 : public GaussNewtonOptimizer<numOfParameterTranslations>::Functor
    {
      AutomaticCameraCalibrator2& calibrator;

      Functor2(AutomaticCameraCalibrator2& calibrator) : calibrator(calibrator) {};

      /**
       * This method computes the error value for a sample and a parameter vector.
       * @param params The parameter vector for which the error should be evaluated.
       * @param measurement The i-th measurement for which the error should be computed.
       * @return The error.
       */
      float operator()(const Parameters& params, size_t measurement) const override;

      size_t getNumOfMeasurements() const override { return calibrator.samples.size(); };
    };
    Functor2 functor;
    friend struct Functor2;
    const CameraInfo::Camera startCamera = CameraInfo::lower;
    CameraInfo::Camera currentCamera = startCamera;
    Pose2f currentRobotPose; 
    CameraMatrix theCameraMatrix;
    std::vector<Sample2> samples;
    CameraCalibration startingCameraCalibration;
    std::vector<float> firstHeadPans;
    std::vector<float> secondHeadPans;
    float currentHeadPan;
    float currentHeadTilt;
    HeadAngleRequest nextHeadAngleRequest;
    RingBufferWithSum<float, 5> headPositionPanBuffer;
    RingBufferWithSum<float, 5> headPositionTiltBuffer;
    unsigned int waitTill = 0; 
    unsigned accumulationTimestamp;
    bool lastActionWasInsertion = false;
    Vector2i unwantedPoint = Vector2i::Zero();
    CameraInfo::Camera deletionOnCamera = CameraInfo::lower;
    Sample2 lastDeletedSample;
    bool alreadyRevertedDeletion = true;
    Vector2i wantedPoint = Vector2i::Zero();
    CameraInfo::Camera insertionOnCamera = CameraInfo::lower;
    Sample2 lastInsertedSample; 
    bool alreadyRevertedInsertion = true;
    bool insertionValueExistant = false, deletionValueExistant = false;
    std::unique_ptr<GaussNewtonOptimizer<numOfParameterTranslations>> optimizer;
    Parameters optimizationParameters;
    CameraCalibration nextCameraCalibration;
    unsigned successiveConvergations; 
    int framesToWait;
    bool setJointOffsets = false;

    

private:
    void idle();
    void init();
    void moveHead();
    void waitForCamera();
    void waitForUser();
    void waitForHeadToReachPosition();
    void accumulate();

    /**
     * Automatically inverts the BodyRotationCorrection so the user does not have to do it.
     * @param the current cameracalibration
     */
    void invert(const CameraCalibration& cameraCalibration);

    void abort();

    void update(CameraCalibrationNext& cameraCalibrationNext) override;
    void update(CameraResolutionRequest& cameraResolutionRequest) override;
    void update(HeadAngleRequest& headAngleRequest) override; 
    void optimize();
    void listen();
    void deleteSample(Vector2i point, CameraInfo::Camera camera);
    void undo();
    void insertSample(Vector2i point, CameraInfo::Camera camera);

    void processManualControls();
    void draw() const;
    void drawFieldLines() const;
    void drawSamples() const;
    float computeError(const Sample2& sample, const CameraCalibration& cameraCalibration, 
                        const Pose2f& robotPose, bool inImage = true) const;

    float operator()(const Parameters& params, size_t measurement) const;

    AutomaticCameraCalibrator2::Parameters pack(const CameraCalibration& cameraCalibration, const Pose2f& robotPose) const;
    void unpack(const Parameters& params, CameraCalibration& cameraCalibration, Pose2f& robotPose) const;
    bool projectLineOnFieldIntoImage(const Geometry::Line& lineOnField, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Geometry::Line& lineInImage) const;
    std::function<void(void)> current_operation;
    State currentState;
    unsigned currentIteration = 0;



    bool override_head_angle_request;

    // cameracontrolengine stuff //

    Rangea panBounds;

    void update_default(HeadAngleRequest& headAngleRequest);

    void calculatePanTiltAngles(const Vector3f& hip2Target, CameraInfo::Camera camera, Vector2a& panTilt) const;
    void adjustTiltBoundToShoulder(Angle pan, CameraInfo::Camera camera, Rangea& tiltBound) const;




};

