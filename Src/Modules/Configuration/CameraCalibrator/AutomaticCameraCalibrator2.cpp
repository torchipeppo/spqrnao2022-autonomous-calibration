

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
    #include "Tools/Module/Module.h"
    #include "Tools/Math/Approx.h"
    #include "Tools/Math/Transformation.h"
    #include "Tools/Streams/InStreams.h"
    #include "Tools/Settings.h"
    #include "Platform/File.h"
    #include "Sample2.h"
    #include <limits>
     
    #include <iostream>
    #include <stdio.h>
     
    #include "CameraCalibratorNewMain.h"

    #include "Tools/Motion/InverseKinematic.h"

     
    #pragma GCC diagnostic pop

    // cameracontrolengine stuff //
    #define moveHeadThreshold 0.18
    #define defaultTilt 0.38
     
    MAKE_MODULE(AutomaticCameraCalibrator2, infrastructure)
     
    AutomaticCameraCalibrator2::AutomaticCameraCalibrator2() : functor(*this), currentState(Idle)
    {
        std::cout << "AutomaticCameraCalibrator2 Constructor!\n";
        current_operation = std::bind(&AutomaticCameraCalibrator2::idle, this);
        currentIteration = 0;
        // currentState = State::Idle;
        // cameracontrolengine stuff //
        panBounds.min = theHeadLimits.minPan();
        panBounds.max = theHeadLimits.maxPan();
        override_head_angle_request = false;
    }
     
    void AutomaticCameraCalibrator2::idle()
    {
        // std::cout << "Idling...\n";
        currentState = State(CameraCalibratorNewMain::getCurrentCameraCalibratorState());
        switch (currentState)
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

        override_head_angle_request = true;
     
        startingCameraCalibration = theCameraCalibration;
        std::cout << "Camera Matrix: " << theCameraMatrix.translation << "\t" << theCameraMatrix.rotation << std::endl;
     
        firstHeadPans.clear();
        secondHeadPans.clear();
        firstHeadPans.resize(headPans.size());
        secondHeadPans.resize(headPans.size());
        if (startCamera == CameraInfo::upper)
        {
            std::reverse_copy(headPans.begin(), headPans.end(), firstHeadPans.begin());
            std::copy(headPans.begin(), headPans.end(), secondHeadPans.begin());
        }
        else
        {
            std::copy(headPans.begin(), headPans.end(), firstHeadPans.begin());
            std::reverse_copy(headPans.begin(), headPans.end(), secondHeadPans.begin());
        }
        headPositionPanBuffer.clear();
        headPositionTiltBuffer.clear();
     
        optimizer = nullptr;
     
        successiveConvergations = 0;
        framesToWait = 0;
        samples.clear();
     
        currentState = MoveHead;
        current_operation = std::bind(&AutomaticCameraCalibrator2::moveHead, this);
    }
    void AutomaticCameraCalibrator2::moveHead()
    {
        {
            OUTPUT_TEXT("Moving Head...!\n");
            ASSERT(headSpeed >= 0.3);
            if (firstHeadPans.empty() && secondHeadPans.empty())
            {
                currentHeadPan = 0.0f;
                OUTPUT_TEXT("Accumulation finished. Waiting to optimize... Or manipulate samples...");
                CameraCalibratorNewMain::setCurrentCameraCalibratorState(10);
                currentState = ManualManipulation;
                current_operation = std::bind(&AutomaticCameraCalibrator2::listen, this);
            }
            else if (!firstHeadPans.empty())
            {
                currentHeadPan = firstHeadPans.back();
                firstHeadPans.pop_back();
                currentState = WaitForHeadToReachPosition;
                current_operation = std::bind(&AutomaticCameraCalibrator2::waitForHeadToReachPosition, this);
            }
            else if (firstHeadPans.empty() && secondHeadPans.size() == headPans.size())
            {
                currentCamera = (startCamera == CameraInfo::upper) ? CameraInfo::lower : CameraInfo::upper;
                currentHeadPan = secondHeadPans.back();
                secondHeadPans.pop_back();
                currentState = WaitForCamera;
                current_operation = std::bind(&AutomaticCameraCalibrator2::waitForCamera, this);
            }
            else if (firstHeadPans.empty() && !secondHeadPans.empty())
            {
                currentHeadPan = secondHeadPans.back();
                secondHeadPans.pop_back();
                currentState = WaitForHeadToReachPosition;
                current_operation = std::bind(&AutomaticCameraCalibrator2::waitForHeadToReachPosition, this);
            }
            currentHeadTilt = currentCamera == CameraInfo::upper ? upperCameraHeadTilt : lowerCameraHeadTilt;
     
            nextHeadAngleRequest.pan = currentHeadPan;
            nextHeadAngleRequest.tilt = currentHeadTilt;
            nextHeadAngleRequest.speed = headSpeed;
        }
    }
     
    /**
     * Automatically inverts the BodyRotationCorrection so the user does not have to do it.
     * @param the current cameracalibration
     */
     
    void AutomaticCameraCalibrator2::waitForCamera()
    {
        if (framesToWait == 0)
        {
            framesToWait = 30;
        }
        else
        {
            --framesToWait;
        }
        if (framesToWait == 0)
        {
            currentState = WaitForHeadToReachPosition;
            current_operation = std::bind(&AutomaticCameraCalibrator2::waitForHeadToReachPosition, this);
        }
    }
     
    void AutomaticCameraCalibrator2::waitForHeadToReachPosition()
    {
        headPositionPanBuffer.push_front(theJointAngles.angles[Joints::headYaw]);
        headPositionTiltBuffer.push_front(theJointAngles.angles[Joints::headPitch]);
     
        auto areBuffersFilled = [&]()
        {
            return headPositionPanBuffer.full() && headPositionTiltBuffer.full();
        };
     
        auto headReachedPosition = [&]()
        {
            return Approx::isEqual(headPositionPanBuffer.average(), currentHeadPan, 1e-4f) && Approx::isEqual(headPositionTiltBuffer.average(), currentHeadTilt, 1e-4f);
        };
     
        auto headStoppedMoving = [&]()
        {
            return std::abs(headPositionPanBuffer[0] - (headPositionPanBuffer[1] + headPositionPanBuffer[2]) / 2.f) < 0.0001 && std::abs(headPositionTiltBuffer[0] - (headPositionTiltBuffer[1] + headPositionTiltBuffer[2]) / 2.f) < 0.0001;
        };
     
        if (areBuffersFilled() && (headReachedPosition() || headStoppedMoving()))
        {
            if (waitTill == 0)
                waitTill = Time::getCurrentSystemTime() + static_cast<unsigned>(std::abs(headMotionWaitTime) * 1000);
     
            if (Time::getCurrentSystemTime() >= waitTill)
            {
                headPositionPanBuffer.clear();
                headPositionTiltBuffer.clear();
                waitTill = 0;
                currentState = Accumulate;
                current_operation = std::bind(&AutomaticCameraCalibrator2::accumulate, this);
            }
        }
        // TODO add timeout?
    }
     
    void AutomaticCameraCalibrator2::accumulate()
    {
        if (theCameraInfo.camera != currentCamera)
            return;
     
        auto isInsideImage = [&](const Vector2i &point)
        {
            return point.x() >= 0 && point.x() < theCameraInfo.width && point.y() >= 0 && point.y() < theCameraInfo.height;
        };
     
        std::vector<Sample2> pointsOnALine;
        for (const LinesPercept::Line &line : theLinesPercept.lines)
        {
            for (const Vector2i &point : line.spotsInImg)
            {
                // store all necessary information in the sample
                Vector2f pointOnField;
                if (!isInsideImage(point))
                    continue;
                else if (!Transformation::imageToRobot(point.x(), point.y(), theCameraMatrix, theCameraInfo, pointOnField))
                {
                    OUTPUT_TEXT("MEEEK! Point not on field!" << (theCameraInfo.camera == CameraInfo::upper ? " Upper " : " Lower "));
                    continue;
                }
     
                Sample2 sample;
                sample.setPointInImage(point); // = point;
                sample.setPointOnField(pointOnField);
                sample.setTorsoMatrix(theTorsoMatrix);
                sample.setHeadYaw(theJointAngles.angles[Joints::headYaw]);
                sample.setHeadPitch(theJointAngles.angles[Joints::headPitch]);
                sample.setCameraInfo(theCameraInfo);
     
                bool sufficientDistanceToOtherSamples = true;
                for (const Sample2 &testSample : pointsOnALine)
                {
                    Vector2f difference = sample.getPointOnField() - testSample.getPointOnField();
                    if (testSample.cameraInfo.camera != sample.cameraInfo.camera)
                        continue;
                    if (difference.norm() < minimumSampleDistance)
                        sufficientDistanceToOtherSamples = false;
                }
                if (sufficientDistanceToOtherSamples)
                {
                    samples.push_back(sample);
                    pointsOnALine.push_back(sample);
                }
            }
        }
     
        accumulationTimestamp = Time::getCurrentSystemTime();
        currentState = WaitForUser;
        current_operation = std::bind(&AutomaticCameraCalibrator2::waitForUser, this);
    }
     
    void AutomaticCameraCalibrator2::waitForUser()
    {
        if ((Time::getCurrentSystemTime() - accumulationTimestamp) > waitForUserTime * 1000)
        {
            currentState = MoveHead;
            current_operation = std::bind(&AutomaticCameraCalibrator2::moveHead, this);
        }
    }
     
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
        override_head_angle_request = false;
        currentState = Idle;
        current_operation = std::bind(&AutomaticCameraCalibrator2::idle, this);
    }
     
    void AutomaticCameraCalibrator2::update(CameraCalibrationNext &cameraCalibrationNext)
    {
        const RobotCameraMatrix robotCameraMatrix(theRobotDimensions,
                                                  theJointAngles.angles[Joints::headYaw],
                                                  theJointAngles.angles[Joints::headPitch],
                                                  theCameraCalibration,
                                                  theCameraInfo.camera);
        theCameraMatrix.computeCameraMatrix(theTorsoMatrix, robotCameraMatrix, theCameraCalibration);
     
        nextCameraCalibration = theCameraCalibration;
        MODIFY_ONCE("module:AutomaticCameraCalibrator2:robotPose", currentRobotPose);
     
        // Allow access to variables by other modules, required for the AutomaticCameraCalibratorHandler
        MODIFY("module:AutomaticCameraCalibrator2:deletionPoint", unwantedPoint);
        MODIFY("module:AutomaticCameraCalibrator2:deletionCurrentCamera", deletionOnCamera);
        MODIFY("module:AutomaticCameraCalibrator2:insertionPoint", wantedPoint);
        MODIFY("module:AutomaticCameraCalibrator2:insertionCurrentCamera", insertionOnCamera);
        MODIFY("module:AutomaticCameraCalibrator2:setJointOffsets", setJointOffsets);
     
        processManualControls();
        current_operation();
        draw();
     
        // std::cout << "And the state is " << (int)currentState << std::endl;
     
        cameraCalibrationNext.setNext(nextCameraCalibration);
    }
     
    void AutomaticCameraCalibrator2::update(CameraResolutionRequest &cameraResolutionRequest)
    {
        if (SystemCall::getMode() == SystemCall::Mode::physicalRobot)
            cameraResolutionRequest.resolutions[CameraInfo::lower] = (currentState == Idle) ? CameraResolutionRequest::Resolutions::w320h240
                                                                                            : CameraResolutionRequest::Resolutions::w640h480;
    }
     
    void AutomaticCameraCalibrator2::update(HeadAngleRequest &headAngleRequest)
    {
        if (override_head_angle_request) {
            if (currentState != Init && currentState != Idle && currentState != MoveHead)
                headAngleRequest = nextHeadAngleRequest;
        }
        else {
            update_default(headAngleRequest);
        }
    }
     
    void AutomaticCameraCalibrator2::optimize()
    {
        OUTPUT_TEXT("Optimizing...");
        
        if (!optimizer)
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
            if (framesToWait <= 0)
            {
                currentIteration++;
                framesToWait = numOfFramesToWait;
                const float delta = optimizer->iterate(optimizationParameters, Parameters::Constant(0.0001f));
                if (!std::isfinite(delta))
                {
                    OUTPUT_TEXT("Restart optimize! An optimization error occured!");
                    optimizer = nullptr;
                    currentState = Accumulate;
                    current_operation = std::bind(&AutomaticCameraCalibrator2::accumulate, this);
                }
                OUTPUT_TEXT("AutomaticCameraCalibrator: delta = " << delta);
     
                // the camera calibration is refreshed from the current optimizer state
                Pose2f robotPoseCorrection;
                unpack(optimizationParameters, nextCameraCalibration, robotPoseCorrection);

                if (std::abs(delta) < terminationCriterion)
                    ++successiveConvergations;
                else
                    successiveConvergations = 0;
                if (successiveConvergations >= minSuccessiveConvergations ||
                    currentIteration == maxIterations)
                {
                    std::string name = "cameraCalibration.cfg";
                    for(std::string& fullName : File::getFullNames(name))
                    {
                        File path(fullName, "r", false);
                        if(path.exists())
                        {
                            name = std::move(fullName);
                            break;
                        }
                    }
                    std::cout << "Saving to" << name << std::endl;
                    OUTPUT_TEXT("Saving to " + name);
                    OutMapFile stream(name, false);
                    ASSERT(stream.exists());
                    stream << theCameraCalibration;
                    // CameraCalibrationMain::setCurrentStateCalib(0);
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
     
                    if (setJointOffsets)
                        invert(theCameraCalibration);
                }
            }
            --framesToWait;
        }
    }
     
    void AutomaticCameraCalibrator2::listen()
    {
        OUTPUT_TEXT("Listening...");
        if (insertionValueExistant)
            insertSample(wantedPoint, insertionOnCamera);
        if (deletionValueExistant)
            deleteSample(unwantedPoint, deletionOnCamera);
        if (CameraCalibratorNewMain::getCurrentCameraCalibratorState() == 9)
        {
            currentState = Optimize;
            current_operation = std::bind(&AutomaticCameraCalibrator2::optimize, this);
        }
    }
     
    void AutomaticCameraCalibrator2::deleteSample(Vector2i point, CameraInfo::Camera camera)
    {
        if (deletionOnCamera != theCameraInfo.camera || samples.empty())
            return;
        Vector2f interest = point.cast<float>();
        Vector2f pointOnField; // Needed for identification of the sample
        bool x = Transformation::imageToRobot(point.x(), point.y(), theCameraMatrix, theCameraInfo, pointOnField);
        if (!x) // Suppress warnings
            return;
        for (auto existingSample = samples.begin(); existingSample != samples.end();)
        {
            if (existingSample->cameraInfo.camera == camera)
            {
                Vector2f pointInImage;
                const RobotCameraMatrix robotCameraMatrix(
                    theRobotDimensions, theJointAngles.angles[Joints::headYaw],
                    theJointAngles.angles[Joints::headPitch],
                    startingCameraCalibration, theCameraInfo.camera);
                const CameraMatrix cameraMatrix(theTorsoMatrix, robotCameraMatrix, startingCameraCalibration);
                if (Transformation::robotToImage(existingSample->getPointOnField(), cameraMatrix, theCameraInfo, pointInImage))
                {
                    float distance = (interest - pointInImage).norm();
                    if (distance <= deletionThreshold)
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
        if (!lastActionWasInsertion)
        {
            if (!alreadyRevertedDeletion)
            {
                alreadyRevertedDeletion = true;
                samples.push_back(lastDeletedSample);
            }
        }
        else
        {
            if (!alreadyRevertedInsertion)
            {
                int counter = 0;
                bool check = false;
                for (const Sample2 &sample : samples)
                {
                    if (sample.getPointInImage().x() == lastInsertedSample.getPointInImage().x() && sample.getPointInImage().y() == lastInsertedSample.getPointInImage().y())
                    {
                        check = true;
                        break;
                    }
                    ++counter;
                }
                if (check)
                {
                    alreadyRevertedInsertion = true;
                    samples.erase(samples.begin() + counter);
                }
            }
        }
    }
     
    void AutomaticCameraCalibrator2::insertSample(Vector2i point, CameraInfo::Camera camera)
    {
        if (insertionOnCamera != theCameraInfo.camera)
            return;
        Sample2 sample;
        if (!Transformation::imageToRobot(point.x(), point.y(), theCameraMatrix, theCameraInfo, sample.getPointOnField()))
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
        DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator2:accumulate")
        {
            if (currentState == WaitForAccumulate)
            {
                currentState = Accumulate;
                current_operation = std::bind(&AutomaticCameraCalibrator2::accumulate, this);
            }
        }
        DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator2:start")
        {
            if (currentState == Idle || currentState == ManualManipulation)
            {
                currentState = Init;
                current_operation = std::bind(&AutomaticCameraCalibrator2::init, this);
            }
        }
        DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator2:stop")
        {
            abort();
        }
        DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator2:optimize")
        {
            if (currentState == WaitForOptimize || currentState == ManualManipulation || samples.size() > numOfParameterTranslations)
            {
                currentState = Optimize;
                current_operation = std::bind(&AutomaticCameraCalibrator2::optimize, this);
            }
        }
        DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator2:deletePoint")
        {
            deletionValueExistant = true;
        }
        DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator2:undo")
        {
            undo();
        }
        DEBUG_RESPONSE_ONCE("module:AutomaticCameraCalibrator2:insertPoint")
        {
            insertionValueExistant = true;
        }
    }
     
    void AutomaticCameraCalibrator2::draw() const
    {
        DEBUG_DRAWING("module:AutomaticCameraCalibrator2:points", "drawingOnImage")
        {
            THREAD("module:AutomaticCameraCalibrator2:points", theCameraInfo.camera == CameraInfo::upper ? "Upper" : "Lower");
     
            CIRCLE("module:AutomaticCameraCalibrator2:drawSamples", -25, -25, 10, 2, Drawings::solidPen,
                   theCameraInfo.camera == CameraInfo::upper ? ColorRGBA::blue : ColorRGBA::red,
                   Drawings::solidBrush, theCameraInfo.camera == CameraInfo::upper ? ColorRGBA::blue : ColorRGBA::red);
            DRAWTEXT("module:AutomaticCameraCalibrator2:points", 10, -10, 40,
                     !(samples.size() > numOfParameterTranslations) ? ColorRGBA::red : ColorRGBA::green,
                     "Points collected: " << static_cast<unsigned>(samples.size()));
        }
     
        DEBUG_DRAWING("module:AutomaticCameraCalibrator2:drawFieldLines", "drawingOnImage")
        drawFieldLines();
        DEBUG_DRAWING("module:AutomaticCameraCalibrator2:drawSamples", "drawingOnImage")
        drawSamples();
    }
     
    void AutomaticCameraCalibrator2::drawFieldLines() const
    {
        THREAD("module:AutomaticCameraCalibrator2:drawFieldLines", theCameraInfo.camera == CameraInfo::upper ? "Upper" : "Lower");
     
        const Pose2f robotPoseInv = currentRobotPose.inverse();
        for (FieldDimensions::LinesTable::Line lineOnField : theFieldDimensions.fieldLines.lines)
        {
            lineOnField.from = robotPoseInv * lineOnField.from;
            lineOnField.to = robotPoseInv * lineOnField.to;
            Geometry::Line lineInImage;
            if (projectLineOnFieldIntoImage(Geometry::Line(lineOnField.from, lineOnField.to - lineOnField.from),
                                            theCameraMatrix, theCameraInfo, lineInImage))
            {
                LINE("module:AutomaticCameraCalibrator2:drawFieldLines", lineInImage.base.x(), lineInImage.base.y(),
                     (lineInImage.base + lineInImage.direction).x(), (lineInImage.base + lineInImage.direction).y(), 1,
                     Drawings::solidPen, ColorRGBA::black);
            }
        }
    }
     
    void AutomaticCameraCalibrator2::drawSamples() const
    {
        THREAD("module:AutomaticCameraCalibrator2:drawSamples", theCameraInfo.camera == CameraInfo::upper ? "Upper" : "Lower");
     
        const RobotCameraMatrix robotCameraMatrix(
            theRobotDimensions, theJointAngles.angles[Joints::headYaw],
            theJointAngles.angles[Joints::headPitch],
            startingCameraCalibration, theCameraInfo.camera);
        const CameraMatrix cameraMatrix(theTorsoMatrix, robotCameraMatrix, startingCameraCalibration);
        for (const Sample2 &sample : samples)
        {
            ColorRGBA color = sample.cameraInfo.camera == CameraInfo::upper ? ColorRGBA::blue : ColorRGBA::red;
            Vector2f pointInImage;
            if (Transformation::robotToImage(sample.getPointOnField(), cameraMatrix, theCameraInfo, pointInImage))
            {
                CROSS("module:AutomaticCameraCalibrator2:drawSamples",
                      static_cast<int>(pointInImage.x() + 0.5), static_cast<int>(pointInImage.y() + 0.5),
                      5, 1, Drawings::solidPen, color);
            }
        }
    }
     
    float AutomaticCameraCalibrator2::computeError(const Sample2 &sample, const CameraCalibration &cameraCalibration,
                                                   const Pose2f &robotPose, bool inImage) const
    {
        const RobotCameraMatrix robotCameraMatrix(theRobotDimensions, sample.getHeadYaw(), sample.getHeadPitch(),
                                                  cameraCalibration, sample.cameraInfo.camera);
        const CameraMatrix cameraMatrix(sample.getTorsoMatrix(), robotCameraMatrix, cameraCalibration);
     
        float minimum = std::numeric_limits<float>::max();
        if (inImage)
        {
            const Pose2f robotPoseInv = robotPose.inverse();
            for (FieldDimensions::LinesTable::Line lineOnField : theFieldDimensions.fieldLines.lines)
            {
                lineOnField.from = robotPoseInv * lineOnField.from;
                lineOnField.to = robotPoseInv * lineOnField.to;
                Geometry::Line lineInImage;
                float distance;
                if (!projectLineOnFieldIntoImage(Geometry::Line(lineOnField.from, lineOnField.to - lineOnField.from),
                                                 cameraMatrix, sample.cameraInfo, lineInImage))
                    distance = aboveHorizonError;
                else
                    distance = Geometry::getDistanceToEdge(lineInImage, sample.getPointInImage().cast<float>());
     
                if (distance < minimum)
                    minimum = distance;
            }
        }
        else
        {
            Vector3f cameraRay(sample.cameraInfo.focalLength, sample.cameraInfo.opticalCenter.x() - sample.getPointInImage().x(),
                               sample.cameraInfo.opticalCenter.y() - sample.getPointInImage().y());
            cameraRay = cameraMatrix * cameraRay;
            if (cameraRay.z() >= 0)
                return aboveHorizonError;
     
            const float scale = cameraMatrix.translation.z() / -cameraRay.z();
            cameraRay *= scale;
            Vector2f pointOnGround(cameraRay.x(), cameraRay.y());
            pointOnGround = robotPose * pointOnGround;
     
            for (const FieldDimensions::LinesTable::Line &lineOnField : theFieldDimensions.fieldLines.lines)
            {
                const Geometry::Line line(lineOnField.from, lineOnField.to - lineOnField.from);
                const float distance = Geometry::getDistanceToEdge(line, pointOnGround);
                if (distance < minimum)
                    minimum = distance;
            }
        }
        return minimum;
    }
     
    float AutomaticCameraCalibrator2::Functor2::operator()(const Parameters &params, size_t measurement) const
    {
        CameraCalibration cameraCalibration = calibrator.nextCameraCalibration;
        Pose2f robotPose;
        calibrator.unpack(params, cameraCalibration, robotPose);
        robotPose.translation *= 1000.0f;
        robotPose.translation += calibrator.currentRobotPose.translation;
        robotPose.rotation += calibrator.currentRobotPose.rotation;
     
        return calibrator.computeError(calibrator.samples[measurement], cameraCalibration, robotPose);
    }
     
    AutomaticCameraCalibrator2::Parameters AutomaticCameraCalibrator2::pack(const CameraCalibration &cameraCalibration, const Pose2f &robotPose) const
    {
        Parameters params;
        params(lowerCameraRollCorrection) = cameraCalibration.cameraRotationCorrections[CameraInfo::lower].x();
        params(lowerCameraTiltCorrection) = cameraCalibration.cameraRotationCorrections[CameraInfo::lower].y();
     
        params(upperCameraRollCorrection) = cameraCalibration.cameraRotationCorrections[CameraInfo::upper].x();
        params(upperCameraTiltCorrection) = cameraCalibration.cameraRotationCorrections[CameraInfo::upper].y();
     
        params(bodyRollCorrection) = cameraCalibration.bodyRotationCorrection.x();
        params(bodyTiltCorrection) = cameraCalibration.bodyRotationCorrection.y();
     
        params(robotPoseXCorrection) = robotPose.translation.x();
        params(robotPoseYCorrection) = robotPose.translation.y();
        params(robotPoseRotationCorrection) = robotPose.rotation;
        return params;
    }
     
    void AutomaticCameraCalibrator2::unpack(const Parameters &params, CameraCalibration &cameraCalibration, Pose2f &robotPose) const
    {
        cameraCalibration.cameraRotationCorrections[CameraInfo::lower].x() = params(lowerCameraRollCorrection);
        cameraCalibration.cameraRotationCorrections[CameraInfo::lower].y() = params(lowerCameraTiltCorrection);
        cameraCalibration.cameraRotationCorrections[CameraInfo::upper].x() = params(upperCameraRollCorrection);
        cameraCalibration.cameraRotationCorrections[CameraInfo::upper].y() = params(upperCameraTiltCorrection);
        cameraCalibration.bodyRotationCorrection.x() = params(bodyRollCorrection);
        cameraCalibration.bodyRotationCorrection.y() = params(bodyTiltCorrection);
     
        robotPose.translation.x() = params(robotPoseXCorrection);
        robotPose.translation.y() = params(robotPoseYCorrection);
        robotPose.rotation = params(robotPoseRotationCorrection);
    }
     
    bool AutomaticCameraCalibrator2::projectLineOnFieldIntoImage(const Geometry::Line &lineOnField,
                                                                 const CameraMatrix &cameraMatrix, const CameraInfo &cameraInfo, Geometry::Line &lineInImage) const
    {
        const float &f = cameraInfo.focalLength;
        const Pose3f cameraMatrixInv = cameraMatrix.inverse();
     
        const Vector2f &p1 = lineOnField.base;
        const Vector2f p2 = p1 + lineOnField.direction;
        Vector3f p1Camera(p1.x(), p1.y(), 0);
        Vector3f p2Camera(p2.x(), p2.y(), 0);
     
        p1Camera = cameraMatrixInv * p1Camera;
        p2Camera = cameraMatrixInv * p2Camera;
     
        const bool p1Behind = p1Camera.x() < cameraInfo.focalLength;
        const bool p2Behind = p2Camera.x() < cameraInfo.focalLength;
        if (p1Behind && p2Behind)
            return false;
        else if (!p1Behind && !p2Behind)
        {
            p1Camera /= (p1Camera.x() / f);
            p2Camera /= (p2Camera.x() / f);
        }
        else
        {
            const Vector3f direction = p1Camera - p2Camera;
            const float scale = (f - p1Camera.x()) / direction.x();
            const Vector3f intersection = p1Camera + direction * scale;
            if (p1Behind)
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


    // cameracontrolengine stuff //

    void AutomaticCameraCalibrator2::update_default(HeadAngleRequest& headAngleRequest)
    {
        Vector2a panTiltUpperCam;
        Vector2a panTiltLowerCam;

        if(theHeadMotionRequest.mode == HeadMotionRequest::panTiltMode)
        {
            panTiltUpperCam.x() = theHeadMotionRequest.pan;
            panTiltUpperCam.y() = theHeadMotionRequest.tilt + theRobotDimensions.getTiltNeckToCamera(false);
            panTiltLowerCam.x() = theHeadMotionRequest.pan;
            panTiltLowerCam.y() = theHeadMotionRequest.tilt + theRobotDimensions.getTiltNeckToCamera(true);
        }
        else
        {
            Vector3f hip2Target;
            if(theHeadMotionRequest.mode == HeadMotionRequest::targetMode)
            hip2Target = theHeadMotionRequest.target;
            else
            hip2Target = theTorsoMatrix.inverse() * theHeadMotionRequest.target;

            calculatePanTiltAngles(hip2Target, CameraInfo::upper, panTiltUpperCam);
            calculatePanTiltAngles(hip2Target, CameraInfo::lower, panTiltLowerCam);

            if(theHeadMotionRequest.cameraControlMode == HeadMotionRequest::autoCamera)
            panTiltLowerCam.y() = defaultTilt;
        }

        if(panTiltUpperCam.x() < panBounds.min)
        {
            panTiltUpperCam.x() = panBounds.min;
            panTiltLowerCam.x() = panBounds.min;
        }
        else if(panTiltUpperCam.x() > panBounds.max)
        {
            panTiltUpperCam.x() = panBounds.max;
            panTiltLowerCam.x() = panBounds.max;
        }

        Rangea tiltBoundUpperCam = theHeadLimits.getTiltBound(panTiltUpperCam.x());
        Rangea tiltBoundLowerCam = theHeadLimits.getTiltBound(panTiltLowerCam.x());

        adjustTiltBoundToShoulder(panTiltUpperCam.x(), CameraInfo::upper, tiltBoundUpperCam);
        adjustTiltBoundToShoulder(panTiltLowerCam.x(), CameraInfo::lower, tiltBoundLowerCam);

        bool lowerCam = false;
        headAngleRequest.pan = panTiltUpperCam.x(); // Pan is the same for both cams

        if(theHeadMotionRequest.cameraControlMode == HeadMotionRequest::upperCamera)
        {
            headAngleRequest.tilt = panTiltUpperCam.y();
            lowerCam = false;
        }
        else if(theHeadMotionRequest.cameraControlMode == HeadMotionRequest::lowerCamera)
        {
            headAngleRequest.tilt = panTiltLowerCam.y();
            lowerCam = true;
        }
        else
        {
            if(theHeadMotionRequest.mode != HeadMotionRequest::panTiltMode)
            {
            if(panTiltLowerCam.y() > tiltBoundLowerCam.max)
            {
                headAngleRequest.tilt = panTiltUpperCam.y();
                lowerCam = false;
            }
            else if(panTiltUpperCam.y() < moveHeadThreshold)
            {
                headAngleRequest.tilt = defaultTilt - std::abs(moveHeadThreshold - panTiltUpperCam.y());
                lowerCam = false;
            }
            else
            {
                headAngleRequest.tilt = panTiltLowerCam.y();
                lowerCam = true;
            }
            }
            else
            {
            headAngleRequest.tilt = panTiltUpperCam.y();
            lowerCam = true;
            }
        }

        if(lowerCam)
            tiltBoundLowerCam.clamp(headAngleRequest.tilt);
        else
            tiltBoundUpperCam.clamp(headAngleRequest.tilt);

        if(theHeadMotionRequest.mode == HeadMotionRequest::panTiltMode)
        {
            if(theHeadMotionRequest.tilt == JointAngles::off)
            headAngleRequest.tilt = JointAngles::off;
            if(theHeadMotionRequest.pan == JointAngles::off)
            headAngleRequest.pan = JointAngles::off;
        }
        headAngleRequest.speed = theHeadMotionRequest.speed;
        headAngleRequest.stopAndGoMode = theHeadMotionRequest.stopAndGoMode;
    }

    void AutomaticCameraCalibrator2::calculatePanTiltAngles(const Vector3f& hip2Target, CameraInfo::Camera camera, Vector2a& panTilt) const
    {
        InverseKinematic::calcHeadJoints(hip2Target, pi_2, theRobotDimensions, camera, panTilt, theCameraCalibration);
    }

    void AutomaticCameraCalibrator2::adjustTiltBoundToShoulder(const Angle pan, CameraInfo::Camera camera, Range<Angle>& tiltBound) const
    {
        Limbs::Limb shoulder = pan > 0_deg ? Limbs::shoulderLeft : Limbs::shoulderRight;
        const Vector3f& shoulderVector = theRobotModel.limbs[shoulder].translation;
        RobotCameraMatrix rcm(theRobotDimensions, pan, 0.0f, theCameraCalibration, camera);
        Vector3f intersection = Vector3f::Zero();
        if(theHeadLimits.intersectionWithShoulderEdge(rcm, shoulderVector, intersection))
        {
            Vector2a intersectionPanTilt;
            calculatePanTiltAngles(intersection, camera, intersectionPanTilt);
            if(intersectionPanTilt.y() < tiltBound.max) // if(tilt smaller than upper bound)
            tiltBound.max = intersectionPanTilt.y();
        }
    }


