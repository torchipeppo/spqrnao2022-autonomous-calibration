/**
 * @file Sample2.h
 *
 * File that defines the Sample
 *
 * @author Amila Sikalo
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"

class Sample2
{
private:
    
public:
    Sample2() {};
    ~Sample2() {};

    CameraInfo cameraInfo;
    Vector2i& getPointInImage();
    Vector2i getPointInImage() const;
    Vector2f& getPointOnField();
    Vector2f getPointOnField() const;
    TorsoMatrix& getTorsoMatrix();
    float& getHeadYaw();
    float& getHeadPitch();
    TorsoMatrix getTorsoMatrix() const;
    float getHeadYaw() const;
    float getHeadPitch() const;
    
    void setPointInImage(Vector2i& pointInImage_);
    void setPointOnField(Vector2f& pointOnField_);
    void setTorsoMatrix(const TorsoMatrix torsoMatrix_);
    void setHeadYaw(float yaw_);
    void setHeadPitch(float pitch_);
    void setCameraInfo(CameraInfo cameraInfo_);


private:
    Vector2i pointInImage;
    Vector2f pointOnField;  
    TorsoMatrix torsoMatrix;
    float headYaw;
    float headPitch;
};



