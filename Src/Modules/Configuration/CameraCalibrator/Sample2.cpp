#include "Sample2.h"

Vector2i& Sample2::getPointInImage()
{
    return pointInImage;
}

Vector2i Sample2::getPointInImage() const
{
    return pointInImage;
}

Vector2f& Sample2::getPointOnField()
{
    return pointOnField;
}

Vector2f Sample2::getPointOnField() const
{
    return pointOnField;
}

TorsoMatrix& Sample2::getTorsoMatrix()
{
    return torsoMatrix;
}

float& Sample2::getHeadYaw()
{
    return headYaw;
}

float& Sample2::getHeadPitch()
{
    return headPitch;
}

TorsoMatrix Sample2::getTorsoMatrix() const
{
    return torsoMatrix;
}

float Sample2::getHeadYaw() const
{
    return headYaw;
}

float Sample2::getHeadPitch() const
{
    return headPitch;
}

void Sample2::setPointInImage(Vector2i& pointInImage_)
{
    pointInImage = pointInImage_;
}

void Sample2::setPointOnField(Vector2f& pointOnField_)
{
    pointOnField = pointOnField_;
}

void Sample2::setTorsoMatrix(const TorsoMatrix torsoMatrix_)
{
    torsoMatrix = torsoMatrix_;
}

void Sample2::setHeadYaw(float yaw_)
{
    headYaw = yaw_;
}

void Sample2::setHeadPitch(float pitch_)
{
    headPitch = pitch_;
}

void Sample2::setCameraInfo(CameraInfo cameraInfo_)
{
    cameraInfo = cameraInfo_;
}