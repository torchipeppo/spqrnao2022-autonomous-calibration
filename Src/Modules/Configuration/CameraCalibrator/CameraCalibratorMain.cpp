/**
 * @file CameraCalibrationMain.h
 *
 * File that enables communication between 
 *
 * @author Amila Sikalo
 */

int curr_state = 0;

namespace CameraCalibrationMain 
{

    int getCurrentStateCard()
    {
        return curr_state;
    }

    void setCurrentStateCard(int s_)
    {
        curr_state = s_;
    }

    int getCurrentStateCalib()
    {
        return curr_state;
    }

    void setCurrentStateCalib(int s_)
    {
        curr_state = s_;
    }

}