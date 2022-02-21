/**
 * @file CameraCalibrationMain.h
 *
 * File that enables communication between 
 *
 * @author Amila Sikalo
 */

int curr_state_new = 0;

namespace CameraCalibratorNewMain 
{

    int getCurrentCameraCalibratorState()
    {
        return curr_state_new;
    }

    void setCurrentCameraCalibratorState(int s_)
    {
        curr_state_new = s_;
    }
    
}