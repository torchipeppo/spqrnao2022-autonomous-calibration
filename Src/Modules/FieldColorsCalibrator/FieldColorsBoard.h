/**
 * A simple shared platform to have FieldColorsCalibrator (in the Lower thread)
 * and ColorCalibrationCard (in the Cognition thread) communicate.
 * 
 * Since the communication is really simple (there's not even need for a queue)
 * and does not need to be used anywhere else in the framework,
 * and thus is entirely under my control,
 * I'll enforce a strict convention to avoid concurrency issues.
 */

#define FLDCOLBRD__NO_MESSAGE 0

#define FLDCOLBRD__C2L_BEGIN_CALIBRATION 1
#define FLDCOLBRD__C2L_BEGIN_PHASE2 2

#define FLDCOLBRD__L2C_CALIBRATION_DONE 1

namespace FieldColorsBoard {

int get_cognition_to_lower();            // Lower uses this.
void set_cognition_to_lower(int msg);    // Cognition uses this.
void del_cognition_to_lower();           // Lower uses this.

int get_lower_to_cognition();            // Cognition uses this.
void set_lower_to_cognition(int msg);    // Lower uses this.
void del_lower_to_cognition();           // Cognition uses this.

}
