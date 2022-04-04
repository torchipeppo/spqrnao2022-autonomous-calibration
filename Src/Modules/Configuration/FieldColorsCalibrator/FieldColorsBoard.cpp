#include "FieldColorsBoard.h"

unsigned cognition_to_lower = 0;
unsigned lower_to_cognition = 0;

namespace FieldColorsBoard {

int get_cognition_to_lower() {
    return cognition_to_lower;
}

void set_cognition_to_lower(int msg) {
    cognition_to_lower = msg;
}

void del_cognition_to_lower() {
    set_cognition_to_lower(0);
}

int get_lower_to_cognition() {
    return lower_to_cognition;
}

void set_lower_to_cognition(int msg) {
    lower_to_cognition = msg;
}

void del_lower_to_cognition() {
    set_lower_to_cognition(0);
}

}
