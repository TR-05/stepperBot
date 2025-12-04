#include <iostream>
#include <format>
#include "serialController.hpp"
#include <cstring>
#include <unistd.h>
#include <sys/select.h>

using namespace std;

float deg_to_mm(float deg) {
    const float steps_per_rev = 200.0f; // 360 / 1.8 degrees
    const float marlin_steps_per_mm = 80.0f;       // Your config value
    const float microstepping = 16.0f;     // Assuming 16x microstepping (3200 steps/rev is common for 80 steps/mm)
    float simulated_mm_per_rev = (steps_per_rev * microstepping) / marlin_steps_per_mm;
    float mm = deg * (simulated_mm_per_rev / 360.0f);
    return mm;
}

void blocking_move_deg(float x, float y, float z, float speed_mm_s, float acel_mm_s_2) {
    x = deg_to_mm(x);
    y = deg_to_mm(y);
    z = deg_to_mm(z);

    float speed_mm_m = speed_mm_s * 60.0f;
    std::string move_gcode = std::format("G1 X{:.3f} Y{:.3f} Z{:.3f} F{:.0f}", 
                                         x, y, z, speed_mm_m);
    std::string acel_gcode = std::format("M204 S{:.0f}", acel_mm_s_2);

    send_gcode(acel_gcode);
    send_gcode(move_gcode);
    send_gcode("M400");
    send_gcode("M114");
}

int main() {
    printf("Entered Main...\n");
    setup_terminal();


    bool keep_running = true;
    while(keep_running) {
        blocking_move_deg(0, 90, 360, 200, 100);
        usleep(500000);
        blocking_move_deg(0, 0, 0, 200, 100);
        usleep(500000);
        if (check_for_quit()) {
            cout << "\n'q' pressed. Exiting loop." << endl;
            keep_running = false;
        }
    }
    return 0;
}