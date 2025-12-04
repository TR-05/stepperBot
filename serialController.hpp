#ifndef SERIALCONTROLLER_H
#define SERIALCONTROLLER_H

#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <cstdlib>

using namespace std;

/**
 * @brief Sets the terminal to raw mode for instant key presses (non-canonical, no echo).
 */
void setup_terminal();
string send_gcode(const string& command);
bool check_for_quit();

#endif // SERIALCONTROLLER_H