#include "serialController.hpp"

using namespace std;

// Global variables for serial port and terminal settings
int serial_port_fd = -1;
struct termios old_tty;
bool terminal_setup = false;

/**
 * @brief Translates an integer baud rate into the corresponding termios constant.
 * @param baud_rate The integer baud rate (e.g., 115200).
 * @return The speed constant (e.g., B115200), or -1 if invalid.
 */
speed_t get_baud_constant(int baud_rate) {
    switch (baud_rate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        default: return (speed_t)-1;
    }
}

/**
 * @brief Opens and configures the serial port.
 * @param port_path The path to the serial port device (e.g., "/dev/ttyACM0").
 * @param baud_rate The desired baud rate (e.g., 115200).
 * @return true on success, false on failure.
 */
bool open_serial_port(const string& port_path, int baud_rate) {
    // 1. Open the serial port file descriptor
    // O_RDWR: Read/Write access
    // O_NOCTTY: This file descriptor will not become the controlling terminal
    // O_NONBLOCK: Non-blocking mode (use O_SYNC for synchronous writes)
    serial_port_fd = open(port_path.c_str(), O_RDWR | O_NOCTTY);

    if (serial_port_fd < 0) {
        cerr << "Error opening serial port " << port_path << ": " << strerror(errno) << endl;
        return false;
    }

    // 2. Configure the port
    struct termios tty;

    // Get current terminal attributes
    if (tcgetattr(serial_port_fd, &tty) != 0) {
        cerr << "Error getting terminal attributes: " << strerror(errno) << endl;
        close(serial_port_fd);
        serial_port_fd = -1;
        return false;
    }

    // Set Baud Rate
    speed_t speed = get_baud_constant(baud_rate);
    if (speed == (speed_t)-1) {
        cerr << "Error: Invalid baud rate provided: " << baud_rate << endl;
        close(serial_port_fd);
        serial_port_fd = -1;
        return false;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // Control flags (8 bits, no parity, one stop bit, disable hardware flow control)
    tty.c_cflag |= (CLOCAL | CREAD); // Ignore modem control lines, enable receiver
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;      // Disable parity bit
    tty.c_cflag &= ~CSTOPB;      // One stop bit
    tty.c_cflag &= ~CRTSCTS;     // Disable RTS/CTS hardware flow control

    // Local flags (Raw input mode - disable canonical, echo, signal chars)
    tty.c_lflag &= ~ICANON;      // Disable canonical mode (input processed raw byte-by-byte)
    tty.c_lflag &= ~ECHO;        // Disable echo
    tty.c_lflag &= ~ECHOE;       // Disable erasure
    tty.c_lflag &= ~ECHONL;      // Disable new-line echo
    tty.c_lflag &= ~ISIG;        // Disable signal characters (INTR, QUIT, SUSP)

    // Input flags (disable software flow control, disable translation)
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable XON/XOFF flow control
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable input processing

    // Output flags (Raw output)
    tty.c_oflag &= ~OPOST;       // Disable post-processing of output
    tty.c_oflag &= ~ONLCR;       // Do not map NL to CR-NL

    // Setting VMIN (minimum number of characters to read) and VTIME (timeout)
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout (in 0.1s units)
    tty.c_cc[VMIN] = 0;  // Return as soon as a byte is available

    // 3. Apply the settings
    if (tcsetattr(serial_port_fd, TCSANOW, &tty) != 0) {
        cerr << "Error setting terminal attributes: " << strerror(errno) << endl;
        close(serial_port_fd);
        serial_port_fd = -1;
        return false;
    }

    cout << "Successfully opened serial port: " << port_path << " at " << baud_rate << " baud." << endl;
    return true;
}


// Function to read one line at a time from the serial port with a timeout
std::string read_line_blocking(int fd, long timeout_ms = 60000) {
    std::string line = "";
    char ch;
    
    // Convert timeout_ms to seconds for use with time()
    long start_time_s = time(NULL); 
    long timeout_s = timeout_ms / 1000;

    while (time(NULL) - start_time_s < timeout_s) {
        
        // Use select to wait for data with a timeout
        fd_set set;
        struct timeval timeout;
        FD_ZERO(&set);
        FD_SET(fd, &set);
        
        // Wait 100ms for data
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000; 

        if (select(fd + 1, &set, NULL, NULL, &timeout) > 0) {
            // Data is available, try to read one byte
            if (read(fd, &ch, 1) > 0) {
                // 🛑 CRITICAL DEBUGGING STEP: Print the character immediately
                std::cout << ch; 
                std::cout.flush(); // Ensure the character is displayed instantly

                line += ch;
                if (ch == '\n') {
                    return line; // Found a complete line
                }
            }
        } 
        // No need for 'else if' checking timeout here, the main loop handles it.
    }
    
    // If we exit the while loop, it's a timeout
    return ""; 
}

/**
 * @brief Sends a G-code command and reads the response.
 * @param command The G-code command string (e.g., "G28").
 * @return The response string from the device, or an ERROR message.
 */
std::string send_gcode(const std::string& command) {
    if (serial_port_fd < 0) {
        return "ERROR: Serial port not open.";
    }
    
    // ... (Your existing code for writing the command) ...
    std::string final_command = command + '\n';
    std::cout << "\n\n--- Sending: " << command << " ---" << std::endl;

    if (write(serial_port_fd, final_command.c_str(), final_command.length()) < 0) {
        // ... (Error handling) ...
        return "ERROR: Write failed.";
    }

    // Now, wait for the acknowledgement from the printer
    std::string full_response = "";
    
    // The printer MUST respond with "ok" or "error" after processing a command.
    // We will always wait for "ok" or "error" to keep it simple and robust.
    
    while (true) {
        std::string received_line = read_line_blocking(serial_port_fd); 
        
        if (received_line.empty()) {
            std::cerr << "\n\n!!! TIMEOUT WAITING FOR RESPONSE to: " << command << " !!!" << std::endl;
            return "ERROR: Timeout.";
        }
        
        full_response += received_line;
        
        // **Search for the completion signal**
        // Marlin uses "ok" for success. The M114 response includes "ok" too.
        if (received_line.find("ok") != std::string::npos || 
            received_line.find("error") != std::string::npos ||
            received_line.find("M112") != std::string::npos) 
        {
            std::cout << "--- COMMAND COMPLETE: " << command << " ---\n" << std::endl;
            return full_response; // Found the successful response line!
        }
    }
}

/**
 * @brief Closes the serial port file descriptor.
 */
void close_serial_port() {
    if (serial_port_fd >= 0) {
        close(serial_port_fd);
        serial_port_fd = -1;
        cout << "\nSerial port closed." << endl;
    }
}

void restore_terminal();

/**
 * @brief Sets the terminal to raw mode for instant key presses (non-canonical, no echo).
 */
void setup_terminal() {
    if (tcgetattr(STDIN_FILENO, &old_tty) == 0) {
        struct termios new_tty = old_tty;
        // Non-canonical mode, disable echo
        new_tty.c_lflag &= ~(ICANON | ECHO);
        // Set VMIN=0 (minimum bytes to read) and VTIME=0 (no timeout) for non-blocking read
        new_tty.c_cc[VMIN] = 0;
        new_tty.c_cc[VTIME] = 0;

        if (tcsetattr(STDIN_FILENO, TCSANOW, &new_tty) == 0) {
            terminal_setup = true;
        } else {
            cerr << "Warning: Could not set terminal attributes for instant input." << endl;
        }
    } else {
        cerr << "Warning: Could not get terminal attributes for instant input." << endl;
    }

    // Ensure terminal is restored and port is closed on exit, even if an error occurs
    atexit(restore_terminal);
    atexit(close_serial_port);

    // NOTE: You MUST change this to your actual 3D printer's port path.
    // Common ports: /dev/ttyACM0, /dev/ttyUSB0, COM3 (on Windows, though this is a Unix example)
    const string MARLIN_PORT = "/dev/ttyACM0"; 
    const int MARLIN_BAUD = 115200;

    // 2. Open serial port
    if (!open_serial_port(MARLIN_PORT, MARLIN_BAUD)) {
        // close_serial_port and restore_terminal will be called by atexit()
    }

}

/**
 * @brief Restores the terminal to its previous settings.
 */
void restore_terminal() {
    if (terminal_setup) {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tty);
        cout << "Terminal restored." << endl;
    }
}

/**
 * @brief Checks if 'q' was pressed to quit.
 * @return true if 'q' was pressed, false otherwise.
 */
bool check_for_quit() {
    char key_press = 0;
    // Read up to 1 byte from standard input (non-blocking)
    ssize_t bytes_read = read(STDIN_FILENO, &key_press, 1);

    if (bytes_read > 0) {
        if (key_press == 'q' || key_press == 'Q') {
            return true;
        }
    }
    return false;
}
