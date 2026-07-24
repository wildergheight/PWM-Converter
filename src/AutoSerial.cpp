#include "AutoSerial.h"
#include "Config.h"
#include "Globals.h"

static void parseAutoCommand(String cmd) {
    cmd.trim();
    int r_idx = cmd.indexOf('R');
    int l_idx = cmd.indexOf('L');
    int comma_idx = cmd.indexOf(',');

    if (r_idx != -1 && l_idx != -1 && comma_idx != -1) {
        String r_val_str = cmd.substring(r_idx + 2, comma_idx);
        String l_val_str = cmd.substring(l_idx + 2);

        g_auto_right_norm = constrain(r_val_str.toFloat(), -1.0, 1.0);
        g_auto_left_norm = constrain(l_val_str.toFloat(), -1.0, 1.0);
        g_last_auto_command_time = millis();
    }
}

void checkAutoSerial() {
    while (AUTO_SERIAL.available()) {
        char c = AUTO_SERIAL.read();
        if (c == '\n') {
            parseAutoCommand(g_auto_serial_buffer);
            g_auto_serial_buffer = "";
        } else {
            g_auto_serial_buffer += c;
        }
    }
}
