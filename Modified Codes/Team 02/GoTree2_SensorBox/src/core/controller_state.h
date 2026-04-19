#pragma once

// Thread-safe flags owned by `main.ino` that describe the actuator controller state.
// Used by the nav loop + HTTP handlers to gate missions and commands.

bool controller_state_expander_ready();
bool controller_state_dac_ready();
bool controller_state_active();
bool controller_state_test_kit();
bool controller_state_checkup_in_progress();

void controller_state_request_checkup();
void controller_state_request_stop();
bool controller_state_consume_stop_request();
