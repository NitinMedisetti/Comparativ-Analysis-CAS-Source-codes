#pragma once

#include "app_state.h"

namespace nav {

// Set to true once the system has been started (e.g., after checkup / UI action).
extern bool g_system_started;

// Handle incoming nav commands triggered by UI buttons.
void handle_nav_command(const char* action);

// Run the navigation FSM and planner using the latest sensor data.
// Writes the navigation snapshot back into `writable.nav` and updates
// `writable.oled_label` each tick.
void nav_tick(const AppState& sensors, AppState& writable);

// Returns the most recent navigation state computed by the decision loop.
const NavigationState& nav_state();

}  // namespace nav
