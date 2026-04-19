#pragma once
#include <Arduino.h>
#include "app_state.h"

// Compact binary snapshot (CBOR) consumed by the dashboard for fast refreshes.

// Builds the CBOR snapshot as per the specified schema.
// Returns number of bytes written, or 0 on failure.
size_t build_snapshot_cbor(const AppState& g, uint8_t* out, size_t cap);
