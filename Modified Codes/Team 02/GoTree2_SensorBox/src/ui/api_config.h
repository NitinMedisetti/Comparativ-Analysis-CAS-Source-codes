#pragma once

#include <stdint.h>

#include "config/dev_params.h"

namespace ui {

// HTTP/WebSocket endpoints exposed by `server_http.*`.

constexpr const char* kEndpointDataCbor = "/data.cbor";
constexpr const char* kEndpointTelemetryWs = "/ws/telemetry";
constexpr const char* kEndpointNavStop = "/nav/stop";
constexpr const char* kEndpointNavGotoPost = "/nav/goto_post";
constexpr const char* kEndpointNavGotoH = "/nav/goto_h";
constexpr const char* kEndpointActuatorInitialCheckup = "/actuator/initial_checkup";
constexpr const char* kEndpointActuatorStop = "/actuator/stop";
constexpr const char* kEndpointConfig = "/config";

constexpr uint32_t kSnapshotIntervalMs = SNAPSHOT_HZ ? (1000 / SNAPSHOT_HZ) : 1000;

}  // namespace ui
