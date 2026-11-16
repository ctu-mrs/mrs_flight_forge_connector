#pragma once

#include <ftxui/component/component.hpp>

#include "app.h"
#include "viz.h"

// Teleport "driving": every key press adds a fixed step to the locally held
// pose and sends it with SetLocationAndRotation. No dynamics.
ftxui::Component DriveComponent(App& app, ViewSettings* view);

// called when the Drive tab becomes visible
void DriveOnEnter(App& app);
