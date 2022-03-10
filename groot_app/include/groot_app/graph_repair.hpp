#pragma once

#include <groot_app/entt.hpp>
#include <groot_app/command_gui.hpp>
#include <groot_app/command.hpp>
#include <groot_app/components.hpp>
#include <groot_app/task.hpp>

GROOT_APP_API async::task<void> graph_compute_connected_components(entt::handle h);
GROOT_APP_API async::task<void> graph_repair_command(entt::handle h);