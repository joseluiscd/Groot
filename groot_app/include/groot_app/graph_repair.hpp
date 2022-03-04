#pragma once

#include <groot_app/entt.hpp>
#include <groot_app/command_gui.hpp>
#include <groot_app/command.hpp>
#include <groot_app/components.hpp>
#include <groot_app/task.hpp>

async::task<entt::entity> graph_compute_connected_components(entt::handle h);
async::task<entt::entity> graph_repair_command(entt::handle h);