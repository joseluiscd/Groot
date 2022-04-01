#pragma once

#include <groot_app/task.hpp>

GROOT_APP_API async::task<void> compute_branch_weights_task(entt::handle h);
GROOT_APP_API async::task<void> compute_orientation_field_task(entt::handle h);
GROOT_APP_API async::task<void> reconstruct_livny_task(entt::handle h, groot::point_finder::PointFinder& pf = groot::point_finder::MinYPointFinder);
