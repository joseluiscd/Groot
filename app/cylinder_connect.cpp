#include "cylinder_connect.hpp"


CylinderConnection::CylinderConnection(entt::handle&& handle)
    : reg(*handle.registry())
{
    target = handle.entity();
    if (reg.valid(target) && reg.all_of<Cylinders>(target)) {
        cylinders = &reg.get<Cylinders>(target);
    } else {
        throw std::runtime_error("Selected entity must have Cylinders component");
    }
}

CommandState CylinderConnection::execute()
{
    result = groot::connect_cylinders(cylinders->cylinders.data(), cylinders->cylinders.size());
    return CommandState::Ok;
}

void CylinderConnection::on_finish(entt::registry& reg)
{
    reg.emplace_or_replace<groot::PlantGraph>(target, std::move(*result));
}