#pragma once

#include <groot/mesh.hpp>
#include <groot_graph/plant_graph.hpp>

namespace groot {

Mesh generate_mesh(const PlantGraph& graph, const PropertyMap<float>& radii, const PropertyMap<Vector_3>& tangents);

}