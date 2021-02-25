#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace groot {

std::vector<glm::vec3> load_PLY(const char* filename);

}