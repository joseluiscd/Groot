#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <sstream>
#include <string>

namespace groot {

template <typename T>
struct GlmFromString {
    T operator()(const std::string& s);
};

template <typename T>
struct GlmToString {
    std::string operator()(const T& v);
};

template <>
inline std::string GlmToString<glm::vec3>::operator()(const glm::vec3& v)
{
    std::stringstream ss;
    ss << v.x << ',' << v.y << ',' << v.z;

    return ss.str();
}

template <>
glm::vec3 GlmFromString<glm::vec3>::operator()(const std::string& s)
{
    char comma;
    glm::vec3 v;
    std::stringstream ss(s);
    ss >> v.x >> comma >> v.y >> comma >> v.z;

    return v;
}

}
