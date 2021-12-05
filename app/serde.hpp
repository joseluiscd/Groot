#pragma once

#include "components.hpp"
#include <groot_graph/plant_graph_serialize.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/archives/binary.hpp>

using Serializer = cereal::BinaryOutputArchive;
using Deserializer = cereal::BinaryInputArchive;

namespace cereal {

template <typename Archive>
void serialize(Archive& ar, Name& name) {
    ar(name.name);
}

template <typename Archive>
void serialize(Archive& ar, PointCloud& cloud) {
    ar(cloud.cloud);
}

template <typename Archive>
void serialize(Archive& ar, PointNormals& normals) {
    ar(normals.normals);
}

template <typename Archive>
void serialize(Archive& ar, Cylinders& cylinders) {
    ar(cylinders.cylinders);
}

}
