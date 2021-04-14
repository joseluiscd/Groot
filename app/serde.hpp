#pragma once

#include "components.hpp"
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

template <typename Archive>
struct Serde {
    Archive archive;

    template <typename... Args>
    Serde(Args&&... a)
        : archive(std::forward<Args>(a)...)
    {
    }

    template <typename T, typename... Args>
    void operator()(T&& a, Args&&... rest)
    {
        (*this)(a);
        (*this)(std::forward<Args>(rest)...);
    }

    template <typename T>
    void operator()(T&& a)
    {
        archive& a;
    }
};

using Serializer = Serde<boost::archive::binary_oarchive>;
using Deserializer = Serde<boost::archive::binary_iarchive>;

namespace boost {
namespace serialization {

template <typename Archive>
void serialize(Archive& ar, Name& name, unsigned int) {
    ar & name.name;
}

template <typename Archive>
void serialize(Archive& ar, PointCloud& cloud, unsigned int) {
    ar & cloud.cloud;
}

template <typename Archive>
void serialize(Archive& ar, PointNormals& normals, unsigned int) {
    ar & normals.normals;
}

template <typename Archive>
void serialize(Archive& ar, Cylinders& cylinders, unsigned int) {
    ar & cylinders.cylinders;
}


}
}
