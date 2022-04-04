#pragma once

#include <glm/glm.hpp>
#include <groot/cgal.hpp>

namespace groot {

inline glm::vec3 to_glm(const Point_3& p)
{
    return glm::vec3(p.x(), p.y(), p.z());
}

inline glm::vec3 to_glm(const Vector_3& p)
{
    return glm::vec3(p.x(), p.y(), p.z());
}

template <glm::length_t L, typename T>
inline Vector_3 to_vector(const glm::vec<L, T>& p)
{
    return Vector_3(p.x, p.y, p.z);
}

template <glm::length_t L, typename T>
inline Point_3 to_point(const glm::vec<L, T>& p)
{
    return Point_3(p.x, p.y, p.z);
}

template <typename OutIterator, typename InIterator>
inline void to_vector_iterator(InIterator begin, InIterator end, OutIterator out)
{
    using T = std::remove_cv_t<typename std::iterator_traits<InIterator>::value_type>;
    using LengthType = typename T::length_type;
    using ValueType = typename T::value_type;
    constexpr const LengthType length = T::length();

    std::transform(begin, end, out, &to_vector<length, ValueType>);
}

template <typename OutIterator, typename InIterator>
inline void to_point_iterator(InIterator begin, InIterator end, OutIterator out)
{
    using T = std::remove_cv_t<typename std::iterator_traits<InIterator>::value_type>;
    using LengthType = typename T::length_type;
    using ValueType = typename T::value_type;
    constexpr const LengthType length = T::length();

    std::transform(begin, end, out, &to_point<length, ValueType>);
}

}