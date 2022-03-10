#pragma once

#include <groot/assert.hpp>
#include <cstddef>
#include <utility>
#include <vector>
#include <cereal/cereal.hpp>
#include <cereal/types/vector.hpp>

namespace groot {

class DisjointSets {
public:
    DisjointSets(size_t n = 0)
        : data(n)
        , n_sets(n)
    {
        GROOT_ASSERT(data.size() == n, "Vector size");

        for (size_t i = 0; i < n; i++) {
            data[i] = i;
        }
    }

    DisjointSets(DisjointSets&& s) = default;
    DisjointSets& operator=(DisjointSets&& s) = default;

    size_t find(size_t i) const
    {
        while (i != data[i]) {
            i = data[i];
        }
        return i;
    }

    size_t find(size_t i)
    {
        if (i == data[i]) return i;
        size_t set = find(data[i]);
        data[i] = set;
        return set;
    }

    void union_set(size_t i, size_t j)
    {
        size_t is = find(i);
        size_t js = find(j);

        if (is != js) {
            data[is] = js;
            n_sets--;
        }
    }

    size_t component_count() const
    {
        return n_sets;
    }

    size_t num_vertices() const
    {
        return data.size();
    }

    template <typename Archive>
    void serialize(Archive& archive)
    {
        archive(CEREAL_NVP(n_sets), CEREAL_NVP(data));
    }


private:
    std::vector<size_t> data;
    size_t n_sets;
};

}