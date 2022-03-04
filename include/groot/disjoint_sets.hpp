#pragma once

#include <cstddef>
#include <utility>

namespace groot {

class DisjointSets {
public:
    DisjointSets(size_t n)
        : data(new size_t[n])
        , n_components(n)
        , size(n)
    {
        for (size_t i = 0; i < n; i++) {
            data[i] = i;
        }
    }

    ~DisjointSets()
    {
        delete[] data;
    }

    DisjointSets(DisjointSets&& s)
        : data(std::exchange(s.data, nullptr))
        , n_components(std::exchange(s.n_components, 0))
        , size(std::exchange(s.size, 0))
    {
    }

    DisjointSets& operator=(DisjointSets&& s)
    {
        data = std::exchange(s.data, nullptr);
        n_components = std::exchange(s.n_components, 0);
        size = std::exchange(s.size, 0);
        return *this;
    }

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
            n_components--;
        }
    }

    size_t component_count() const
    {
        return n_components;
    }

    size_t num_vertices() const
    {
        return size;
    }


private:
    size_t* data;
    size_t n_components;
    size_t size;
};

}