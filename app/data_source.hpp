#pragma once

#include <utility>

template <typename T>
class IDataSource {
public:
    virtual T& operator*() = 0;
    virtual const T& operator*() const = 0;
};

template <typename T>
using IDataSourceChanged = void();

using IDataSourceDestroyed = void();

template <typename T>
class OwnedDataSource {
public:
    OwnedDataSource(T&& _data)
        : data(std::move(_data))
    {
    }

    virtual T& operator*() { return data; }
    virtual const T& operator*() const { return data; }

private:
    T data;
};