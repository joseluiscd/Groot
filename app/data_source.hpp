#pragma once

template <typename T>
class IDataSource {
public:
    virtual T& operator*() = 0;
    virtual const T& operator*() const = 0;
};

template <typename T>
using IDataSourceChanged = void();


using IDataSourceDestroyed = void();
