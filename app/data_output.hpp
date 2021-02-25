#pragma once

template <typename T>
class IDataOutput {
public:
    virtual T& operator=(T&& o) = 0;
};