#pragma once

#include <stack>

template <typename Input, typename T>
struct CommandInputTrait {
    static constexpr bool value = false;
    static const Input& load(const T& t) = 0;
};

template <typename Output, typename T>
struct CommandOutputTrait {
    static constexpr bool value = false;
    void store(Output&& o, T& c);
};

template <typename T>
struct CommandOutputTrait<T, T*> {
    static constexpr bool value = true;
    void store(T&& t, T*& c)
    {
        *c = std::move(t);
    }
};

template <typename T>
struct CommandOutputTrait<T, std::stack<T>> {
    static constexpr bool value = true;
    void store(T&& t, std::stack<T>& c)
    {
        c.emplace(t);
    }
};

template <typename Input, typename Output>
class Command {
public:
    void run();
};