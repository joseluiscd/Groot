#pragma once

#include <async++.h>
#include <groot_app/entt.hpp>
#include <groot_app/groot_app.hpp>

GROOT_APP_API async::threadpool_scheduler& async_scheduler();
GROOT_APP_API async::fifo_scheduler& sync_scheduler();

inline void discard() { }

template <typename Result>
class TaskBuilder;

template <typename T>
TaskBuilder<T> create_task(async::task<T>&& t);

template <typename Result>
class TaskBuilder {
public:
    inline TaskBuilder(async::task<Result>&& _task)
        : task(std::move(_task))
    {
    }

    inline TaskBuilder()
        : task(async::make_task())
    {
    }

    template <typename F, typename S>
    inline auto then(S&& s, F&& f)
    {
        return create_task(
            this->task.then(std::forward<S>(s), std::forward<F>(f)));
    }

    template <typename F>
    inline auto then_sync(F&& f)
    {
        return this->then(sync_scheduler(), std::forward<F>(f));
    }

    template <typename F>
    inline auto then_async(F&& f)
    {
        return this->then(async_scheduler(), std::forward<F>(f));
    }

    template <typename F>
    inline auto then_inline(F&& f)
    {
        return this->then(async::inline_scheduler(), std::forward<F>(f));
    }

    operator async::task<Result>()
    {
        return this->build();
    }

    inline async::task<Result> build()
    {
        async::task<Result> r = std::move(task);
        return r;
    }

private:
    async::task<Result> task;
};

template <typename T>
inline TaskBuilder<T> create_task(async::task<T>&& t)
{
    return TaskBuilder<T>(std::move(t));
}

inline TaskBuilder<void> create_task()
{
    return TaskBuilder<void>();
}


class GROOT_APP_LOCAL TaskBroker {
public:
    TaskBroker()
        : tasks()
    {
    }

    TaskBroker(TaskBroker&&) = default;
    TaskBroker(const TaskBroker&) = delete;

    TaskBroker& operator=(TaskBroker&&) = default;
    TaskBroker& operator=(const TaskBroker&) = delete;

    void push_task(std::string_view s, async::task<void>&& task)
    {
        tasks.emplace_back(s, std::move(task));
    }

    template <typename Result>
    void push_task(std::string_view s, async::task<Result>&& task)
    {
        push_task(s, task.then(discard));
    }

    /**
    Cycle through tasks. Apply function `f` to active tasks.
    On error, call `e` with the exception.
    */
    template <typename F, typename E>
    inline void cycle_tasks(F&& f, E&& err)
    {
        auto it = tasks.begin();
        while (it != tasks.end()) {
            if (it->second.ready()) {
                tasks.erase(it++);
            } else {
                try {
                    std::invoke(f, std::string_view(it->first));
                } catch (const std::exception& e) {
                    err(e.what());
                }
                ++it;
            }
        }
    }

    bool empty()
    {
        return tasks.empty();
    }

private:
    std::list<std::pair<std::string, async::task<void>>> tasks;
};