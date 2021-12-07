#pragma once

#include <groot/groot.hpp>
#include <async++.h>
#include <groot_app/entt.hpp>

async::threadpool_scheduler& async_scheduler();
async::fifo_scheduler& sync_scheduler();

inline void discard() { }

class GROOT_LOCAL TaskBroker {
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