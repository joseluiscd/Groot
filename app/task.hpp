#pragma once

#include <async++.h>
#include <entt/entt.hpp>

async::threadpool_scheduler& async_scheduler();
async::fifo_scheduler& sync_scheduler();

inline void discard() { }

class TaskBroker {
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

    /**
    Cycle through tasks. Apply function `f` to active tasks.
    */
    template <typename F>
    void cycle_tasks(F&& f)
    {
        auto it = tasks.begin();
        while (it != tasks.end()) {
            if (it->second.ready()) {
                tasks.erase(it++);
            } else {
                std::invoke(f, std::string_view(it->first));
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