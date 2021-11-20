#pragma once

#include <async++.h>
#include <entt/entt.hpp>
#include <tuple>
#include <type_traits>

template <typename T>
struct is_tuple: std::false_type {};

template <typename... Args>
struct is_tuple<std::tuple<Args...>> : std::true_type {};

template <typename T>
constexpr const bool is_tuple_v = is_tuple<T>::value;

template <typename F, typename... T>
struct apply_result : std::invoke_result<F, T...> {};

template <typename F, typename... Args>
struct apply_result<F, std::tuple<Args...>> : std::invoke_result<F, Args...> {};

template <typename F>
struct apply_result<F, void> : std::invoke_result<F> {};

template <typename F, typename... T>
using apply_result_t = typename apply_result<F, T...>::type;

async::threadpool_scheduler& async_scheduler();
async::fifo_scheduler& sync_scheduler();

template <typename Component>
struct SharedLock {
    size_t count;
};

template <typename Component>
struct UniqueLock {
};

template <typename Result>
class Task {
public:
    Task(async::task<Result>&& _task)
        : task(std::move(_task))
    {
    }

    template <typename F>
    auto then_sync(F&& f)
    {
        return then_sched(std::forward<F>(f), sync_scheduler());
    }

    template <typename F>
    auto then_async(F&& f)
    {
        return then_sched(std::forward<F>(f), async_scheduler());
    }

    template <typename F>
    auto then_inline(F&& f)
    {
        return then_sched(std::forward<F>(f), async::inline_scheduler());
    }

    /*
    Start() -> T
    End(T&&) -> void 
    */
    template <typename F, typename Start, typename End>
    auto guard(F&&f, Start&& start, End&& end)
    {
        return this->then_inline([start = std::forward<Start>(start)](Result&& result) {
            return std::make_pair(std::move(start()), std::move(result));
        }).then_async([f = std::forward<F>(f)](auto&& pair){
            return std::make_pair(std::move(pair.first), std::move(f(std::move(pair.second))));
        }).then_inline([end = std::forward<End>(end)](auto&& pair) {
            std::invoke(end, pair.first);
            return pair.second;
        });
    }

    Task<void> and_discard_result()
    {
        return this->then_sync([](){});
    }

    /// Wait for result and get return value
    Result get()
    {
        return task.get();
    }

    /// Runs all sync operations in the current thread and return this task's result
    Result run_and_get()
    {
        sync_scheduler().run_all_tasks();
        return task.get();
    }

    friend Task<void> create_task();

    friend Task<Result> make_task_from(async::task<Result>&&);

    template <typename F, typename Sched>
    auto then_sched(F&& f, Sched& sched) 
    {
        using ResultType = apply_result_t<F, Result>;

        if constexpr (is_tuple_v<Result>) {
            return Task<ResultType>(task.then(sched, [f = std::forward<F>(f)](Result&& r){
                return std::apply(f, r);
            }));
        } else if constexpr (std::is_void_v<Result>) {
            return Task<ResultType>(std::move(task.then(sched, [f = std::forward<F>(f)](){
                return std::invoke(f);
            })));
        } else {
            return Task<ResultType>(task.then(sched, [f=std::forward<F>(f)](Result&& r){
                return std::invoke(f, std::forward<Result>(r));
            }));
        }
    }

    async::task<Result> task;
};

inline Task<void> create_task()
{
    return Task<void>(async::spawn(async::inline_scheduler(), [](){}));
}

template <typename Result>
inline Task<Result> make_task_from(async::task<Result>&& task)
{
    return Task<Result>(std::move(task));
}