#include "task.hpp"

void pre_run()
{
    static size_t threads = std::thread::hardware_concurrency();
}

async::threadpool_scheduler& async_scheduler()
{

    return async::default_threadpool_scheduler();
}

async::fifo_scheduler& sync_scheduler()
{
    static async::fifo_scheduler scheduler;
    return scheduler;
}