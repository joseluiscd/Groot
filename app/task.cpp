#include "task.hpp"

async::threadpool_scheduler& async_scheduler()
{
    return async::default_threadpool_scheduler();
}

async::fifo_scheduler& sync_scheduler()
{
    static async::fifo_scheduler scheduler;
    return scheduler;
}