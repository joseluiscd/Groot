#pragma once

#include <future>
#include <memory>

struct CylinderMarchingParams {
};

template <typename Function, typename Parameters, typename Result, typename UpdateFunc>
class BackgroundTaskProcess : public entt::process<BackgroundTaskProcess, float> {
public:
    BackgroundTaskProcess(Function f, Parameters&& _params, UpdateFunc update_f)
        : params(std::move(_params))
        , function(f)
        , update_function(update_f)
    {
    }

    void init()
    {
        result = std::async([&]() {
            std::unique_ptr<void> deferred(nullptr, std::bind([&]{
                finished = true;
            }));
            return function(params);
        });
    }

    void update(float delta, void*)
    {
        if (!finished) {
            
        }
        update_function(delta);
    }

private:
    std::atomic<bool> finished = false;
    std::future<Result> result;
    entt::scheduler asdf;

    Function function;
    UpdateFunc update_function;
    Parameters params;
};
