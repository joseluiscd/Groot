#pragma once

#include <groot_app/entt.hpp>
#include <groot_app/command_gui.hpp>
#include <groot_app/command.hpp>
#include <groot_app/components.hpp>
#include <groot_app/task.hpp>
#include <boost/iterator/transform_iterator.hpp>

struct GraphResampleArgs {
    float sample_length = 0.3;
};

async::task<entt::entity> graph_resample_command(entt::handle h, float sample_length);
async::task<entt::entity> graph_match_command(entt::handle h1, entt::handle h2);

template <typename Iterator>
auto graph_resample_command(entt::registry& reg, Iterator begin, Iterator end, float sample_length)
{
    auto f = [&](entt::entity e){
        return graph_resample_command(entt::handle(reg, e), sample_length);
    };
    return async::when_all(boost::make_transform_iterator(begin, f), boost::make_transform_iterator(end, f));
}

class GraphResampleGui: public DialogGui {
public:
    GraphResampleGui(std::vector<entt::entity>&& _targets, const GraphResampleArgs& _args = GraphResampleArgs{})
        : args(_args)
        , targets(std::move(_targets))
    {}

    void schedule_commands(entt::registry& reg) override;
    void draw_dialog() override;
    std::string_view name() const override { return "Graph resample"; }

private:
    GraphResampleArgs args;
    std::vector<entt::entity> targets;
};