#pragma once

#include <future>
#include <gfx/imgui/imgui.h>
#include <variant>

template <typename Result>
class Operation {

public:
    Operation(std::function<void(Result&&)> on_result);
    
    virtual std::variant<Result, std::string> operation() const = 0;
    virtual void window() = 0;

    void run_gui();
    void run();

    void show() { _show = true; }
    void hide() { _show = false; }

protected:
    bool running;
    bool _show;

    bool _error;
    std::string _error_string;

    std::function<void(Result&&)> on_result;

    std::future<std::variant<Result, std::string>> result;
};

template <typename Result>
Operation<Result>::Operation(std::function<void(Result&&)> _on_result)
    : on_result(_on_result)
    , running(false)
    , _show(false)
    , _error(false)
    , _error_string("")
{
}

template <typename Result>
void Operation<Result>::run()
{
    result = std::async([&](){
        return this->operation();
    });

    running = true;
}

template <typename Result>
void Operation<Result>::run_gui()
{
    if (! _show) {
        return;
    }

    this->window();

    if (running) {
        if (result.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            std::variant<Result, std::string> r(std::move(result.get()));
            
            running = false;
            
            if (r.index() == 0) {
                on_result(std::move(std::get<0>(r)));
                this->hide();
            } else {
                _error = true;
                _error_string = std::move(std::get<1>(r));
            }

            return;
        }

        if (ImGui::BeginPopupModal("Running")) {
            ImGui::Text("Please wait");
            ImGui::EndPopup();
        }

        ImGui::OpenPopup("Running");
    }

    if (_error) {
        if (ImGui::BeginPopupModal("Error")) {
            ImGui::Text(_error_string.c_str());

            if (ImGui::Button("Ok")) {
                _error_string = "";
                _error = false;
                ImGui::CloseCurrentPopup();
            }

            ImGui::EndPopup();
        }

        ImGui::OpenPopup("Error");
    }
}