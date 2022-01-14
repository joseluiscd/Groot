#pragma once

#include <groot_app/groot_app.hpp>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/spdlog.h>
#include <gfx/imgui/imgui.h>

/// Modified AppLog from ImGui
struct GROOT_APP_LOCAL AppLog : public spdlog::sinks::base_sink<std::mutex> {
    ImGuiTextBuffer buffer;
    bool ScrollToBottom = true;

    AppLog()
        : buffer()
    {
    }

    void clear() { buffer.clear(); }

    void add_log(const char* str)
    {
        buffer.append(str);
        ScrollToBottom = true;
    }

    void draw(const char* title, bool* p_opened = nullptr);

    void sink_it_(const spdlog::details::log_msg& msg) override;

    void flush_() override
    {
    }
};
