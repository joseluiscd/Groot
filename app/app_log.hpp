#pragma once

#include <gfx/imgui/imgui.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/spdlog.h>


struct AppLog : public spdlog::sinks::base_sink<std::mutex> {
    AppLog() {
    }

    ImGuiTextBuffer Buf;
    bool ScrollToBottom;

    void Clear() { Buf.clear(); }

    void AddLog(const char* str)
    {
        Buf.append(str);
        ScrollToBottom = true;
    }

    void Draw(const char* title, bool* p_opened = NULL)
    {
        ImGui::Begin(title, p_opened);
        ImGui::TextUnformatted(Buf.begin());
        if (ScrollToBottom)
            ImGui::SetScrollHereY(1.0f);
        ScrollToBottom = false;
        ImGui::End();
    }

    void sink_it_(const spdlog::details::log_msg& msg) override
    {

        // log_msg is a struct containing the log entry info like level, timestamp, thread id etc.
        // msg.raw contains pre formatted log

        // If needed (very likely but not mandatory), the sink formats the message before sending it to its final destination:
        spdlog::memory_buf_t formatted;
        spdlog::sinks::base_sink<std::mutex>::formatter_->format(msg, formatted);
        AddLog(fmt::to_string(formatted).data());
    }

    void flush_() override
    {
    }
};
