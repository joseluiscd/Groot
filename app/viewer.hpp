#pragma once
#include "data_source.hpp"
#include <gfx/camera.hpp>
#include <gfx/framebuffer.hpp>
#include <gfx/imgui/gfx.hpp>
#include <spdlog/spdlog.h>

class AbstractViewer {
public:
    AbstractViewer();

    virtual ~AbstractViewer() { }
    virtual void update() { }
    virtual void remove() { }

    virtual void render() = 0;
    virtual bool draw_gui();
protected:
    bool open;
    gfx::Framebuffer framebuffer;
    gfx::CameraRig camera_rig;
    gfx::PerspectiveCameraLens camera_lens;
    glm::ivec2 size;
};

template <typename T>
class Viewer : public AbstractViewer {
public:
    Viewer(IDataSource<T>& _input);
    ~Viewer() {}

protected:
    IDataSource<T>& input;
};

template <typename T>
Viewer<T>::Viewer(IDataSource<T>& _input)
    : AbstractViewer()
    , input(_input)
{
}