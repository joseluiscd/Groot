#pragma once
#include <glm/glm.hpp>
#include <groot_graph/plant_graph.hpp>
#include <vector>

namespace groot {

class Particles {
public:
    template <typename T, typename P, typename Q>
    Particles(T _positions, P _velocities, Q _accelerations)
        : positions(_positions)
        , velocities(_velocities)
        , accelerations(_accelerations)
    {
    }

    void step(float dt);

    /// Called by step(float)
    virtual void pre_step(float dt) { }

    /// Called by step(float)
    virtual void post_step(float dt) { }

protected:
    std::vector<glm::vec4> positions;
    std::vector<glm::vec4> velocities;
    std::vector<glm::vec4> accelerations;
};

class TreeParticles : public Particles {
public:
    TreeParticles(const std::vector<glm::vec3>& _cloud);
    virtual void pre_step(float dt) override;
    virtual void post_step(float dt) override;

    /// Distance to merge two particles
    void set_merge_threshold(float _merge_threshold) { merge_threshold = _merge_threshold; }

    /// Set gravitation constant
    void set_attraction_constant(float _attraction_constant) { attraction_constant = _attraction_constant; }

private:
    std::vector<float> masses;

    PlantGraph graph;
    std::vector<Vertex> last_vertices;

    //# Parameters

    float merge_threshold = 0.1f;
    float attraction_constant = 0.1f;
};

}
