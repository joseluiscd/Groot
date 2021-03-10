#include <algorithm>
#include <groot/particles.hpp>
#include <glm/gtx/norm.hpp>

namespace groot {

void Particles::step(float dt)
{
    pre_step(dt);

    for (size_t i = 0; i < positions.size(); i++) {
        // Update using Symplectic/Semi-implicit Euler
        velocities[i] += accelerations[i] * dt;
        positions[i] += velocities[i] * dt;
    }

    post_step(dt);
}

TreeParticles::TreeParticles(const std::vector<glm::vec3>& _cloud)
    : Particles(_cloud.size(), _cloud.size(), _cloud.size())
    , masses(_cloud.size(), 1.0f)
{
    std::transform(_cloud.begin(), _cloud.end(), positions.end(), [](const glm::vec3& v) { return glm::vec4(v, 1.0); });
}

void TreeParticles::pre_step(float dt)
{
    for (size_t i = 0; i < positions.size(); i++) {
        accelerations[i] = glm::vec4(0.0);
        for (size_t j = 0; j < i;) {
            float distance = glm::distance2(positions[i], positions[j]);
            glm::vec4 direction = glm::normalize(positions[j] - positions[i]);
            accelerations[i] = direction * attraction_constant * masses[j] / distance;
        }
    }
}

void TreeParticles::post_step(float dt)
{
    size_t last = positions.size() - 1;
    for (size_t i = 0; i < positions.size(); i++) {
        for (size_t j = 0; j < positions.size();) {
            if (i != j && glm::distance(positions[i], positions[j]) < this->merge_threshold) {
                masses[i] += masses[j];
                masses[j] = masses[last];

                velocities[i] += velocities[j];
                velocities[j] = velocities[last];

                positions[i] += positions[j] * 0.5f;
                positions[j] = positions[last];
            }

        }
    }
}

}
