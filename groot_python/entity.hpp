#pragma once

#include "groot_app/cloud_system.hpp"
#include "python.hpp"
#include "python_task.hpp"
#include <groot_app/components.hpp>
#include <groot_app/entt.hpp>

#include <groot_app/application.hpp>
#include <groot_app/cloud_io.hpp>
#include <groot_app/cloud_system.hpp>
#include <groot_app/components.hpp>
#include <groot_app/create_graph.hpp>
#include <groot_app/cylinder_marching.hpp>
#include <groot_app/graph_cluster.hpp>
#include <groot_app/graph_io.hpp>
#include <groot_app/graph_resample.hpp>

void create_entity_type(py::module_& m);

struct AnyComponent {
};

using DynamicComponent = std::pair<entt::type_info, void*>;

template <>
struct entt::storage_traits<entt::entity, AnyComponent> {
    using storage_type = entt::basic_sparse_set<entt::entity>;
};

class Entity {
public:
    static entt::type_info type_id;

    Entity(entt::registry& _reg, entt::entity _e)
        : e(_reg, _e)
    {
    }

    Entity(const entt::handle& h)
        : e(h)
    {
    }

    Entity(entt::handle&& h)
        : e(h)
    {
    }

    auto to_py_entity()
    {
        return [&reg = *e.registry()](entt::entity e) {
            return py::cast(Entity(reg, e));
        };
    }

    void destroy()
    {
        e.destroy();
    }

    template <typename Component>
    Component* get_component()
    {
        return e.try_get<Component>();
    }

    template <typename Component>
    void remove_component()
    {
        e.remove<Component>();
    }

    py::object get_component_runtime(py::object parent, const entt::type_info& type)
    {
        AcquireGilGuard guard;

        auto& storage = e.registry()->storage<AnyComponent>(type.hash());
        if (!storage.contains(e)) {
            return py::none();
        }
        void* component = storage.get(e);

        return any_to_python(parent, component, type);
    }

    void set_component_runtime(const entt::type_info& type, py::object component)
    {
        auto& storage = e.registry()->storage<entt::meta_any>(type.hash());

        entt::meta_any constructed = entt::resolve(type).construct(component.ptr());
        assert((bool)constructed);

        if (storage.contains(e.entity())) {
            storage.erase(e.entity());
        }

        storage.emplace(e.entity(), constructed.as_ref());
    }

    void remove_component_runtime(const entt::type_info& type)
    {
        auto& storage = e.registry()->storage<void*>(type.hash());
        storage.remove(e.entity());
    }

    void move_component(Entity& target, const entt::type_info& type)
    {
        ReleaseGilGuard guard;

        if (e.registry() != target.e.registry()) {
            throw std::runtime_error("Source and target entities must be in the same registry");
        }

        if (target.e == e) {
            // Nothing to do, as they are the same
            return;
        }

        auto& storage = e.registry()->storage<AnyComponent>(type.hash());

        if (!storage.contains(e)) {
            throw std::runtime_error("Entity does not have this component");
        }

        storage.remove(target.e);
        storage.emplace(target.e, storage.get(e));
        storage.remove(e);
    }

    void select()
    {
        e.registry()->ctx<SelectedEntity>().selected = e.entity();
    }

    bool is_visible()
    {
        return e.all_of<Visible>();
    }

    void set_visible(bool v)
    {
        if (v) {
            e.emplace_or_replace<Visible>();
        } else {
            e.remove<Visible>();
        }
    }

    std::string* get_name()
    {
        Name* name = e.try_get<Name>();
        return name == nullptr ? &name->name : nullptr;
    }

    void set_name(const std::string& v)
    {
        e.emplace_or_replace<Name>(v);
    }

    /// @return None
    PythonTask compute_normals(
        int k,
        float radius)
    {
        return compute_normals_command(e, k, radius);
    }

    /// @return None
    PythonTask cylinder_marching(
        int min_points,
        float epsilon,
        float sampling,
        float normal_deviation,
        float overlook_probability,
        float voxel_size)
    {
        ReleaseGilGuard guard;
        groot::Ransac::Parameters params;

        params.min_points = min_points;
        params.epsilon = epsilon;
        params.cluster_epsilon = sampling;
        params.normal_threshold = std::cos(normal_deviation * M_PI / 180.0);
        params.probability = overlook_probability;

        return cylinder_marching_command(e, params, voxel_size);
    }

    PythonTask cylinder_filter(
        float radius_min,
        float radius_max,
        float length_min,
        float length_max)
    {
        CylinderFilterParams params;

        params.radius = true;
        params.radius_range[0] = radius_min;
        params.radius_range[1] = radius_max;
        params.length = true;
        params.length_range[0] = length_min;
        params.length_range[1] = length_max;

        return cylinder_filter_command(e, params);
    }

    /// @return None
    PythonTask rebuild_cloud_from_cylinders()
    {
        return cylinder_point_filter_command(e);
    }

    /// @return None
    PythonTask build_graph_from_cylinders()
    {
        return cylinder_connect_graph_command(e);
    }

    void graph_cluster(int intervals)
    {
        ReleaseGilGuard guard;

        GraphCluster cmd { entt::handle(e) };
        cmd.interval_count = intervals;
        cmd.run(*e.registry());
    }

    void graph_from_cloud_knn(int k)
    {
        ReleaseGilGuard guard;

        CreateGraph cmd { entt::handle(e) };
        cmd.selected_method = CreateGraph::Method::kKnn;
        cmd.k = k;
        cmd.run(*e.registry());
    }

    void graph_from_cloud_radius(float r)
    {
        ReleaseGilGuard guard;

        CreateGraph cmd { entt::handle(e) };
        cmd.selected_method = CreateGraph::Method::kRadius;
        cmd.radius = r;
        cmd.run(*e.registry());
    }

    void graph_from_alpha_shape(float k)
    {
        ReleaseGilGuard guard;

        CreateGraph cmd { entt::handle(e) };
        cmd.selected_method = CreateGraph::Method::kAlphaShape;
        cmd.alpha = k;
        cmd.run(*e.registry());
    }

    /// @return Entity
    PythonTask graph_resample(float length)
    {
        return PythonTask {
            graph_resample_command(e, length)
                .then(sync_scheduler(), to_py_entity())
        };
    }

    /// @return Entity
    PythonTask match_graph(Entity other)
    {
        return PythonTask {
            graph_match_command(this->e, other.e)
                .then(sync_scheduler(), to_py_entity())
        };
    }

    PythonTask save_ply(const std::string& filename)
    {
        return export_ply_command(e, filename);
    }

    entt::handle e;
};