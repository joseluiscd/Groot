Registry = {}

-- Get the registry instance
function Registry.get()
    return "Registry"
end

-- Get the selected entity
function Registry:selected(self)
    return "Entity"
end

-- Get a list of all the entities
function Registry:entities(self)
    return {"entity1", "entity2", "etc"}
end

-- Load a registry from a file
function Registry:load(self, filename)
    return "Ok"
end

-- Save registry to file
function Registry:save(self, filename)
    return "Ok"
end

-- Import a PLY and return its entity
function Registry:load_ply(self, filename)
    return "Entity"
end

-- ##############################################
Entity = {}

function Entity.new()
    return "new entity"
end

-- Returns the point cloud component
function Entity:point_cloud(self)
end

-- Returns the cloud normals component
function Entity:cloud_normals(self)
end

-- Set entity as selected
function Entity:select(self)
end

function Entity:compute_normals(self, args)
    args = {
        radius = 1.0,
        k = 0
    }
end
-- Runs cylinder marching
function Entity:cylinder_marching(self, args)
    args = {
        min_points = 20,
        epsilon = 0.03,
        sampling = 0.06,
        normal_deviation = 25.0,
        overlook_probability = 0.01,
        voxel_size = 1.0f,
    }
end