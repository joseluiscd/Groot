Registry = {}

-- Get the registry instance
function Registry.get()
    return "Registry"
end

-- Get the selected entity
function Registry:selected(self)
    return "Entity"
end

-- Load a registry from a file
function Registry:load(self, filename)
    return "Ok"
end

-- Save registry to file
function Registry:save(self, filename)
    return "Ok"
end

Entity = {}

function Entity.new()