params = {
    input = "/home/joseluis/Trees/sampled.ply",
    output = "cylinders.ggf",

    normals = {
        k = 99999,
        radius = 0.08
    },

    cylinders = {
        min_points = 500,
        epsilon = 0.03,
        sampling = 0.06,
        normal_deviation = 25.0,
        overlook_probability = 0.01,
        voxel_size = 0.5,
    },

    cylinder_filter = {
        radius = { 0.0, 0.8 },
        length = { 0.0, 9999.99 }
    }
}