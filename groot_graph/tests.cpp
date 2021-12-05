#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include <groot_graph/plant_graph.hpp>
#include <groot_graph/plant_graph_compare.hpp>
#include <groot_graph/cylinder_marching.hpp>


TEST_CASE("cylinder inclusion test")
{
    groot::Cylinder c1 = {
        .center = groot::Point_3(0.0, 0.0, 0.0),
        .direction = groot::Vector_3(1.0, 0.0, 0.0),
        .radius = 0.5,
        .middle_height = 0.5,
    };
    //Points in the cylinder
    groot::Point_3 p1(0.0, 0.0, 0.0);
    groot::Point_3 p2(0.499, 0.49, 0.0);

    CHECK(point_in_cylinder(p1, c1));
    CHECK(point_in_cylinder(p2, c1));

    //Points outside the cylinder
    groot::Point_3 p3(0.9, 0.0, 0.0);
    groot::Point_3 p4(0.499, 0.49, 0.49);

    CHECK_FALSE(point_in_cylinder(p3, c1));
    CHECK_FALSE(point_in_cylinder(p4, c1));
}


TEST_CASE("Graph Resample Empty")
{
    groot::PlantGraph g;

    groot::PlantGraph r = groot::resample_plant_graph(g, 1.0);

    REQUIRE_EQ(boost::num_vertices(r), 0);
    REQUIRE_EQ(boost::num_edges(r), 0);
}

TEST_CASE("Graph Resample single edge")
{
    using namespace groot;
    PlantGraph g;
    Vertex a = boost::add_vertex(g);
    Vertex b = boost::add_vertex(g);

    g[a].position = Point_3(0.0, 0.0, 0.0);
    g[b].position = Point_3(0.0, 1.001, 0.0);

    Edge e = boost::add_edge(a, b, g).first;
    g[e].length = 1.0;

    g.m_property->root_index = a;

    SUBCASE("Oversample")
    {
        PlantGraph r = resample_plant_graph(g, 0.33);

        CHECK_EQ(boost::num_vertices(r), 4);
        CHECK_EQ(boost::num_edges(r), 3);
        CHECK_EQ(g[g.m_property->root_index].position, r[r.m_property->root_index].position);
    }

    SUBCASE("Same resolution")
    {
        PlantGraph r = resample_plant_graph(g, 1.0);

        CHECK_EQ(boost::num_vertices(r), 2);
        CHECK_EQ(boost::num_edges(r), 1);
        CHECK_EQ(g[g.m_property->root_index].position, r[r.m_property->root_index].position);
    }

}
