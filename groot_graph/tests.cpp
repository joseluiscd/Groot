#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include <groot_graph/plant_graph.hpp>
#include <groot_graph/plant_graph_compare.hpp>


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
