#pragma once

#include <CGAL/Alpha_shape_3.h>
#include <CGAL/Alpha_shape_cell_base_3.h>
#include <CGAL/Alpha_shape_vertex_base_3.h>
#include <CGAL/Cartesian_converter.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Incremental_neighbor_search.h>
#include <CGAL/K_neighbor_search.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Regular_triangulation_3.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <groot/cgal.hpp>
#include <groot_graph/plant_graph.hpp>
#include <tuple>

namespace groot::cgal {

typedef std::tuple<Point_3, Vertex> Point3Vertex;

using ExactKernel = CGAL::Exact_predicates_inexact_constructions_kernel;

typedef CGAL::Triangulation_vertex_base_with_info_3<Vertex, ExactKernel> DelaunayVertex;
typedef CGAL::Triangulation_data_structure_3<DelaunayVertex> Tds;
typedef CGAL::Delaunay_triangulation_3<ExactKernel, Tds> Delaunay;

typedef CGAL::Alpha_shape_vertex_base_3<ExactKernel, CGAL::Triangulation_vertex_base_with_info_3<Vertex, ExactKernel>> AlphaVertex;
typedef CGAL::Triangulation_data_structure_3<AlphaVertex, CGAL::Alpha_shape_cell_base_3<ExactKernel>> AlphaTds;
typedef CGAL::Delaunay_triangulation_3<ExactKernel, AlphaTds> AlphaDelaunay;
typedef CGAL::Alpha_shape_3<AlphaDelaunay> AlphaShape;

typedef CGAL::Dimension_tag<3> D;
typedef CGAL::Search_traits_adapter<Point3Vertex,
    CGAL::Nth_of_tuple_property_map<0, Point3Vertex>,
    CGAL::Search_traits_3<Kernel>>
    SearchTraits;

typedef CGAL::Kd_tree<SearchTraits> KdTree;
typedef CGAL::K_neighbor_search<SearchTraits> KNeighbour;
typedef CGAL::Incremental_neighbor_search<SearchTraits> IncrementalKNN;

}
