#pragma once

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Regular_triangulation_3.h>
#include <CGAL/K_neighbor_search.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/Alpha_shape_3.h>
#include <CGAL/Alpha_shape_vertex_base_3.h>
#include <CGAL/Alpha_shape_cell_base_3.h>
#include <groot/cgal.hpp>
#include <groot/plant_graph.hpp>
#include <tuple>

namespace groot::cgal {

typedef std::tuple<Point_3, Vertex> Point3Vertex;

typedef CGAL::Triangulation_vertex_base_with_info_3<Vertex, Kernel> DelaunayVertex;
typedef CGAL::Triangulation_data_structure_3<DelaunayVertex> Tds;
typedef CGAL::Delaunay_triangulation_3<Kernel, Tds> Delaunay;

typedef CGAL::Alpha_shape_vertex_base_3<Kernel, CGAL::Triangulation_vertex_base_with_info_3<Vertex, Kernel>> AlphaVertex;
typedef CGAL::Triangulation_data_structure_3<AlphaVertex, CGAL::Alpha_shape_cell_base_3<Kernel>> AlphaTds;
typedef CGAL::Delaunay_triangulation_3<Kernel, AlphaTds> AlphaDelaunay;
typedef CGAL::Alpha_shape_3<AlphaDelaunay> AlphaShape;

typedef CGAL::Dimension_tag<3> D;
typedef CGAL::Search_traits_adapter<Point3Vertex,
    CGAL::Nth_of_tuple_property_map<0, Point3Vertex>,
    CGAL::Search_traits_3<Kernel>>
    SearchTraits;

typedef CGAL::Kd_tree<SearchTraits> KdTree;
typedef CGAL::K_neighbor_search<SearchTraits> KNeighbour;



}
