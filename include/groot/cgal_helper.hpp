#pragma once

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <lemon/list_graph.h>

namespace cgal {

typedef CGAL::Simple_cartesian<double> Kernel;

typedef Kernel::Point_3 Point_3;
typedef Kernel::Segment_3 Segment_3;
typedef Kernel::Triangle_3 Triangle_3;

typedef CGAL::Triangulation_vertex_base_with_info_3<lemon::ListGraph::Node, Kernel> Vertex;
typedef CGAL::Triangulation_data_structure_3<Vertex> Tds;
typedef CGAL::Delaunay_triangulation_3<Kernel, Tds> Delaunay;

}
