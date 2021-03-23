#pragma once 

#include <CGAL/Cartesian.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/K_neighbor_search.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Search_traits.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <tuple>

namespace groot::cgal {

typedef CGAL::Simple_cartesian<float> Kernel;

typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;
typedef Kernel::Segment_3 Segment_3;
typedef Kernel::Triangle_3 Triangle_3;
typedef Kernel::Line_3 Line_3;

}

namespace groot {
    using namespace cgal;
}