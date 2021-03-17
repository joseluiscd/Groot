#include <boost/iterator/transform_iterator.hpp>
#include <groot/cylinder_marching.hpp>
#include <tbb/parallel_for.h>

namespace groot {

void compute_differential_quantities(cgal::Point_3* cloud, Curvature* q_out, size_t count, size_t k)
{
    KdTree kdtree;
    kdtree.insert(cloud, cloud + count);
    kdtree.build<CGAL::Parallel_tag>();

    cgal::Point_3 nn[k];
    tbb::parallel_for((size_t)0, count, (size_t)1, [&](size_t i) {
        KNeighbour knn(kdtree, cloud[i], k);

        Monge_via_jet_fitting monge_fitting;

        size_t u = 0;
        for (auto it = knn.begin(); it != knn.end(); ++it) {
            nn[u++] = it->first;
        }

        Monge_form f = monge_fitting(nn, nn+k, 2, 2);

        float r = 1.0f / f.principal_curvatures(0);
        cgal::Vector_3 n = f.normal_direction();
        cgal::Vector_3 d1 = f.maximal_principal_direction();
        cgal::Point_3 b = cloud[i] - r * n;

        q_out[i].curvature_center = b;
        q_out[i].direction = d1;
        q_out[i].radius = r;
    });
}

}
