//
// Created by noort on 23/01/2022.
//

#ifndef TREEGROWTHMODELLING_LOBE_H
#define TREEGROWTHMODELLING_LOBE_H


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Polygon_mesh_processing/measure.h>

#include <easy3d/core/types.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/core/surface_mesh.h>

#include "graph_gt.h"


// CGAL geometry for convex hull
typedef CGAL::Exact_predicates_inexact_constructions_kernel K_cgal;
typedef K_cgal::Point_3 Point_3_cgal;
typedef CGAL::Surface_mesh<Point_3_cgal> Surface_mesh_cgal;


namespace easy3d {
    class PointCloud;
    class SurfaceMesh;
}


class Lobe {
public:
    Lobe();
    ~Lobe();

    // getters
    const GraphGT& get_subgraph() const { return graph_; }
    VertexDescriptorGTGraph get_connector_node() const { return connector_node_; }
    int get_index() const { return lobe_index_; }
    const Surface_mesh_cgal& get_convex_hull() const { return convex_hull_; }
    const easy3d::PointCloud& get_pointcloud() const { return points_easy3d_; }

    // setters
    void set_connector_node(VertexDescriptorGTGraph vert) { connector_node_ = vert; }
    void set_index(int idx) { lobe_index_ = idx; }
    void set_skeleton(GraphGT graph) { graph_ = graph; }

    bool build_lobe_hulls(easy3d::SurfaceMesh *mesh);

private:
    GraphGT graph_; // subgraph of lobe
    VertexDescriptorGTGraph connector_node_;  // vertex index from GSkeleton->corresponding_ graph that this lobe connects to
    int lobe_index_;    // index number of lobe

    easy3d::PointCloud points_easy3d_;

    std::vector<Point_3_cgal> points_cgal_;
    Surface_mesh_cgal convex_hull_;

    // todo: perhaps kd tree of vertices will be useful?



};


#endif //TREEGROWTHMODELLING_LOBE_H
