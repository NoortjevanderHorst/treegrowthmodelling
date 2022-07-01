//
// Created by noort on 23/01/2022.
//

#ifndef TREEGROWTHMODELLING_SKELETON_GROW_H
#define TREEGROWTHMODELLING_SKELETON_GROW_H


#include <easy3d/core/types.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/principal_axes.h>
#include <3rd_party/kd_tree/Vector3D.h>
#include <3rd_party/kd_tree/KdTree.h>
#include <3rd_party/tetgen/tetgen.h>

#include <fstream>
#include <utility>
#include <chrono>

#include "graph_gt.h"
#include "lobe.h"
#include "cylinder_fitter.h"


// CGAL geometry for convex hull
typedef CGAL::Exact_predicates_inexact_constructions_kernel K_cgal;
typedef K_cgal::Point_3 Point_3_cgal;
typedef CGAL::Surface_mesh<Point_3_cgal> Surface_mesh_cgal;
typedef K_cgal::Line_3 Line_3_cgal;


namespace easy3d {
    class PointCloud;
    class SurfaceMesh;
}


class GSkeleton {
public:
    GSkeleton();
    ~GSkeleton();

    // properties
    GraphGT graph_skel;

    // getters
    const GraphGT& get_delaunay() const { return delaunay_; }
    const GraphGT& get_mst() const { return mst_; }
    const GraphGT& get_simplified() const { return simplified_; }
    const GraphGT& get_corresponding() const { return corresponding_; }
    const GraphGT& get_ts_main() const { return ts_main_; }

    easy3d::vec3 get_translation() const { return translation_; }
    int get_max_branching_level() const { return max_branching_level_; }
    int get_max_weight() const { return max_weight_; }
    int get_num_lobes() const { return num_lobes_; }
    KdTree* get_kdtree() const { return kdtree_; }
    VertexDescriptorGTGraph get_root() const { return rootv_; }
    VertexDescriptorGTGraph get_root_corr() const { return rootv_corr_; }
    double get_max_skeleton_distance() const { return max_dist_skel_; }

    // setters
    void set_translation(easy3d::vec3 trans) { translation_ = trans; }
    void set_corresponding(GraphGT* graph) { corresponding_ = *graph; }   // only for storing main skeleton of merged (graph is copied!)

    // control
    bool reconstruct_skeleton(const easy3d::PointCloud *cloud, easy3d::SurfaceMesh* result);
    bool reconstruct_branches(const easy3d::PointCloud *cloud, easy3d::SurfaceMesh* mesh_result, GraphGT* graph, VertexDescriptorGTGraph rootv);

    // build structures
    bool load_points(const easy3d::PointCloud* cloud);
    bool build_delaunay();
    bool build_mst();
    bool build_KdTree(GraphGT graph);
    bool build_simplified();
    bool build_corresponding(GraphGT graph_corr, VertexDescriptorGTGraph root_vert);   // timestamp correspondence skeleton
    bool build_corresponding_merged();  // merged correspondence skeleton (correspondence base)
    bool detect_lobe_points(std::vector<Lobe*>& lobes);  // flag all lobe vertices & their connectors, build set of Lobes
    void build_ts_main(GraphGT graph_corr);
    void intermediary_skeleton_distance(GraphGT graph_corr);

    // compute structural information
    bool compute_branching_levels(GraphGT& graph);
    bool compute_importance(GraphGT& graph);
    bool compute_normals_pointcloud(GraphGT& graph);
    bool find_all_lobe_nodes(GraphGT& graph);

    // utility
    void clean_graph(GraphGT& graph);
    void simplify_branch(GraphGT& graph,
                         std::vector<VertexDescriptorGTGraph> branch,
                         bool allow_tip_removal = true,
                         double min_length = 0.1,
                         double epsilon = 0.5);
    bool simplify_line(GraphGT& graph,
                       std::vector<VertexDescriptorGTGraph> line,
                       bool allow_tip_removal = true,
                       double epsilon = 0.5);
    double distance_point_to_line(easy3d::vec3 point, easy3d::vec3 line_left, easy3d::vec3 line_right);
    VertexDescriptorGTGraph find_main_bifurcation();

    // export
    bool export_weights(const char *file_out);
    bool export_levels(const char *file_out);
    bool export_to_ply(const char *file_out);

private:
//    GraphGT skeleton_;
    std::vector<easy3d::vec3> pc_points_;
    GraphGT delaunay_;
    GraphGT mst_;
    GraphGT simplified_;
    GraphGT corresponding_;
    GraphGT ts_main_;

    // kd-tree
    Vector3D* kd_points_;
    KdTree* kdtree_;
    Vector3D* kd_points_corr_;
    KdTree* kdtree_corr_;

    easy3d::vec3 translation_;  // translation done by viewer to make root (0, 0, 0)
    VertexDescriptorGTGraph rootv_; // root vertex of the skeleton graph(s)
    VertexDescriptorGTGraph rootv_corr_; // root vertex after constraining with average skeleton
    // (so everything except corresponding_ should have the "rootv_" root)
    int max_branching_level_;
    int max_weight_;
    int num_lobes_; // total number of connected lobes
    double max_dist_skel_;  // maximum distance between ts and merged main skeleton edges

};


#endif //TREEGROWTHMODELLING_SKELETON_GROW_H
