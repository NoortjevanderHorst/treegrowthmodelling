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

    /*-------------------------------------------------------------*/
    /*-------------------------GETTERS-----------------------------*/
    /*-------------------------------------------------------------*/

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

    /*-------------------------------------------------------------*/
    /*-------------------------SETTERS-----------------------------*/
    /*-------------------------------------------------------------*/

    void set_translation(easy3d::vec3 trans) { translation_ = trans; }
    void set_corresponding(GraphGT* graph) { corresponding_ = *graph; }   // only for storing main skeleton of merged (graph is copied!)

    /*-------------------------------------------------------------*/
    /*-------------------------CONTROL-----------------------------*/
    /*-------------------------------------------------------------*/

    /// Reconstruct skeleton graph (skeletonization).
    bool reconstruct_skeleton(const easy3d::PointCloud *cloud, easy3d::SurfaceMesh* result);
    /// Reconstruct branch geometry (cylinder fitting).
    bool reconstruct_branches(const easy3d::PointCloud *cloud, easy3d::SurfaceMesh* mesh_result, GraphGT* graph, VertexDescriptorGTGraph rootv);

    /*-------------------------------------------------------------*/
    /*--------------------BUILD STRUCTURES-------------------------*/
    /*-------------------------------------------------------------*/

    /// Read in point cloud points.
    bool load_points(const easy3d::PointCloud* cloud);
    /// Compute Delaynay triangulation between input point cloud points.
    bool build_delaunay();
    /// Compute Minimum Spanning Tree from Delaunay triangulation graph.
    bool build_mst();
    /// Construct Kd_tree index of the graph's vertices.
    bool build_KdTree(GraphGT graph);
    /// Simplify MST reconstruction.
    bool build_simplified();
    /// Build timestamp correspondence skeleton.
    bool build_corresponding(GraphGT graph_corr, VertexDescriptorGTGraph root_vert);
    /// Build merged correspondence skeleton (correspondence base).
    bool build_corresponding_merged();
    /// Flag all lobe vertices & their connectors, build set of Lobes.
    bool detect_lobe_points(std::vector<Lobe*>& lobes);
    /// Connect & process timestamp main edges (flagged during building of ts correspondence).
    void build_ts_main(GraphGT graph_corr);
    /**
     * Process intermediary timestamp-specific main skeleton during skeleton correspondence computation process.
     * Enables visualisation/computation of differences between main skeletons (ts and merged).
     */
    void intermediary_skeleton_distance(GraphGT graph_corr);

    /*-------------------------------------------------------------*/
    /*--------------COMPUTE STRUCTURAL INFORMATION-----------------*/
    /*-------------------------------------------------------------*/

    /// Set branching level per edge and per vertex.
    bool compute_branching_levels(GraphGT& graph);
    /// Set importance weight per vertex.
    bool compute_importance(GraphGT& graph);
    /// Set vertex neighbourhood eigenvector per vertex.
    bool compute_normals_pointcloud(GraphGT& graph);
    /// Detect viable bifurcation points in current (main merged correspondence) skeleton.
    bool find_all_lobe_nodes(GraphGT& graph);

    /*-------------------------------------------------------------*/
    /*-------------------------UTILITY-----------------------------*/
    /*-------------------------------------------------------------*/

    /**
     * For deleting vertices from the graph.
     * First mark vertices to delete as v.clear_mark=true, then run clean_graph().
     */
    void clean_graph(GraphGT& graph);
    /**
     * Go through all edges connected to vertices in the branch vector, simplify (poly)lines.
     * Note: tips will still be removed if they are below min_length, even if allow_tip_removal=false!
     */
    void simplify_branch(GraphGT& graph,
                         std::vector<VertexDescriptorGTGraph> branch,
                         bool allow_tip_removal = true,
                         double min_length = 0.1,
                         double epsilon = 0.5);
    /// Douglas-Peucker line simplification for (poly)line represented by vector of vertices.
    bool simplify_line(GraphGT& graph,
                       std::vector<VertexDescriptorGTGraph> line,
                       bool allow_tip_removal = true,
                       double epsilon = 0.5);
    /// Euclidean 3D distance from a 3D point to a 3D line (given as coordinates).
    double distance_point_to_line(easy3d::vec3 point, easy3d::vec3 line_left, easy3d::vec3 line_right);
    /// Detect the first major vertex at which the tree's branching structure splits into multiple major branches.
    VertexDescriptorGTGraph find_main_bifurcation();

    /*-------------------------------------------------------------*/
    /*--------------------------EXPORT-----------------------------*/
    /*-------------------------------------------------------------*/

    /// Debug: export the MST graph with vertex weights to a .xyz point cloud file.
    bool export_weights(const char *file_out);
    /// Debug: export the MST graph with vertex branching levels to a .xyz point cloud file.
    bool export_levels(const char *file_out);
    /// Debug: export the MST graph to a .ply graph file.
    bool export_to_ply(const char *file_out);

private:
    /// input point cloud
    std::vector<easy3d::vec3> pc_points_;
    /// Delaunay triangulation graph
    GraphGT delaunay_;
    /// Minimum Spanning Tree graph
    GraphGT mst_;
    /// Simplified skeleton graph
    GraphGT simplified_;
    /// Merged main corresponding skeleton graph + lobe branches reconstruction
    GraphGT corresponding_;
    /// Original timestamp-specific skeleton graph reconstruction
    GraphGT ts_main_;

    /// Kd-tree index of input point cloud points
    KdTree* kdtree_;
    Vector3D* kd_points_;
    /// Kd-tree index of merged main point cloud points
    KdTree* kdtree_corr_;
    Vector3D* kd_points_corr_;

    /// translation done by viewer to make root (0, 0, 0)
    easy3d::vec3 translation_;
    /// root vertex of the skeleton graph(s)
    VertexDescriptorGTGraph rootv_;
    /// root vertex after constraining with average skeleton (so everything except corresponding_ should have the "rootv_" root)
    VertexDescriptorGTGraph rootv_corr_;
    /// maximum branching level found (vertex property)
    int max_branching_level_;
    /// maximum vertex weight found
    int max_weight_;
    /// total number of connected lobes
    int num_lobes_;
    /// maximum distance between ts and merged main skeleton edges
    double max_dist_skel_;

};


#endif //TREEGROWTHMODELLING_SKELETON_GROW_H
