//
// Created by noort on 02/05/2022.
//

#ifndef TREEGROWTHMODELLING_CYLINDER_FITTER_H
#define TREEGROWTHMODELLING_CYLINDER_FITTER_H


#include <easy3d/core/types.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/graph.h>
#include <easy3d/core/principal_axes.h>
#include <3rd_party/kd_tree/Vector3D.h>
#include <3rd_party/kd_tree/KdTree.h>

#include "graph_boost.h"
#include "cylinder.h"


//define the graph path to smooth branches
typedef std::vector<VertexDescriptorGraphB> Path;


class CylFit {
public:
    CylFit();
    ~CylFit();

    struct Branch {
        std::vector<easy3d::vec3> points;
        std::vector<double>       radii;
    };
    std::vector<Branch> get_branches_parameters() const;

    /// getters
    double get_trunk_radius() { return TrunkRadius_; }

    /// setters
    void set_skeleton(GraphB graph);
    void set_root(VertexDescriptorGraphB vert) { RootV_ = vert; }
    void set_trunk_radius(double radius) { TrunkRadius_ = radius; }

    /// control
    bool reconstruct_branches(easy3d::SurfaceMesh* mesh_result);

    /// additional
    void obtain_initial_radius(const GraphB *graph);    // for timestamp graphs (trunk radius estimate)
    void obtain_initial_radius_inter(double r_base, double r_target, int step, int nr_steps);   // for intermediary graphs (interpolation)
    void construct_kd_tree(const GraphB *graph);
    void compute_length_of_subtree(GraphB *i_Graph, VertexDescriptorGraphB i_dVertex);
    void compute_graph_edges_weight(GraphB *i_Graph);
    void compute_all_edges_radius(double trunkRadius);

    /// structural
    bool compute_branch_radius();
    void assign_points_to_edges();
    void fit_trunk();

    /// smoothing
    bool smooth_skeleton();
    void get_graph_for_smooth(std::vector<Path> &pathList);

    /// mesh
    bool extract_branch_surfaces(easy3d::SurfaceMesh* mesh_result);
    void add_generalized_cylinder_to_model(easy3d::SurfaceMesh *mesh, const Branch &branch, int slices);


private:
    // Kd-tree index
    Vector3D* Points_;
    KdTree* KDtree_;

    GraphB skeleton_;
    GraphB skeleton_smooth_;

    VertexDescriptorGraphB RootV_;
    Vector3D RootPos_;
    double TrunkRadius_;
    double TreeHeight_;
    double BoundingDistance_;

};


#endif //TREEGROWTHMODELLING_CYLINDER_FITTER_H
