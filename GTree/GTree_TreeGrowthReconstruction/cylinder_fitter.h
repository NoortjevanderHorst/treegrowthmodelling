//
// Created by noort on 02/05/2022.
//

#ifndef TREEGROWTHMODELLING_CYLINDER_FITTER_H
#define TREEGROWTHMODELLING_CYLINDER_FITTER_H


#include <easy3d/core/types.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/principal_axes.h>
#include <3rd_party/kd_tree/Vector3D.h>
#include <3rd_party/kd_tree/KdTree.h>

#include "graph_gt.h"
#include "cylinder.h"


//define the graph path to smooth branches
typedef std::vector<VertexDescriptorGTGraph> Path;


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

    /// setters
    void set_skeleton(GraphGT* graph) { skeleton_ = *graph; }
    void set_root(VertexDescriptorGTGraph vert) { RootV_ = vert;}

    /// control
    bool reconstruct_branches(easy3d::SurfaceMesh* mesh_result);

    /// additional
    void obtain_initial_radius(const easy3d::PointCloud *cloud);
    void construct_kd_tree(const easy3d::PointCloud *cloud);
    void compute_length_of_subtree(GraphGT *i_Graph, VertexDescriptorGTGraph i_dVertex);
    void compute_graph_edges_weight(GraphGT *i_Graph);
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

    GraphGT skeleton_;
    GraphGT skeleton_smooth_;

    VertexDescriptorGTGraph RootV_;
    Vector3D RootPos_;
    double TrunkRadius_;
    double TreeHeight_;
    double BoundingDistance_;

};


#endif //TREEGROWTHMODELLING_CYLINDER_FITTER_H
