/*
*	Copyright (C) 2022 by
*       Noortje van der Horst (noortje.v.d.horst1@gmail.com)
*       Liangliang Nan (liangliang.nan@gmail.com)
*       3D Geoinformation, TU Delft, https://3d.bk.tudelft.nl
*
*	This file is part of GTree, which implements the 3D tree
*   reconstruction and growth modelling method described in the following thesis:
*   -------------------------------------------------------------------------------------
*       Noortje van der Horst (2022).
*       Procedural Modelling of Tree Growth Using Multi-temporal Point Clouds.
*       Delft University of Technology.
*       URL: http://resolver.tudelft.nl/uuid:d284c33a-7297-4509-81e1-e183ed6cca3c
*   -------------------------------------------------------------------------------------
*   Please consider citing the above thesis if you use the code/program (or part of it).
*
*   GTree is based on the works of Easy3D and AdTree:
*   - Easy3D: Nan, L. (2021).
*       Easy3D: a lightweight, easy-to-use, and efficient C++ library for processing and rendering 3D data.
*       Journal of Open Source Software, 6(64), 3255.
*   - AdTree: Du, S., Lindenbergh, R., Ledoux, H., Stoter, J., & Nan, L. (2019).
*       AdTree: accurate, detailed, and automatic modelling of laser-scanned trees.
*       Remote Sensing, 11(18), 2074.
*
*	GTree is free software; you can redistribute it and/or modify
*	it under the terms of the GNU General Public License Version 3
*	as published by the Free Software Foundation.
*
*	GTree is distributed in the hope that it will be useful,
*	but WITHOUT ANY WARRANTY; without even the implied warranty of
*	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*	GNU General Public License for more details.
*
*	You should have received a copy of the GNU General Public License
*	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TREEGROWTHMODELLING_CYLINDER_FITTER_H
#define TREEGROWTHMODELLING_CYLINDER_FITTER_H


#include <easy3d/core/types.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/principal_axes.h>
#include <3rd_party/kdtree/ETH_Kd_Tree/vector3D.h>
#include <3rd_party/kdtree/ETH_Kd_Tree/kdTree.h>

#include "graph_gt.h"
#include "cylinder.h"

using kdtree::KdTree;

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
