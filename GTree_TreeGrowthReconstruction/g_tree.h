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

#ifndef TREEGROWTHMODELLING_G_TREE_H
#define TREEGROWTHMODELLING_G_TREE_H


#include "skeleton_grow.h" // Lobe already imported in skeleton
#include "growth_model_region.h"


class GTree {
public:
    GTree();
    ~GTree();

    /// Store skeletons of timestamps.
    bool add_ts_skeletons(std::vector<GSkeleton*> skels);
    /// Reconstruct point cloud of all timestamps merged together.
    bool construct_merged_cloud();
    /// Store skeleton of merged model.
    bool add_merged_skeleton(GSkeleton* skel);

    // representation
    /// Detect edge correspondence between merged main and all timestamp graphs.
    bool compute_correspondence();
    /// Find the main edges of the merged main skeleton.
    bool compute_main_edges();
    /// Control function for building corresponding timestamp skeletons.
    bool constrain_ts_skeletons();
    /// Find convex hull of timestamp lobe clusters.
    bool compute_lobe_hulls();
    /// Add the grown branching structure inside the lobes to the corresponding timestamp skeletons.
    bool add_grown_lobes_to_corr();

    // growing
    /// Execute region growing inside lobe hulls.
    bool grow_lobes();
    /// For all timestamps minus the last one, compute geometric correspondences.
    bool interpolate();

    // utility
    easy3d::vec3 get_closest_point_on_line(easy3d::vec3 point, easy3d::vec3 line_left, easy3d::vec3 line_right);

    // getters
    easy3d::PointCloud* get_merged_cloud() const {return cloud_merged_;}
    GSkeleton* get_merged_skeleton() const {return skeleton_merged_;}
    int get_num_timestamps() const {return skeletons_.size();}
    std::vector<easy3d::SurfaceMesh*> get_lobe_meshes() const {return lobe_meshes_;}
    std::vector<std::pair<int, std::vector<Lobe*> > > get_lobes() const {return lobes_;}
    std::vector<easy3d::SurfaceMesh*> get_branches_meshes() const {return branches_meshes_;}

private:
    std::vector<std::pair<int, GSkeleton*> > skeletons_;    // store skeletons of all timestamps
    std::vector<std::pair<int, std::vector<Lobe*> > > lobes_;   // store lobe sets per timestamp
    std::vector<easy3d::SurfaceMesh*> lobe_meshes_; // vector of meshes per ts lobe set
    std::vector<easy3d::SurfaceMesh*> branches_meshes_; // storing meshes of ts reconstructed branches

    // store merged
    easy3d::PointCloud* cloud_merged_;
    GSkeleton* skeleton_merged_;
    easy3d::SurfaceMesh* branches_merged_;
    Vector3D* kd_points_merged_;
    KdTree* kdtree_merged_;

};


#endif //TREEGROWTHMODELLING_G_TREE_H
