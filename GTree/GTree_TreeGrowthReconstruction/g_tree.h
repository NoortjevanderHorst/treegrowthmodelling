//
// Created by noort on 23/01/2022.
//

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
