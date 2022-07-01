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

#ifndef TREEGROWTHMODELLING_GROWTH_VIEWER_H
#define TREEGROWTHMODELLING_GROWTH_VIEWER_H


#include "growth_viewer_imgui.h"

#include <3rd_party/glfw/include/GLFW/glfw3.h>	// Include glfw3.h after our OpenGL definitions

#include <easy3d/util/dialogs.h>
#include <easy3d/util/file_system.h>
#include <easy3d/fileio/surface_mesh_io.h>
#include <easy3d/fileio/translator.h>

#include <easy3d/core/types.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/core/graph.h>
#include <easy3d/core/model.h>

#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/shader_program.h>
#include <easy3d/renderer/shader_manager.h>
#include <easy3d/renderer/camera.h>
#include <easy3d/renderer/setting.h>
#include <easy3d/renderer/soft_shadow.h>

#include <vector>
#include <chrono>

#include "g_tree.h"


// The original easy3d viewer enables multiple models. With the number of input timestamps as n,
// the GTree Viewer has:
//    - model [0, n - 1]: the input point clouds + their skeleton graphs
//    - model [n]: the merged main point cloud + skeleton graphs
//    - model [n + 1, 2n]: the 3D mesh model of tree lobes
//    - model [2n + 1, (2n + 1) + 2n]: the 3D mesh model of the tree branches,
//    respectively for the corresponding and timestamp-specific skeletons
//    - model [4n + 1]: the 3D mesh model of the tree branches of the merged main skeleton
// ------------- example with 3 timestamps:
// models: ts0 - ts1 - ts2 - merged cloud
//          0     1     2          3
//       - lobes 0 - lobes 1 - lobes 2
//            4         5         6
//       - branches corr 0 - branches ts 0
//               7              8
//       - branches corr 1 - branches ts 1
//               9              10
//       - branches corr 2 - branches ts 2 - branches merged
//              11              12               13

// A very good tutorial for imgui:
// https://eliasdaler.github.io/using-imgui-with-sfml-pt1/
// https://eliasdaler.github.io/using-imgui-with-sfml-pt2/


class Skeleton;

namespace easy3d {
    class PointCloud;
    class SurfaceMesh;
    class SoftShadow;
}

class GrowthViewer : public easy3d::ViewerImGuiGrow {
public:
    GrowthViewer();
    ~GrowthViewer() override;

protected:
//    virtual std::string usage() const override;

    /*-------------------------------------------------------------*/
    /*----------------------------IN-------------------------------*/
    /*-------------------------------------------------------------*/

    /// Import one or multiple .xyz point cloud files.
    bool open() override;
    /// Import one or multiple .obj mesh files.
    bool open_mesh() override;
    /// Get location/name of a multi-temporal .xyz point cloud file to open.
    std::string open_multitemporal() override;
    /// Import multi-temporal .xyz point cloud files using provided file names/paths.
    bool complete_multitemporal_import(std::vector<std::string> filenames) override;

    /*-------------------------------------------------------------*/
    /*---------------------------OUT-------------------------------*/
    /*-------------------------------------------------------------*/

    /// Export the current merged and merged main skeletons (.ply).
    void export_skeleton() const override;

    /// Export the timestamp-specific main skeletons (.ply).
    void export_main() const override;

    /// Custom method for saving the supplied skeleton graph to a file (.ply).
    void export_graph(const GraphGT& skeleton, easy3d::vec3 translation, const std::string& initial_name) const;

    /** Custom method for saving the supplied mesh to a file (.obj)
     * Note: mesh is not translated back to original position (would result in inaccuracies in geometry)!
     */
    void export_mesh(const easy3d::SurfaceMesh* mesh, const std::string& initial_name) const;

    /// Save the 3D surface mesh of the lobe hulls of each timestamp (.obj).
    void export_lobes() const override;

    /// Save the 3D surface mesh of the branches of the merged main + lobe branches per timestamp (.obj).
    void export_branches_corr() const override;

    /// Save the 3D surface mesh of the branches of the timestamp-specific main structure (.obj).
    void export_branches_ts() const override;

    /** Save the timestamp-specific main skeleton as a graph (.ply).
     * Save the vertex and edge transformation operations between each timestamp and the next as correspondence files (.csv).
     * These files can be used by the GTree interpolator to compute and visualise the tree's structure changing (growing) between timestamps.
     */
    void export_correspondences() const override;

    /*-------------------------------------------------------------*/
    /*-----------------------RECONSTRUCT---------------------------*/
    /*-------------------------------------------------------------*/

    /// Generate skeleton graph reconstructions of all timestamps.
    bool reconstruct_multitemporal() override;

    /// Combine all timestamps into a merged point cloud.
    bool add_merged_cloud() override;

    /** Reconstruct corresponding main structures:
     * 1) Reconstruct main merged skeleton.
     * 2) Find correspondences between timestamp edges.
     * 3)Constrain timestamp-specific main structure with merged main structure.
     * 4) Generate lobes.
     * @return
     */
    bool model_correspondence() override;

    /// Perform region growing to reconstruct branching structure in lobes of all timestamps.
    bool model_growth() override;

    /// Construct geometry for visualisation of grown multi-temporal structures (lobe mesh, branches mesh).
    bool reconstruct_geometry() override;

    /// Compute necessary correspondence operations on the main graph of a timestamp to transform it into the next.
    bool model_interpolation() override;

    /// General control method to execute all growth reconstruction steps at once.
    bool reconstruct_all() override;

    /*-------------------------------------------------------------*/
    /*-------------------------DRAWING-----------------------------*/
    /*-------------------------------------------------------------*/

    /* Common visualisation parameters
     * - type:          current type of skeleton graph.
     * - ts_index:      current timestamp index.
     * - show:          whether this visualisation should be turned off or on.
     * - default_color: base color of the vertices/edges if not visualised (show is false).
     */

    /// Visualize weight property of the current skeleton's vertices.
    bool update_importance_visuals_vertices(SkeletonType type, int ts_index, bool show, ImVec4 default_color);

    /// Draw eigenvector of vertex neighbourhood in MST graph of current skeleton.
    bool create_normals_drawable(int ts_index);

    /// Draw edges of current skeleton graph.
    bool create_skeleton_drawable(SkeletonType type, int skeleton_index, ImVec4 default_color);

    /// Visualize branching level property of the current skeleton's edges.
    bool update_importance_visuals_edges(SkeletonType type, int ts_index, bool show, ImVec4 default_color);

    /**
     * Visualize number of timestamp edges the merged main skeleton corresponds with.
     * Only applicable to merged main!
     * Color scale: red=1, green=2, blue=3 correspondences.
     */
    bool update_correspondence_visuals(SkeletonType type, int ts_index, bool show, ImVec4 default_color);

    /// Draw skeleton sub-graph that was detected as the main structure
    bool create_main_skeleton_drawable(SkeletonType type, int ts_index);

    /// Visualize the vertices of the current skeleton graph that were detected as the main structure.
    bool show_vertex_correspondence(SkeletonType type, int ts_index, bool show, ImVec4 default_color);

    /// Visualize the edges of the current skeleton graph that were detected as the main structure.
    bool show_edge_correspondence(SkeletonType type, int ts_index, bool show, ImVec4 default_color);

    /// Visualize the bifurcation points in the current skeleton graph.
    bool show_bifur(SkeletonType type, int ts_index, bool show, ImVec4 default_color);

    /// Visualize the lobe points in the current skeleton graph.
    bool show_lobes(SkeletonType type, int ts_index, bool show, ImVec4 default_color);

    /// Draw skeleton graph inside lobes
    bool create_lobe_skeleton_drawable(SkeletonType type, int ts_index);

    /// Visualize (Euclidean) distance of current skeleton graph to merged main skeleton.
    bool show_skeleton_distance(SkeletonType type, int ts_index, bool show, ImVec4 default_color);

    /**
     * Visualize correspondences between the given base timestamp and the next timestamp (target).
     * Draws base in green, target in red, and blue lines between corresponding vertices.
     * @param item_index base graph index
     */
    bool create_inter_skeleton_drawable(int item_index);

    /*-------------------------------------------------------------*/
    /*--------------------------ACCESS-----------------------------*/
    /*-------------------------------------------------------------*/

    /// Timestamp input point cloud model.
    easy3d::PointCloud* cloud_ts(int time_index) const;

    /// Access the different types of graphs within skeletons (from trees_).
    const GraphGT* skeleton_ts(SkeletonType type, int time_index) const;

    /// Timestamp 3D lobe convex hull mesh.
    easy3d::SurfaceMesh* lobe_ts(int time_index) const;

    /// Timestamp 3D branches cylinder mesh.
    easy3d::SurfaceMesh* branches_ts(int time_index, int type_idx) const;

    /// Timestamp visualisation methods controller.
    bool ts_visualisation(int ts_index, int item_index, bool show, int skeleton_type, std::vector<ImVec4> colors) override;

    /// Timestamp visualisation default color controller.
    bool ts_change_colors(int ts_index, int item_index, ImVec4 color) override;

    /// Inter-timestamp correspondence visualisation controller.
    bool inter_visualisation(int item_index, bool show) override;

    /// Compute color based on intensity.
    easy3d::vec3 colormap(float intensity);

    /// Additional functions linked to key presses.
    bool key_press_event(int key, int modifiers) override;

    /// Extend viewer drawing with shadowing.
    void draw() const override;

    /// Clean attributes, reset viewer object.
    void cleanup() override;

private:
    easy3d::SoftShadow*     shadow_;
    std::vector<GSkeleton*> trees_;
    GTree* gtree_;
};


#endif //TREEGROWTHMODELLING_GROWTH_VIEWER_H
