//
// Created by noort on 23/01/2022.
//

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


// The original easy3d viewer enables multiple models. In TreeViewer, we allow only three:
//    - model #1: the point cloud
//    - model #2: the 3D model of tree branches
//    - model #3: the 3D model of leaves

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

    //overload the key press event to conduct the modelling process
    bool key_press_event(int key, int modifiers) override;

    /*-------------------------------------------------------------*/
    /*----------------------------IN-------------------------------*/
    /*-------------------------------------------------------------*/

    bool open() override;
    bool open_mesh() override;
    bool open_multiple() override;
    std::string open_multitemporal() override;
    bool complete_multitemporal_import(std::vector<std::string> filenames) override;
    bool save() const override;
    bool save_batch() override;

    /*-------------------------------------------------------------*/
    /*---------------------------OUT-------------------------------*/
    /*-------------------------------------------------------------*/

    void export_skeleton() const override;
    void export_lobes() const override;
    void export_main() const override;
    void export_graph(const GraphGT& skeleton, easy3d::vec3 translation, const std::string& initial_name) const;
    void export_graph_indices(const GraphGT& skeleton, easy3d::vec3 translation, const std::string& initial_name) const;
    void export_branches_corr() const override;
    void export_branches_ts() const override;
    void export_correspondences() const override;

    /*-------------------------------------------------------------*/
    /*-----------------------RECONSTRUCT---------------------------*/
    /*-------------------------------------------------------------*/

    bool reconstruct_skeleton() override;
    bool add_leaves() override;
    bool batch_process() override;
    bool reconstruct_multitemporal() override;
    bool add_merged_cloud() override;
    bool model_correspondence() override;
    bool model_growth() override;
    bool reconstruct_geometry() override;
    bool model_interpolation() override;
    bool reconstruct_all() override;

    /*-------------------------------------------------------------*/
    /*-------------------------DRAWING-----------------------------*/
    /*-------------------------------------------------------------*/

    /** Visualisation common parameters
     * @param type current type of skeleton graph.
     * @param ts_index current timestamp index.
     * @param show whether this visualisation should be turned off or on.
     * @param default_color base color of the vertices/edges if not visualised (show is false).
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

    /// Timestamp model access
    easy3d::PointCloud* cloud_ts(int time_index) const;

    /// Timestamp skeleton graph access
    const GraphGT* skeleton_ts(SkeletonType type, int time_index) const;

    /// Timestamp lobe access
    easy3d::SurfaceMesh* lobe_ts(int time_index) const;

    /// Timestamp branches mesh access
    easy3d::SurfaceMesh* branches_ts(int time_index, int type_idx) const;

    /// Timestamp visualisation methods controller
    bool ts_visualisation(int ts_index, int item_index, bool show, int skeleton_type, std::vector<ImVec4> colors) override;

    /// Timestamp visualisation default color controller
    bool ts_change_colors(int ts_index, int item_index, ImVec4 color) override;

    /// Inter-timestamp correspondence visualisation controller
    bool inter_visualisation(int item_index, bool show) override;

    /// Compute color based on intensity
    easy3d::vec3 colormap(float intensity);

    /// Extend viewer drawing with shadowing
    void draw() const override;

    /// Clean attributes, reset viewer object
    void cleanup() override;

private:
    easy3d::SoftShadow*     shadow_;
//    GSkeleton*              skeleton_;
    std::vector<GSkeleton*> trees_;
    GTree* gtree_;
};


#endif //TREEGROWTHMODELLING_GROWTH_VIEWER_H
