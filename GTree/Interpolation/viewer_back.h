//
// Created by noort on 12/05/2022.
//

#ifndef EASY3D_VIEWER_BACK_H
#define EASY3D_VIEWER_BACK_H


#ifndef ADTREE_TREE_VIEWER_H
#define ADTREE_TREE_VIEWER_H

/*
*	Copyright (C) 2019 by
*       Shenglan Du (dushenglan940128@163.com)
*       Liangliang Nan (liangliang.nan@gmail.com)
*       3D Geoinformation, TU Delft, https://3d.bk.tudelft.nl
*
*	This file is part of AdTree, which implements the 3D tree
*   reconstruction method described in the following paper:
*   -------------------------------------------------------------------------------------
*       Shenglan Du, Roderik Lindenbergh, Hugo Ledoux, Jantien Stoter, and Liangliang Nan.
*       AdTree: Accurate, Detailed, and Automatic Modeling of Laser-Scanned Trees.
*       Remote Sensing. 2019, 11(18), 2074.
*   -------------------------------------------------------------------------------------
*   Please consider citing the above paper if you use the code/program (or part of it).
*
*	AdTree is free software; you can redistribute it and/or modify
*	it under the terms of the GNU General Public License Version 3
*	as published by the Free Software Foundation.
*
*	AdTree is distributed in the hope that it will be useful,
*	but WITHOUT ANY WARRANTY; without even the implied warranty of
*	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*	GNU General Public License for more details.
*
*	You should have received a copy of the GNU General Public License
*	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/


#include "viewer_front.h"
#include "interpolator.h" // has graph class import
#include "cylinder_fitter.h"

#include <iostream>
#include <chrono>
#include <vector>

#include <3rd_party/imgui/misc/fonts/imgui_fonts_droid_sans.h>
#include <3rd_party/imgui/imgui.h>
#include <3rd_party/imgui/backends/imgui_impl_glfw.h>
#include <3rd_party/imgui/backends/imgui_impl_opengl3.h>
#include <3rd_party/glfw/include/GLFW/glfw3.h>

#include <easy3d/util/dialogs.h>
#include <easy3d/util/file_system.h>
#include <easy3d/fileio/surface_mesh_io.h>
#include <easy3d/renderer/camera.h>
#include <easy3d/core/types.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/core/graph.h>
#include <easy3d/core/model.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_triangles.h>

/*#include <easy3d/renderer/opengl.h>        // Initialize with glewInit()
#include <3rd_party/glfw/include/GLFW/glfw3.h>    // Include glfw3.h after our OpenGL definitions

#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/graph.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/core/poly_mesh.h>
#include <easy3d/renderer/shader_program.h>
#include <easy3d/renderer/shader_manager.h>
#include <easy3d/renderer/transform.h>
#include <easy3d/renderer/shapes.h>
#include <easy3d/renderer/camera.h>
#include <easy3d/renderer/manipulated_camera_frame.h>
#include <easy3d/renderer/key_frame_interpolator.h>
#include <easy3d/renderer/framebuffer_object.h>
#include <easy3d/renderer/opengl_error.h>
#include <easy3d/renderer/setting.h>
#include <easy3d/renderer/text_renderer.h>
#include <easy3d/renderer/texture_manager.h>
#include <easy3d/fileio/resources.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/fileio/graph_io.h>
#include <easy3d/fileio/surface_mesh_io.h>
#include <easy3d/fileio/poly_mesh_io.h>
#include <easy3d/fileio/ply_reader_writer.h>
#include <easy3d/fileio/point_cloud_io_ptx.h>
#include <easy3d/util/dialogs.h>
#include <easy3d/util/file_system.h>
#include <easy3d/util/logging.h>
#include <easy3d/util/timer.h>
#include <easy3d/util/string.h>
#include <easy3d/renderer/manipulator.h>*/


namespace easy3d {
    class PointCloud;
    class SurfaceMesh;
    class SoftShadow;
}


// The original easy3d viewer enbales multiple models. In TreeViewer, we allow only three:
//    - model #1: the point cloud
//    - model #2: the 3D model of tree branches
//    - model #3: the 3D model of leaves

class ViewerB : public easy3d::ViewerF
{
public:
    ViewerB();
    ~ViewerB() override;

protected:

    virtual std::string usage() const override;
    bool key_press_event(int key, int modifiers) override;
    void cleanup() override;

//    bool open_corresponding() override;
    bool save() const override;
    bool save_interpolation() override;

    bool open_correspondence() override;
    bool open_ts_graphs() override;
    bool open() override;

    void compute_correspondence() override;
    bool reconstruct_geometry() override;

    easy3d::Graph* graph_ts(int time_index) const;
    easy3d::SurfaceMesh* branches(int time_index) const;

private:
    Interpolator* interp_;
    int inter_idx_ = 0;
    easy3d::vec3 trans_ = {0, 0, 0};    // translation for all models (set with first loaded ts graph)
    bool show_branches_ = false;

};

#endif



#endif //EASY3D_VIEWER_BACK_H
