#ifndef EASY3D_VIEWER_BACK_H
#define EASY3D_VIEWER_BACK_H

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
