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

/**
 * Definition of GUI/renderer.
 * Based on the easy3d viewer and imgui.
 */

#ifndef TREEGROWTHMODELLING_GROWTH_VIEWER_IMGUI_H
#define TREEGROWTHMODELLING_GROWTH_VIEWER_IMGUI_H


#include <easy3d/viewer/viewer.h>

#include <3rd_party/imgui/misc/fonts/imgui_fonts_droid_sans.h>
#include <3rd_party/imgui/imgui.h>
#include <3rd_party/imgui/backends/imgui_impl_glfw.h>
#include <3rd_party/imgui/backends/imgui_impl_opengl3.h>
#include <3rd_party/glfw/include/GLFW/glfw3.h>

#include <iostream>
#include <sstream>


struct ImGuiContext;


namespace easy3d {

    class ViewerImGuiGrow : public Viewer
    {
    public:
        ViewerImGuiGrow(
                const std::string& title = "Easy3D ImGui Viewer Grow",
                int samples = 4,
                int gl_major = 3,
                int gl_minor = 2,
                bool full_screen = false,
                bool resizable = true,
                int depth_bits = 24,
                int stencil_bits = 8
        );

    protected:

        // imgui plugins
        void init() override;

        // draw the widgets
        void pre_draw() override;

        //  the widgets
        void post_draw() override;

        void cleanup() override;

        void post_resize(int w, int h) override;

        bool callback_event_cursor_pos(double x, double y) override;
        bool callback_event_mouse_button(int button, int action, int modifiers) override;
        bool callback_event_keyboard(int key, int action, int modifiers) override;
        bool callback_event_character(unsigned int codepoint) override;
        bool callback_event_scroll(double dx, double dy) override;

        void draw_menu_file();
        void draw_menu_view();
        void draw_menu_reconstruction();
        void draw_panel();
        void draw_multitemp_import(bool* is_open);

        //create the line drawable for skeleton graphs
        enum SkeletonType {
            ST_DELAUNAY = 0,
            ST_MST,
            ST_SIMPLIFIED,
            ST_CORRESPONDING,
            ST_TS_MAIN
        };

        virtual bool ts_visualisation(int ts_index, int item_index, bool show, int skeleton_type, std::vector<ImVec4> colors) = 0;
        virtual bool ts_change_colors(int ts_index, int item_index, ImVec4 color) = 0;
        virtual bool inter_visualisation(int item_index, bool show) = 0;

        virtual bool open_mesh() = 0;
        virtual std::string open_multitemporal() = 0;
        virtual bool complete_multitemporal_import(std::vector<std::string> filenames) = 0;

        virtual void export_skeleton() const = 0;
        virtual void export_lobes() const = 0;
        virtual void export_main() const = 0;
        virtual void export_branches_corr() const = 0;
        virtual void export_branches_ts() const = 0;
        virtual void export_correspondences() const = 0;

        virtual bool reconstruct_multitemporal() = 0;
        virtual bool add_merged_cloud() = 0;
        virtual bool model_correspondence() = 0;
        virtual bool model_growth() = 0;
        virtual bool reconstruct_geometry() = 0;
        virtual bool model_interpolation() = 0;
        virtual bool reconstruct_all() = 0;

    protected:
        // Ratio between the framebuffer size and the window size.
        // May be different from the DPI scaling!
        double pixel_ratio();

        double widget_scaling() { return dpi_scaling() / pixel_ratio(); }

        // We don't need a per-window font. So this function is static
        void  reload_font(int font_size = 16);

    protected:
        // Single global context by default, but can be overridden by the user
        static ImGuiContext *	context_;

        // Global variables for all the windows
        float	alpha_;

        // shadowing
        bool shadowing_enabled_;
    };

}


#endif //TREEGROWTHMODELLING_GROWTH_VIEWER_IMGUI_H
