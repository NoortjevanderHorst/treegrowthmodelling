//
// Created by noort on 23/01/2022.
//

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

        virtual bool ts_visualisation(int ts_index, int item_index, bool show, int skeleton_type) = 0;
        virtual bool ts_change_colors(int ts_index, int item_index, ImVec4 color) = 0;
        virtual bool inter_visualisation(int item_index, bool show) = 0;

        virtual bool open_multiple() = 0;
        virtual bool open_mesh() = 0;
        virtual std::string open_multitemporal() = 0;
        virtual bool complete_multitemporal_import(std::vector<std::string> filenames) = 0;
        virtual void export_skeleton() const = 0;
        virtual void export_leaves() const = 0;
        virtual void export_lobes() const = 0;
        virtual void export_main() const = 0;
        virtual void export_branches_corr() const = 0;
        virtual void export_branches_ts() const = 0;
        virtual void export_correspondences() const = 0;

        virtual bool reconstruct_skeleton() = 0;
        virtual bool add_leaves() = 0;
        virtual bool batch_process() = 0;
        virtual bool save_batch() = 0;
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
