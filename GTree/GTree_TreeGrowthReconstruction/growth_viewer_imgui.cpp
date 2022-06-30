//
// Created by noort on 23/01/2022.
//


#include "growth_viewer_imgui.h"
#include <easy3d/core/types.h>
#include <easy3d/core/point_cloud.h>



namespace easy3d {

    ImGuiContext* ViewerImGuiGrow::context_ = nullptr;

    ViewerImGuiGrow::ViewerImGuiGrow(
            const std::string& title /* = "Easy3D ImGui Viewer" */,
            int samples /* = 4 */,
            int gl_major /* = 3 */,
            int gl_minor /* = 2 */,
            bool full_screen /* = false */,
            bool resizable /* = true */,
            int depth_bits /* = 24 */,
            int stencil_bits /* = 8 */
    )
            : Viewer(title, samples, gl_major, gl_minor, full_screen, resizable, depth_bits, stencil_bits)
            , alpha_(0.8f), shadowing_enabled_(false)
    {
    }


    void ViewerImGuiGrow::init() {
        Viewer::init();

        if (!context_) {
            // Setup ImGui binding
            IMGUI_CHECKVERSION();

            context_ = ImGui::CreateContext();

            const char* glsl_version = "#version 150";
            ImGui_ImplGlfw_InitForOpenGL(window_, false);
            ImGui_ImplOpenGL3_Init(glsl_version);
            ImGuiIO& io = ImGui::GetIO();
            io.WantCaptureKeyboard = true;
            io.WantTextInput = true;
            io.IniFilename = nullptr;
            ImGui::StyleColorsLight();
            ImGuiStyle& style = ImGui::GetStyle();
            style.FrameRounding = 5.0f;

            // load font
            reload_font();
        }
    }


    double ViewerImGuiGrow::pixel_ratio() {
        // Computes pixel ratio for hidpi devices
        int fbo_size[2], win_size[2];
        glfwGetFramebufferSize(window_, &fbo_size[0], &fbo_size[1]);
        glfwGetWindowSize(window_, &win_size[0], &win_size[1]);
        return static_cast<double>(fbo_size[0]) / static_cast<double>(win_size[0]);
    }


    void ViewerImGuiGrow::reload_font(int font_size)
    {
        ImGuiIO& io = ImGui::GetIO();
        io.Fonts->Clear();
        io.Fonts->AddFontFromMemoryCompressedTTF(droid_sans_compressed_data, droid_sans_compressed_size, static_cast<float>(font_size * dpi_scaling()));
        io.FontGlobalScale = static_cast<float>(1.0 / pixel_ratio());
        ImGui_ImplOpenGL3_DestroyDeviceObjects();
    }


    void ViewerImGuiGrow::post_resize(int w, int h) {
        Viewer::post_resize(w, h);
        if (context_) {
            ImGui::GetIO().DisplaySize.x = float(w);
            ImGui::GetIO().DisplaySize.y = float(h);
        }
    }


    bool ViewerImGuiGrow::callback_event_cursor_pos(double x, double y) {
        if (ImGui::GetIO().WantCaptureMouse)
            return true;
        else
            return Viewer::callback_event_cursor_pos(x, y);
    }


    bool ViewerImGuiGrow::callback_event_mouse_button(int button, int action, int modifiers) {
        if (ImGui::GetIO().WantCaptureMouse)
            return true;
        else
            return Viewer::callback_event_mouse_button(button, action, modifiers);
    }


    bool ViewerImGuiGrow::callback_event_keyboard(int key, int action, int modifiers) {
        if (ImGui::GetIO().WantCaptureKeyboard)
            return true;
        else
            return Viewer::callback_event_keyboard(key, action, modifiers);
    }


    bool ViewerImGuiGrow::callback_event_character(unsigned int codepoint) {
        if (ImGui::GetIO().WantCaptureKeyboard)
            return true;
        else
            return Viewer::callback_event_character(codepoint);
    }


    bool ViewerImGuiGrow::callback_event_scroll(double dx, double dy) {
        if (ImGui::GetIO().WantCaptureMouse)
            return true;
        else
            return Viewer::callback_event_scroll(dx, dy);
    }


    void ViewerImGuiGrow::cleanup() {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();

        ImGui::DestroyContext(context_);

        Viewer::cleanup();
    }


    void ViewerImGuiGrow::pre_draw() {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        Viewer::pre_draw();
    }


    // todo: change to description of GTree
    void ViewerImGuiGrow::post_draw() {
        static bool show_about = false;
        if (show_about) {
            ImGui::SetNextWindowPos(ImVec2(width() * 0.5f, height() * 0.5f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
            ImGui::Begin("About AdTree", &show_about, ImGuiWindowFlags_NoResize);
            ImGui::Text(
                    "AdTree implements the reconstruction method of the following paper\n"
                    "\tShenglan Du, Roderik Lindenbergh, Hugo Ledoux, Jantien Stoter, and Liangliang Nan.\n"
                    "\tAdTree: Accurate, Detailed, and Automatic Modeling of Laser-Scanned Trees.\n"
                    "\tRemote Sensing. 2019, 11(18), 2074.\n\n"
            );
            ImGui::Separator();
            ImGui::Text(
                    "\n"
                    "Shenglan Du (dushenglan940128@163.com)\n"
                    "Liangliang Nan (liangliang.nan@gmail.com)\n"
                    "3D Geoinformation Group, TU Delft\n"
                    "https://3d.bk.tudelft.nl"
            );
            ImGui::End();
        }

        static bool show_manual = false;
        if (show_manual) {
            int w, h;
            glfwGetWindowSize(window_, &w, &h);
            ImGui::SetNextWindowPos(ImVec2(w * 0.5f, h * 0.5f), ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.5f));
            ImGui::Begin("AdTree Manual", &show_manual, ImGuiWindowFlags_NoResize);
            ImGui::Text("%s", usage().c_str());
            ImGui::End();
        }

        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5, 8));
        if (ImGui::BeginMainMenuBar())
        {
            draw_menu_file();

            draw_menu_view();

            draw_menu_reconstruction();

            if (ImGui::BeginMenu("Help"))
            {
                ImGui::MenuItem("Manual", nullptr, &show_manual);
                ImGui::Separator();
                ImGui::MenuItem("About", nullptr, &show_about);
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        draw_panel();

        ImGui::PopStyleVar();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        Viewer::post_draw();
    }


    // make menu buttons here, in sub-categories
    void ViewerImGuiGrow::draw_menu_file() {
        static bool show_multitemp_import = false;
        if (show_multitemp_import) {
            draw_multitemp_import(&show_multitemp_import);
        }

        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Open cloud", "Ctrl+O"))
                open();
            if (ImGui::MenuItem("Open multiple clouds"))
                open_multiple();
            if (ImGui::MenuItem("Open mesh", nullptr))
                open_mesh();
            if (ImGui::MenuItem("Open multi-temporal", nullptr, &show_multitemp_import));
            ImGui::Separator();
            if (ImGui::MenuItem("Save branches ...", "Ctrl+S"))
                save();
            if (ImGui::MenuItem("Save batch ..."))
                save_batch();
            if (ImGui::MenuItem("Save leaves ..."))
                export_leaves();
            ImGui::Separator();
            if (ImGui::MenuItem("Save skeleton corresponding ..."))
                export_skeleton();
            if (ImGui::MenuItem("Save skeleton ts main ..."))
                export_main();
            ImGui::Separator();
            if (ImGui::MenuItem("Save lobes ..."))
                export_lobes();
            if (ImGui::MenuItem("Save branches corresponding ..."))
                export_branches_corr();
            if (ImGui::MenuItem("Save branches timestamp ..."))
                export_branches_ts();
            if (ImGui::MenuItem("Save correspondences ..."))
                export_correspondences();
            ImGui::Separator();
            if (ImGui::MenuItem("Quit", "Alt+F4"))
                glfwSetWindowShouldClose(window_, GLFW_TRUE);
            ImGui::EndMenu();
        }
    }


    void ViewerImGuiGrow::draw_menu_view() {
        if (ImGui::BeginMenu("View"))
        {
            if (ImGui::BeginMenu("Options"))
            {
                ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.50f);

                static int style_idx = 2;
                if (ImGui::Combo("Style", &style_idx, "Classic\0Dark\0Light\0")) {
                    switch (style_idx) {
                        case 0: ImGui::StyleColorsClassic(); break;
                        case 1: ImGui::StyleColorsDark(); break;
                        case 2: ImGui::StyleColorsLight(); break;
                    }
                }
                ImGui::ColorEdit3("Background Color", background_color_.data(), ImGuiColorEditFlags_NoInputs);

                if (ImGui::Checkbox("Showdowing", &shadowing_enabled_))
                    update();

                ImGui::PopItemWidth();
                ImGui::EndMenu();
            }

            ImGui::EndMenu();
        }
    }


    void ViewerImGuiGrow::draw_menu_reconstruction() {
        if (ImGui::BeginMenu("Reconstruction"))
        {
            if (ImGui::MenuItem("Reconstruct Skeleton", nullptr))
                reconstruct_skeleton();
            if (ImGui::MenuItem("Add Leaves", nullptr))
                add_leaves();
            if (ImGui::MenuItem("Batch Reconstruct", nullptr))
                batch_process();
            ImGui::Separator();
            if (ImGui::MenuItem("Reconstruct Multi-temporal", nullptr))
                reconstruct_multitemporal();
            if (ImGui::MenuItem("Add Merged Cloud", nullptr))
                add_merged_cloud();
            if (ImGui::MenuItem("Model Correspondence", nullptr))
                model_correspondence();
            ImGui::Separator();
            if (ImGui::MenuItem("Model Growth", nullptr))
                model_growth();
            if (ImGui::MenuItem("Reconstruct Geometry", nullptr))
                reconstruct_geometry();
            if (ImGui::MenuItem("Model Interpolation", nullptr))
                model_interpolation();
            ImGui::Separator();
            if (ImGui::MenuItem("Reconstruct/Model ALL", nullptr))
                reconstruct_all();

            ImGui::EndMenu();
        }
    }


    void ViewerImGuiGrow::draw_panel(){
        ImGui::SetNextWindowSize({260,400}, ImGuiCond_FirstUseEver);
        int w, h;
        glfwGetWindowSize(window_, &w, &h);
        ImGui::SetNextWindowPos(ImVec2((w-265) , 40), ImGuiCond_FirstUseEver, ImVec2(0, 0));
        ImGui::Begin("Timestamp Visualisation");

        // see how many point cloud models were imported
        int nr_timestamps = 0;
        for (Model* m : models()){
            if (dynamic_cast<easy3d::PointCloud*>(m)){
                nr_timestamps++;
            }
        }

        // disable visualization menu if no point clouds were imported yet
        static bool ts_visualisation_enabled;
        if (nr_timestamps == 0){
            ts_visualisation_enabled = false;
        } else {
            ts_visualisation_enabled = true;
        }

        // visualisation menu options per timestamp
        if (ts_visualisation_enabled){
            ImGui::Indent();

            static std::vector<std::vector<std::pair<std::string, bool> > > checkbox_map;
            static std::vector<std::vector<ImVec4> > colors;
            static std::vector<int> skeleton_types;

            static std::vector<std::pair<std::string, bool> > checkboxes_inter;

            // initialization of selected value containers
            if (checkbox_map.size() < nr_timestamps){
                static std::vector<std::pair<std::string, bool> > checked_init = {{"points", true},
                                                                                  {"vertex importance", false},
                                                                                  {"normals", false},
                                                                                  {"edges", false},
                                                                                  {"branching levels", false},
                                                                                  {"correspondence", false},
                                                                                  {"main", false},
                                                                                  {"main vertices", false},
                                                                                  {"merged main", false},
                                                                                  {"bifurcation", false},
                                                                                  {"lobes", false},
                                                                                  {"lobe mesh", false},
                                                                                  {"lobe skeleton", false},
                                                                                  {"skeleton distance", false},
                                                                                  {"branches", false}};
                while (checkbox_map.size() < nr_timestamps){
                    checkbox_map.push_back(checked_init);
                }
            }
            if (colors.size() < nr_timestamps){
                while (colors.size() < nr_timestamps){
                    static ImVec4 color_default_v = {0.3f, 0.67f, 1.0f, 1.0f};
                    static ImVec4 color_default_e = {0.0f, 0.3f, 0.56f, 1.0f};
                    colors.push_back({color_default_v, color_default_e});
                }
            }
            if (skeleton_types.size() < nr_timestamps){
                while (skeleton_types.size() < nr_timestamps){
                    static int skel_type = 4;
                    skeleton_types.push_back(skel_type);
                }
            }

            if (nr_timestamps > 2) {
                if (checkboxes_inter.size() < (nr_timestamps - 2)) {
                    for (int k = checkboxes_inter.size(); k < (nr_timestamps - 2); ++k) {
                        std::stringstream ss;
                        ss << "inter " << k << "/" << k + 1;
                        std::string name_string = ss.str();
                        std::pair<std::string, bool> pair = {name_string, false};
                        checkboxes_inter.push_back(pair);
                    }
                }
            }

            for (int i = 0; i < nr_timestamps; ++i) {
                ImGui::PushID(i);

                std::stringstream ss;
                ss << "timestamp " << i;
                std::string name_string = ss.str();
                const char* timestamp_name = name_string.c_str();

                if (ImGui::CollapsingHeader(timestamp_name)){
                    // type of skeleton
                    if (ImGui::TreeNode("skeleton type")) {
                        ImGui::RadioButton("Delaunay", &skeleton_types[i], 0);
                        ImGui::RadioButton("MST", &skeleton_types[i], 1);
                        ImGui::RadioButton("Simplified", &skeleton_types[i], 2);
                        ImGui::RadioButton("Corresponding", &skeleton_types[i], 3);
                        ImGui::RadioButton("Timestamp Main", &skeleton_types[i], 4);

                        // todo: re-visualize edges if radio button changes value
                        // perhaps with update button?

                        ImGui::TreePop();
                    }

                    // colors
                    if (ImGui::TreeNode("colors")){
                        bool changed_cv = ImGui::ColorEdit3("vertex color", (float*)&colors[i][0], ImGuiColorEditFlags_NoInputs);
                        bool changed_ce = ImGui::ColorEdit3("edge color", (float*)&colors[i][1], ImGuiColorEditFlags_NoInputs);

                        if (changed_cv)
                            ts_change_colors(i, 0, colors[i][0]);
                        if (changed_ce)
                            ts_change_colors(i, 1, colors[i][1]);

                        ImGui::TreePop();
                    }

                    // drawable selection
                    if (ImGui::TreeNode("structure")) {
                        for (int j = 0; j < checkbox_map[i].size(); ++j) {
                            bool changed = ImGui::Checkbox(checkbox_map[i][j].first.c_str(),&checkbox_map[i][j].second);
                            if (changed) {
                                ts_visualisation(i, j, checkbox_map[i][j].second, skeleton_types[i], colors[i]);
                            }
                        }

                        ImGui::TreePop();
                    }
                }
                ImGui::PopID();
            }
            // interpolation boxes
            if (nr_timestamps > 2) {
                if (ImGui::CollapsingHeader("Correspondence")) {
                    for (int k = 0; k < (nr_timestamps - 2); ++k) {
                        bool changed = ImGui::Checkbox(checkboxes_inter[k].first.c_str(), &checkboxes_inter[k].second);
                        if (changed){
                            inter_visualisation(k, checkboxes_inter[k].second);
                        }
                    }
                }
            }

            ImGui::Unindent();
        }
        ImGui::End();
    }


    void ViewerImGuiGrow::draw_multitemp_import(bool* is_open){
        // todo: add button to clear selected file

        ImGui::SetNextWindowSize({0,0});
        int w, h;
        glfwGetWindowSize(window_, &w, &h);
        ImGui::SetNextWindowPos(ImVec2(w * 0.5f, h * 0.5f), ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.5f));

        if (!ImGui::Begin("Open multi-temporal data", is_open, ImGuiWindowFlags_NoResize)){
            ImGui::End();
        } else {
            ImGui::BulletText("Select how many timestamp files to import:");
            ImGui::Indent();

            // select number of timestamps to import
            static int nr_timestamps = 3;
            float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
            ImGui::PushButtonRepeat(true);
            if (ImGui::ArrowButton("##left", ImGuiDir_Left)) {nr_timestamps--; }
            ImGui::SameLine(0.0f, spacing);
            if (ImGui::ArrowButton("##right", ImGuiDir_Right)) { nr_timestamps++; }
            ImGui::PopButtonRepeat();
            ImGui::SameLine();
            ImGui::Text("%d timestamps", nr_timestamps);

            ImGui::Unindent();
            ImGui::BulletText("Click the buttons to select a file:");
            ImGui::Indent();

            // create buttons & import for number of selected files
            static std::vector<std::string> filenames(nr_timestamps);
            filenames.resize(nr_timestamps);
            for (int i = 0; i < nr_timestamps; ++i) {
                ImGui::PushID(i);

                std::stringstream ss;
                ss << "input time " << i;
                std::string name_string = ss.str();
                const char* timestamp_name = name_string.c_str();

                if (ImGui::Button(timestamp_name, ImVec2(200, 50))){
                    std::string file_name_in = open_multitemporal();
                    filenames[i] = file_name_in;

                }
                // display selected filename (without directories)
                std::string filename_short = filenames[i];
                std::string::size_type pos = filename_short.find_last_of("/\\");
                if (pos != std::string::npos)
                    filename_short = filename_short.substr((pos+1));
                ImGui::SameLine();
                ImGui::Text("%s", filename_short.c_str());

                ImGui::PopID();
            }

            ImGui::Unindent();

            if (ImGui::Button(" accept  ")){
                if (complete_multitemporal_import(filenames)){
                    std::cout << "multitemporal import successful" << std::endl;
                }
                *is_open = false;
            }
            ImGui::SameLine();
            if (ImGui::Button(" cancel  ")){
                *is_open = false;
            }
            ImGui::End();
        }
    }

}
