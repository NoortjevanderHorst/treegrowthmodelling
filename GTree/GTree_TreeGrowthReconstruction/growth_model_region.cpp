//
// Created by noort on 17/03/2022.
//

#include "growth_model_region.h"

#include <utility>


GModelRegion::GModelRegion (easy3d::vec3 root, easy3d::vec3 root_dir) : kd_points_in_(nullptr), kdtree_in_(nullptr){
    // set default values
    def_forward_ = 1;
    def_rotate_ = 0.436332;  // radians, = 25 degrees
    def_roll_ = 0.436332;

    num_attr_points_ = 0;

    accuracy_ = 3;

    // set growth parameters
    // todo: use input?
    max_internode_length_ = 1;
    min_internode_length_ = 0.2;

    axiom_ = "A";

    loc_ = root;

    plane_.set_row(0, easy3d::vec3{1, 0, 0});
    plane_.set_row(1, easy3d::vec3{0, 1, 0});
    plane_.set_row(2, easy3d::vec3{0, 0, 1});

    // set starting rotation to direction of parent branch:

    //-- rotation around Z-axis
    // project to xy plane
    easy3d::vec3 dir_xy = {root_dir.x, root_dir.y, 0};

    // find angle between planar vector & x-axis, around the z-axis (radians)
    double angle_z = get_z_angle(dir_xy);

    //-- rotation around Y-axis
    // rotate vector to XZ plane
    easy3d::vec3 zaxis = {0, 0, 1};
    easy3d::vec3 dir_xz = easy3d::mat3::rotation(zaxis, -angle_z) * root_dir;

    // find angle between planar vector & x-axis, around the y-axis
    double angle_y = get_y_angle(dir_xz);

    // make sure angles very close to 0 or 360 degrees get outputted as 0
    if (!(abs(2 * M_PI - angle_y) > 0.0001 && (abs(0 - angle_y) > 0.0001))) {
        angle_y = 0;
    }
    if (!(abs(2 * M_PI - angle_z) > 0.0001 && (abs(0 - angle_z) > 0.0001))) {
        angle_z = 0;
    }

    // rotate plane
    go_rotate(angle_y);
    go_roll(angle_z);

    // store for resetting after each iteration
    start_loc_ = loc_;
    start_plane_ = plane_;

    // initialize R-tree with root
    Point_3_boost pt_root = {loc_.x, loc_.y, loc_.z};
    rtree_out_.insert(pt_root);

}


GModelRegion::GModelRegion(GModelRegion const &other) {
    *this = other;
    graph_.clear();

    // copy pointers
    // leaving default will delete data when copy object (& its pointer attribute) is deleted!
    int num_pts = num_attr_points_;
    kd_points_in_ = new Vector3D[num_pts];
    *kd_points_in_ = *other.kd_points_in_;
    kdtree_in_ = new KdTree(kd_points_in_, num_pts, 16);
}


GModelRegion::~GModelRegion() {
    if (kdtree_in_)
        delete kdtree_in_;
    if (kd_points_in_)
        delete kd_points_in_;
}

//--- general

bool GModelRegion::model_growth(const easy3d::PointCloud* points_in, const Surface_mesh_cgal* lobe_hull){
    if (!points_in){
        std::cout << "ERROR: could not do region growing, input point cloud does not exist" << std::endl;
        return false;
    }
    if (points_in->n_vertices() < 5){
        std::cout << "ERROR: could not do region growing, input point cloud has too few points" << std::endl;
        return false;
    }
    if (!lobe_hull){
        std::cout << "ERROR: could not do region growing, input convex hull does not exist" << std::endl;
        return false;
    }
    if (lobe_hull->is_empty()){
        std::cout << "ERROR: could not do region growing, input convex hull is empty" << std::endl;
        return false;
    }

    // build kd tree and store points internally
    points_ = *points_in;

    unsigned int nr_points = points_.n_vertices();
    kd_points_in_ = new Vector3D[nr_points];
    num_attr_points_ = nr_points;

    easy3d::PointCloud::VertexProperty<easy3d::vec3> pts = points_.get_vertex_property<easy3d::vec3>("v:point");
    int cnt = 0;
    for (auto v : points_.vertices()) {
        kd_points_in_[cnt].x = pts[v].x;
        kd_points_in_[cnt].y = pts[v].y;
        kd_points_in_[cnt].z = pts[v].z;
        cnt++;
    }

    kdtree_in_ = new KdTree(kd_points_in_, nr_points, 16);

    // set lobe hull property
    hull_mesh_ = *lobe_hull;

    // grow
    bool continue_grow = true;
    while (continue_grow && num_iterations_ < 12) {
        continue_grow = grow_iteration();
    }

    return true;
}


//--- growing

bool GModelRegion::grow_iteration(){
    // clear graph from previous iterations, because axiom gets read anew
    graph_.clear();
    // reset starting position
    loc_ = start_loc_;
    plane_ = start_plane_;

    // update axiom
    axiom_ = read_line(axiom_);
    num_iterations_++;

    // pass back whether this should be the final iteration (no more growth possible) at end
    // todo: more advanced check?
    if (axiom_.find('A') == std::string::npos && axiom_.find('T') == std::string::npos &&
        axiom_.find('M') == std::string::npos && axiom_.find('N') == std::string::npos){
        return false;
    } else {
        return true;
    }
}


std::string GModelRegion::read_line(std::string line){
    // add starting vert to graph
    add_vertex(0);

    // set trunk location
    unsigned int n_trunk = 0;

    // store transformed axiom
    std::string line_trans = line;
    int offset_replace = 0;

    // flag if current location is at start of (sub) branch/trunk
    bool branch_start = false;

    for (unsigned int i = 0; i < line.length(); ++i){
        // store passed values as numbers (for F, <> and +-, signified by "(...)")
        float value = 0;
        int offset_value = 0;

        if (line[i + 1] == '('){
            offset_value = 2;
            std::string found_value;
            while (line[i + offset_value] != ')'){
                // todo: catch when ")" does not exist?
                found_value += line[i + offset_value];
                offset_value++;
            }
            value = std::stof(found_value);
        }

        // process nesting
        if (line[i] == '['){
            // store nesting positions
            int nested_open = 1;
            int nested_closed = 0;
            int jump = 0;

            // process rest of line recursively as nested
            for (unsigned int i_nest = i; i_nest < line.size(); ++i_nest){
                // nesting end reached
                if (line[i_nest] == ']' && nested_open == nested_closed){
                    // recurse self
                    GModelRegion model_temp3(*this); // = new GModelRegion(*this);

                    std::string axiom_replacement = model_temp3.read_line(line.substr(i + 1, i_nest - i - 1));
                    std::string line_trans_start = line_trans.substr(0, i+1 + offset_replace);
                    std::string line_trans_end = line_trans.substr(i_nest + offset_replace, line_trans.size() + offset_replace);

                    line_trans = line_trans_start + axiom_replacement + line_trans_end;

                    int offset_diff = axiom_replacement.size() - line.substr(i + 1, i_nest - i - 1).size();
                    offset_replace += offset_diff;

                    // add found vertices
                    auto cVertexList = model_temp3.graph_.m_vertices;
                    unsigned int offset_verts = graph_.m_vertices.size() - 1;

                    for (unsigned int l = 1; l < model_temp3.graph_.m_vertices.size(); ++l) {
                        // ignore point 0 (is the same point as branch point)
                        if (l == 1) { cVertexList[1].m_property.parent = n_trunk; }
                        else { cVertexList[l].m_property.parent += offset_verts; }
                        graph_.m_vertices.emplace_back(cVertexList[l]);
                    }

                    // add found edges
                    GraphGT graph_recursed = graph_;

                    int offset_edges = 0;
                    for (const auto &e: model_temp3.graph_.m_edges) {
                        unsigned int s;

                        // first edge connects to the trunk
                        if (offset_edges == 0) { s = n_trunk; }
                        else { s = e.m_source + offset_verts; }

                        unsigned int t = e.m_target + offset_verts;

                        boost::add_edge(s, t, graph_recursed);
                        offset_edges++;
                    }

                    // end recursion: add recursed structure to current attributes
                    graph_ = graph_recursed;
                    rtree_out_ = model_temp3.rtree_out_;
                    branch_start = true;
                    break;
                }

                // end marker reached, but not end of current overall nesting
                else if (line[i_nest] == ']'){
                    nested_open++;
                }
                // new sub-nesting found
                else if (line[i_nest] == '['){
                    nested_closed++;
                }

                // count current character towards offset
                jump++;
            }

            // set current position to end of nesting
            i += jump;
            continue;
        }

        // growth: apical bud
        else if (line[i] == 'A' || line[i] == 'T'){
            // determine parent point
            easy3d::vec3 p_parent;
            // parent of first vertex is in main part of skeleton, cannot find it with indices
            if (graph_.m_vertices.size() == 1){
                // rotation already correct, so going backwards will give correct relative angles between current vertex and the next
                GModelRegion model_temp2(*this); // = new GModelRegion(*this);
                model_temp2.go_forward(-1);
                p_parent = model_temp2.loc_;
            }
            // parent is known, so direction can be computed from indices
            else {
                p_parent = graph_[graph_[n_trunk].parent].coords;   // n_trunk is latest added vertex (excluding any branching)
            }

            // compute position of new potential apex if grown
            easy3d::vec3 pot_dir = compute_growth_direction(p_parent);
            easy3d::vec3 pot_pt = loc_ + pot_dir;

            std::string replacement;

            // check viability (constraints)
            if (is_viable(pot_pt)){
                // kill attraction points in kill zone of node
                kill_zone(pot_pt);

                // add node to be grown to R-tree
                // (to make sure other new nodes after this can detect it already so they won't grow into it)
                Point_3_boost pot_pt_boost = {pot_pt.x, pot_pt.y, pot_pt.z};
                rtree_out_.insert(pot_pt_boost); // doubles are handled by boost

                // add lateral node potential marker + new internode + new apical bud
                // encode lateral bud alternating with M/N and T/A
                std::tuple<double, double, double> movement_values = points_to_movement(p_parent, pot_pt);
                if (line[i] == 'A')
                    replacement = "M" + movement_to_string(movement_values) + "T";
                if (line[i] == 'T')
                    replacement = "N" + movement_to_string(movement_values) + "A";
            } else {
                // mark as dead/dormant
                replacement = "D";
            }

            // add transformation to pending new axiom
            line_trans.insert(i + offset_replace, replacement);
            line_trans.erase(i + offset_replace + replacement.size(), 1);

            offset_replace += replacement.size() - 1;
        }

        // create lateral buds
        else if (line[i] == 'N' || line[i] == 'M'){
            float phyl_angle = 25 * M_PI / 180;    // todo: make parameter

            std::string replacement;

            // check both lateral buds
            for (int bud = 0; bud < 2; ++bud){

                // determine parent point
                easy3d::vec3 p_parent;
                // parent of first vertex is in main part of skeleton, cannot find it with indices
                if (graph_.m_vertices.size() == 1){
                    // get relative direction by going backwards
                    GModelRegion model_temp5(*this);
                    model_temp5.go_forward(-1);
                    p_parent = model_temp5.loc_;
                }
                // parent is known, so direction can be computed from indices
                else {
                    p_parent = graph_[graph_[n_trunk].parent].coords;   // n_trunk is latest added vertex (excluding any branching)
                }

                // get new potential bud location
                GModelRegion model_temp4(*this); // = new GModelRegion(*this);

                // rotate
                if (line[i] == 'N'){
                    if (bud == 0){
                        model_temp4.go_rotate(phyl_angle);
                    } else { // bud = 1
                        model_temp4.go_rotate(-phyl_angle);
                    }
                } else { // line = "M"
                    if (bud == 0){
                        model_temp4.go_roll(phyl_angle);
                    } else {
                        model_temp4.go_roll(-phyl_angle);
                    }
                }

                // go forward
                float dist_parent = (loc_ - p_parent).length();
                if (dist_parent > max_internode_length_){
                    dist_parent = max_internode_length_;
                }
                model_temp4.go_forward(dist_parent);
                easy3d::vec3 pot_pt_default = model_temp4.loc_;

                easy3d::vec3 pot_dir = compute_growth_direction_lateral(pot_pt_default, p_parent);
                easy3d::vec3 pot_pt = loc_ + pot_dir;

                // check viability (constraints)
                if (is_viable(pot_pt)){ // is_viable_lateral(pot_pt, p_parent) &&
                    // kill attraction points in kill zone of node
                    kill_zone(pot_pt);

                    // add node to be grown to R-tree
                    // (to make sure other new nodes after this can detect it already so they won't grow into it)
                    Point_3_boost pot_pt_boost = {pot_pt.x, pot_pt.y, pot_pt.z};
                    rtree_out_.insert(pot_pt_boost);

                    // remove lateral node potential marker, add new (lateral) internode + new apical bud
                    std::tuple<double, double, double> movement_values = points_to_movement(p_parent, pot_pt);
                    if (line[i] == 'N')
                        replacement += "[" + movement_to_string(movement_values) + "T]";
                    if (line[i] == 'M')
                        replacement += "[" + movement_to_string(movement_values) + "A]";
                } else {
                    // mark as dead/dormant
                    replacement += "[D]";
                }
            }

            // add transformation to pending new axiom
            line_trans.insert(i + offset_replace, replacement);
            line_trans.erase(i + offset_replace + replacement.size(), 1);

            offset_replace += replacement.size() - 1;
        }

        // forward command
        else if (line[i] == 'F'){
            // no value given in string
            if (value == 0){
                go_forward(def_forward_);
            } else {
                go_forward(value);
            }

            // add vertex & edge to current graph
            auto verts = graph_.m_vertices;
            unsigned int n_curr = verts.size();
            if (!branch_start){ // not start of (sub) branch
                // add vertex & edge to graph
                unsigned int n_parent = n_curr -1;   // parent is always latest added vertex
                add_vertex(n_parent);
                boost::add_edge(n_parent, n_curr, graph_);
            } else { // current loc is start of (sub) branch ("[]F")
                branch_start = false;

                // add vertex & edge to graph
                add_vertex(n_trunk);    // parent is always trunk point
                boost::add_edge(n_trunk, n_curr, graph_);
            }

            // set trunk vertex to latest added vertex
            n_trunk = graph_.m_vertices.size() - 1;
        }

        // rotate commands
        else if (line[i] == '+'){
            if (value == 0){
                go_rotate(def_rotate_);
            } else {
                go_rotate(value);
            }
        }
        else if (line[i] == '-'){
            if (value == 0){
                go_rotate(-def_roll_);
            } else {
                go_rotate(-value);
            }
        }

        // roll commands
        else if (line[i] == '>'){
            if (value == 0){
                go_roll(def_roll_);
            } else {
                go_roll(value);
            }
        }
        else if (line[i] == '<'){
            if (value == 0){
                go_roll(-def_roll_);
            } else {
                go_roll(-value);
            }
        }

        // process end of nesting
        else if (line[i] == ']'){
            // terminate (recursive) execution if end of nesting is found
            return line_trans;
        }

        i += offset_value;
    }

    return line_trans;
}


bool GModelRegion::is_viable(easy3d::vec3 pot_loc){
    // todo: maximum angle between parent
    // todo: and minimum angle

    // enforce internode size bounds
    float internode_length = (loc_ - pot_loc).length();
    if (internode_length < min_internode_length_){
        return false;
    } else if (internode_length > max_internode_length_){
        // todo: perhaps shorten instead? (should be done in direction computation...)
        return false;
    }

    // constrain with hull
    CGAL::Side_of_triangle_mesh<Surface_mesh_cgal, K_cgal> inside(hull_mesh_);
    Point_3_cgal pot_pt_cgal = {pot_loc.x, pot_loc.y, pot_loc.z};
    CGAL::Bounded_side side_res = inside(pot_pt_cgal);
    if (!(side_res == CGAL::ON_BOUNDED_SIDE || side_res == CGAL::ON_BOUNDARY)){
        // use distance to allow
        std::vector<Point_3_cgal> pts_for_dist;
        pts_for_dist.push_back(pot_pt_cgal);
        double dist_to_hull = CGAL::Polygon_mesh_processing::max_distance_to_triangle_mesh<CGAL::Sequential_tag>(pts_for_dist, hull_mesh_);
        double dist_allowance = 0.1; // todo: make parameter, make dependent on previous edge length?
        if (dist_to_hull > dist_allowance){
            return false;
            // todo: shorten to fit?
        }
    }

    // check if it's too close to already grown nodes
    Point_3_boost pot_pt_boost = {pot_loc.x, pot_loc.y, pot_loc.z};
    std::vector<Point_3_boost> nbors;
    rtree_out_.query(boost::geometry::index::nearest(pot_pt_boost, 1), std::back_inserter(nbors));
    easy3d::vec3 pt_closest_nbor = {nbors[0].get<0>(), nbors[0].get<1>(), nbors[0].get<2>()};
    float dist_closest_nbor = (pot_loc - pt_closest_nbor).length();

    if (dist_closest_nbor < 0.1){   // todo: make parameter (for now == kill distance)
        return false;
    }

    return true;
}


int GModelRegion::kill_zone(easy3d::vec3 node){
    // todo: make parameter
    float kill_distance = pow(0.3, 2);  // [m]

    // distance query (orb/radius)
    Vector3D p_curr = {node.x, node.y, node.z};
    kdtree_in_->queryRange(p_curr, kill_distance, true);
    int nr_nbors = kdtree_in_->getNOfFoundNeighbours();
    std::vector<unsigned int> kill_idx; // indices of points in kdtree point set to kill
    if (nr_nbors > 0) {
        for (int i = 0; i < nr_nbors; ++i) {
            unsigned int hit_idx = kdtree_in_->getNeighbourPositionIndex(i);
            kill_idx.push_back(hit_idx);
        }
    }

    // put all items from original kdtree points into new array if they weren't found
    int nr_pts_remaining = num_attr_points_ - nr_nbors;
    auto kdtree_pts_new = new Vector3D[nr_pts_remaining];

    int count = 0;
    for (int idx = 0; idx < num_attr_points_; ++idx){
        if (std::find(kill_idx.begin(), kill_idx.end(), idx) == kill_idx.end()){
            kdtree_pts_new[count] = kd_points_in_[idx];
            count++;
        }
    }

    // assign data to pointers, do not make new pointers! Otherwise heap corruption.
    *kd_points_in_ = *kdtree_pts_new;
    auto kd_tree_new = new KdTree(kd_points_in_, nr_pts_remaining, 16);
    *kdtree_in_ = *kd_tree_new;

    return nr_nbors;
}


easy3d::vec3 GModelRegion::compute_growth_direction(easy3d::vec3 p_parent){
    // todo: minimum internode length

    // todo: grow with increments until max internode length (could be set to shorter that way
    //  & results in shorter shoots, but more steps needed)

    Point_3_cgal p_loc_gcal = {loc_.x, loc_.y, loc_.z};
    auto hull_res = CGAL::Polygon_mesh_processing::locate(p_loc_gcal, hull_mesh_);
    Point_3_cgal pt_hull = CGAL::Polygon_mesh_processing::construct_point(hull_res, hull_mesh_);
    easy3d::vec3 p_hull = {static_cast<float>(pt_hull[0]),
                              static_cast<float>(pt_hull[1]),
                              static_cast<float>(pt_hull[2])};
    float dist_to_hull = (p_hull - loc_).length();

    easy3d::vec3 direction{0, 0, 0};
    easy3d::vec3 centroid = {0, 0, 0};

    // cone search parameters
    float dist_parent = (loc_ - p_parent).length();
    float cone_depth = 4 * dist_parent;    // cone height/depth [m], advised: [4l, 12l]
    float cone_angle = 45 * M_PI / 180;   // max angle of cone [radians] (advised: 45 deg)
    // todo: make parameters

    // cone search needed points
    Vector3D p_curr = {loc_.x, loc_.y, loc_.z};

    easy3d::vec3 cone_endpt = {0, 0, 0};
    GModelRegion model_cone(*this);
    model_cone.go_forward(cone_depth);
    easy3d::vec3 pt_cone_end = model_cone.loc_;
    Vector3D p_cone_end = {pt_cone_end.x, pt_cone_end.y, pt_cone_end.z};

    // cone search
    kdtree_in_->queryConeIntersection(p_curr, p_curr, p_cone_end, cone_angle, false, true);
    double nr_nbors = kdtree_in_->getNOfFoundNeighbours();

    if (nr_nbors > 0) {
        // use perception cone
        for (int i = 0; i < nr_nbors; ++i) {
            unsigned int hit_idx = kdtree_in_->getNeighbourPositionIndex(i);
            Vector3D p_hit = kd_points_in_[hit_idx];
            easy3d::vec3 pt_hit = {p_hit.x, p_hit.y, p_hit.z};

            // update centroid
            centroid.x += p_hit.x;
            centroid.y += p_hit.y;
            centroid.z += p_hit.z;

            // update average vector
            easy3d::vec3 dir_curr_norm = (pt_hit - loc_).normalize();
            direction += dir_curr_norm;
        }
    }
    // perception cone cannot be used (attraction points don't exist or are too far away)
    else {
        // use default forward as attraction point, plus nearest neighbours if they can be found

        // simply go forward with fraction of previous edge length, use that as attraction point
        float factor_forward = 0.95;    // distance decrease as branches in lobe grow further from connector node
        // todo: make parameter

        GModelRegion model_temp(*this);
        float dist_parent_f = (loc_ - p_parent).length();
        model_temp.go_forward(dist_parent_f * factor_forward);
        easy3d::vec3 pot_pt = model_temp.loc_;

        // update centroid
        centroid.x += pot_pt.x;
        centroid.y += pot_pt.y;
        centroid.z += pot_pt.z;

        // update average vector
        easy3d::vec3 dir_curr_norm = (pot_pt - loc_).normalize();
        direction += dir_curr_norm;
        nr_nbors = 1;

        // use knn if possible to attract to closest neighbour
        // todo: better alternative could be to enlarge perception cone step-wise
        kdtree_in_->queryRange(p_curr, cone_depth, true);
        double nr_nbors_nn = kdtree_in_->getNOfFoundNeighbours();
        if (nr_nbors_nn > 0){   // a knn nbor was found
            // todo: penalise angle diff with cone direction
            for (int j = 0; j < nr_nbors_nn; ++j) {
                unsigned int hit_idx = kdtree_in_->getNeighbourPositionIndex(j);
                Vector3D p_hit = kd_points_in_[hit_idx];
                easy3d::vec3 pt_hit = {p_hit.x, p_hit.y, p_hit.z};

                // update centroid
                centroid.x += p_hit.x;
                centroid.y += p_hit.y;
                centroid.z += p_hit.z;

                // update average vector
                easy3d::vec3 dir_curr_norm = (pt_hit - loc_).normalize();
                direction += dir_curr_norm;
                nr_nbors += 1;
            }
        }
    }

    // if hull is within perception cone distance, use it as negative attractor
    float hull_tolerance = dist_parent;  // todo: make parameter
    // todo: buffer lobes instead/as well?
    if (dist_to_hull < hull_tolerance){
        easy3d::vec3 dir_hull = (p_hull - loc_).normalize();
        float hull_dist_weight = 1 - (dist_to_hull / hull_tolerance); // hull force percentage of perception cone, outside it no influence
        easy3d::vec3 p_hit_hull = (p_hull - (dir_hull * hull_dist_weight));

        centroid.x += p_hit_hull.x;
        centroid.y += p_hit_hull.y;
        centroid.z += p_hit_hull.z;

        direction += -(dir_hull * hull_dist_weight); // add direction away from hull, also weighted (?)
        nr_nbors += 1;
    }

    // compute averages
    centroid /= nr_nbors;
    direction /= nr_nbors;
    float dist = (loc_ - centroid).length();
    if (dist > max_internode_length_){
        dist = max_internode_length_;
    }
    // direction = average normalized vectors * distance to centroid
    direction *= dist;

    return direction;
}


easy3d::vec3 GModelRegion::compute_growth_direction_lateral(easy3d::vec3 pot_pt, easy3d::vec3 p_parent){
    easy3d::vec3 direction = {0, 0, 0};
    easy3d::vec3 centroid = {0, 0, 0};

    //-- add potential point (at full weight)
    centroid.x += pot_pt.x;
    centroid.y += pot_pt.y;
    centroid.z += pot_pt.z;
    easy3d::vec3 dir_curr_norm = (pot_pt - loc_).normalize();
    direction += dir_curr_norm;
    int nr_nbors = 1;

    //--- correct with perception cone & attraction points
    float dist_parent = (loc_ - p_parent).length();
    float cone_depth = 4 * dist_parent;    // cone height/depth [m], advised: [4l, 12l]
    float cone_angle = 45 * M_PI / 180;   // max angle of cone [radians] (advised: 45 deg)
    float influence_radius = 1.2 * dist_parent; // radius within points are seen as attraction points
    float influence_angle = 25 * M_PI / 180;    // max angle between used attractor point and original potential point
    // todo: make parameters
    // todo: perhaps make more strict for laterals
    // maximum distance between potential point and attraction point found
    float max_dist_diff = sqrt(pow(dist_parent, 2) + pow(influence_radius, 2)
                               - (2 * dist_parent * influence_radius * cos((0.5 * influence_radius))));

    // cone search needed points
    Vector3D p_curr = {loc_.x, loc_.y, loc_.z};

    easy3d::vec3 cone_endpt = {0, 0, 0};
    GModelRegion model_cone(*this);
    model_cone.go_forward(cone_depth);
    easy3d::vec3 pt_cone_end = model_cone.loc_;
    Vector3D p_cone_end = {pt_cone_end.x, pt_cone_end.y, pt_cone_end.z};

    // cone search
    kdtree_in_->queryConeIntersection(p_curr, p_curr, p_cone_end, cone_angle, false, true);
    double nr_kd_hits = kdtree_in_->getNOfFoundNeighbours();
    if (nr_kd_hits > 0){
        //-- use perception cone to attract towards nearby points
        for (int i = 0; i < nr_kd_hits; ++i) {
            unsigned int hit_idx = kdtree_in_->getNeighbourPositionIndex(i);
            Vector3D p_hit = kd_points_in_[hit_idx];
            easy3d::vec3 pt_hit = {p_hit.x, p_hit.y, p_hit.z};

            easy3d::vec3 dir_to_pot = pot_pt - loc_;
            easy3d::vec3 dir_to_hit = pt_hit - loc_;
            double angle_diff = acos(dot(dir_to_pot, dir_to_hit) / (length(dir_to_pot) * length(dir_to_hit)));

            if ((pt_hit - loc_).length() <= influence_radius && angle_diff <= influence_angle){
                // establish weight
                dir_to_hit = dir_to_hit.normalize();
                float hit_weight = 0.5 * (1 - ((pt_hit - loc_).length() / max_dist_diff));
                easy3d::vec3 pt_hit_weighted = loc_ + (dir_to_hit * hit_weight);

                // update centroid
                centroid.x += pt_hit_weighted.x;
                centroid.y += pt_hit_weighted.y;
                centroid.z += pt_hit_weighted.z;

                // update average vector
                direction += (dir_to_hit * hit_weight);
                nr_nbors += 1;
            }
        }

        //--- correct with hull negative attraction influence
        Point_3_cgal p_loc_gcal = {loc_.x, loc_.y, loc_.z};
        auto hull_res = CGAL::Polygon_mesh_processing::locate(p_loc_gcal, hull_mesh_);
        Point_3_cgal pt_hull = CGAL::Polygon_mesh_processing::construct_point(hull_res, hull_mesh_);
        easy3d::vec3 p_hull = {static_cast<float>(pt_hull[0]),
                               static_cast<float>(pt_hull[1]),
                               static_cast<float>(pt_hull[2])};
        float dist_to_hull = (p_hull - loc_).length();

        // if hull is within perception cone distance, use it as negative attractor
        float hull_tolerance = dist_parent;  // todo: make parameter
        // todo: buffer lobes instead/as well?
        if (dist_to_hull < hull_tolerance) {
            easy3d::vec3 dir_hull = (p_hull - loc_).normalize();
            float hull_dist_weight = 1 - (dist_to_hull /
                                          hull_tolerance); // hull force percentage of perception cone, outside it no influence
            easy3d::vec3 p_hit_hull = (p_hull - (dir_hull * hull_dist_weight));

            centroid.x += p_hit_hull.x;
            centroid.y += p_hit_hull.y;
            centroid.z += p_hit_hull.z;

            direction += -(dir_hull * hull_dist_weight); // add direction away from hull, also weighted (?)
            nr_nbors += 1;
        }

        // compute averages
        centroid /= nr_nbors;
        direction /= nr_nbors;
        float dist = (loc_ - centroid).length();
        if (dist > max_internode_length_){
            dist = max_internode_length_;
        }
        // direction = average normalized vectors * distance to centroid
        direction *= dist;

        return direction;
    }
    // if no hits within perception cone at all, do not grow lateral branch (return 0 vector, will be seen as non-viable)
    else {
        return {0, 0, 0};
    }
}


easy3d::vec3 GModelRegion::compute_potential_location(std::string movement){
    GModelRegion model_temp(*this);
    model_temp.read_line(movement);
    easy3d::vec3 bud_loc = model_temp.loc_;

    return bud_loc;
}


std::tuple<double, double, double> GModelRegion::points_to_movement(easy3d::vec3 p_parent, easy3d::vec3 p_next){
    std::tuple<double, double, double> movement = {0, 0, 0};    // y angle, z angle, distance

    //-- forward distance
    float distance = easy3d::distance(loc_, p_next);
    std::get<2>(movement) = distance;

    // relative vectors around current loc
    easy3d::vec3 vec_to_next = (p_next - loc_);
    easy3d::vec3 vec_from_parent = (loc_ - p_parent);

    easy3d::vec3 xaxis = {1, 0, 0};
    easy3d::vec3 yaxis = {0, 1, 0};
    easy3d::vec3 zaxis = {0, 0, 1};

    //-- rotation around Z-axis

    // project to xy plane
    easy3d::vec3 from_parent_xy = {vec_from_parent.x, vec_from_parent.y, 0};
    easy3d::vec3 to_next_xy = {vec_to_next.x, vec_to_next.y, 0};

    // find angle between planar vectors & x-axis, around the z-axis (radians)
    double angle_z_parent = get_z_angle(from_parent_xy);
    double angle_z_next = get_z_angle(to_next_xy);

    //-- rotation around Y-axis

    // rotate vectors to XZ plane
    easy3d::vec3 from_parent_xz = easy3d::mat3::rotation(zaxis, -angle_z_parent) * vec_from_parent;
    easy3d::vec3 to_next_xz = easy3d::mat3::rotation(zaxis, -angle_z_next) * vec_to_next;

    // find angle between planar vectors & x-axis, around the y-axis
    double angle_y_parent = get_y_angle(from_parent_xz);
    double angle_y_next = get_y_angle(to_next_xz);

    //-- get relative rotations

    double angle_diff_z = angle_z_next - angle_z_parent;
    double angle_diff_y = angle_y_next - angle_y_parent;

    // make sure angles very close to 0 or 360 degrees get outputted as 0
    if (abs(2 * M_PI - angle_diff_y) > 0.0001 && (abs(0 - angle_diff_y) > 0.0001)) {
        std::get<0>(movement) = angle_diff_y;   // angle y
    }
    if (abs(2 * M_PI - angle_diff_z) > 0.0001 && (abs(0 - angle_diff_z) > 0.0001)) {
        std::get<1>(movement) = angle_diff_z;   // angle z
    }

    return movement;
}


std::string GModelRegion::movement_to_string(std::tuple<double, double, double> movement){
    double angle_y = std::get<0>(movement);
    double angle_z = std::get<1>(movement);
    double distance = std::get<2>(movement);

    //-- to string
    std::string movement_str;

    // write rotation
    // rounded to [accuracy] decimals
    if (angle_y > 0){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(accuracy_) << angle_y;
        std::string angle_y_string = ss.str();
        movement_str += "+(" + angle_y_string + ")";
    }
    if (angle_y < 0){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(accuracy_) << abs(angle_y);
        std::string angle_y_string = ss.str();
        movement_str += "-(" + angle_y_string + ")";
    }
    // write roll
    if (angle_z > 0){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(accuracy_) << angle_z;
        std::string angle_z_string = ss.str();
        movement_str += ">(" + angle_z_string + ")";
    }
    if (angle_z < 0){
        std::stringstream ss;
        ss << std::fixed << std::setprecision(accuracy_) << abs(angle_z);
        std::string angle_z_string = ss.str();
        movement_str += "<(" + angle_z_string + ")";
    }
    // write forward
    if (distance > 0) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(accuracy_) << distance;
        std::string dist_string = ss.str();
        movement_str += "F(" + dist_string + ")";
    }

    return movement_str;
}


double GModelRegion::get_z_angle(easy3d::vec3 vec){
    easy3d::vec3 xaxis = {1, 0, 0};
    double angle_z;

    // angle is dependent on what side of the x-axis (XY plane) the vector is
    if (vec.y < 0){
        angle_z = - acos(dot(vec, xaxis) / (length(vec) * length(xaxis)));
    }
    else{
        angle_z = acos(dot(vec, xaxis) / (length(vec) * length(xaxis)));
    }

    // zero angle returns nan, should be 0
    if (isnan(angle_z)) {
        angle_z = 0;
    }

    return angle_z;
}


double GModelRegion::get_y_angle(easy3d::vec3 vec){
    easy3d::vec3 xaxis = {1, 0, 0};
    double angle_y;

    // angle is dependent on what side of the x-axis (XZ plane) the vector is
    if (vec.z < 0) {
        angle_y = acos(dot(vec, xaxis) / (length(vec) * length(xaxis)));
    }
    else{
        angle_y = (2 * M_PI) - acos(dot(vec, xaxis) / (length(vec) * length(xaxis)));
    }

    // zero angle returns nan, should be 0
    if (isnan(angle_y)) {
        angle_y = 0;
    }

    return angle_y;
}


//--- movement

void GModelRegion::go_forward(float distance) {
    loc_ = loc_ + (plane_ * easy3d::vec3(1, 0, 0) * distance);
}

void GModelRegion::go_rotate(float angle) {
    //-- 1: roll to XZ plane
    // project current z-axis onto XY plane;
    easy3d::vec3 xAxis_orig = {1, 0, 0};
    easy3d::vec3 xAxis = plane_ * xAxis_orig;
    easy3d::vec3 xAxis_proj = {xAxis.x, xAxis.y, 0.0};

    // angle of projected x-axis to original x-axis
    float angle_z;

    if (xAxis_proj.y < 0) {
        angle_z = -acos(dot(xAxis_proj, xAxis_orig) / (length(xAxis_proj) * length(xAxis_orig)));
    } else {
        angle_z = acos(dot(xAxis_proj, xAxis_orig) / (length(xAxis_proj) * length(xAxis_orig)));
    }
    // angle_z is close to 0
    // problem: nan
    if (std::isnan(angle_z)) {
        angle_z = 0;
    }
    // vector points (almost) straight up/down
    // problem: incorrect angle_z in 2d cases
    if (abs(xAxis.z) - 1 < 0.1 && abs(xAxis.z) - 1 > - 0.1 &&
        abs(xAxis.x)  == 0 ||
        abs(xAxis.z) - 1 < 0.1 && abs(xAxis.z) - 1 > - 0.1 &&
        abs(xAxis.y) == 0) {
        angle_z = 0;
    }

    go_roll(-angle_z);

    //-- 2: rotation around Y axis
    easy3d::Mat3<float> ry(1);
    ry(0, 0) = std::cos(angle);
    ry(0, 2) = std::sin(angle);
    ry(2, 0) = -std::sin(angle);
    ry(2, 2) = std::cos(angle);

    plane_ = ry * plane_;

    //-- 3: roll back from XZ plane
    go_roll(angle_z);
}

void GModelRegion::go_roll(float angle) {
    easy3d::Mat3<float> rz(1);
    rz(0, 0) = std::cos(angle);
    rz(0, 1) = -std::sin(angle);
    rz(1, 0) = std::sin(angle);
    rz(1, 1) = std::cos(angle);

    plane_ = rz * plane_;
}

void GModelRegion::add_vertex(unsigned int n_parent){
    SGraphVertexPropGT vnew;
    vnew.coords = loc_;
    if (n_parent >= 0) {
        vnew.parent = n_parent;
    } else {
        vnew.parent = NULL;
    }
    graph_.m_vertices.emplace_back(vnew);
}

