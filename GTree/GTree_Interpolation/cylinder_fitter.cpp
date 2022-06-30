//
// Created by noort on 02/05/2022.
//


#include "cylinder_fitter.h"


CylFit::CylFit() : Points_(nullptr), KDtree_(nullptr) {
    TrunkRadius_ = 0;
    TreeHeight_ = 0;
    BoundingDistance_ = 0;
}


CylFit::~CylFit() {
    if (KDtree_)
        delete KDtree_;
    if (Points_)
        delete Points_;
}


/// --- CONTROL
bool CylFit::reconstruct_branches(easy3d::SurfaceMesh* mesh_result){
//    std::cout << "started reconstruction, root: " << RootV_ << std::endl;
    // compute all needed preliminary info (kd-tree and skeleton are set beforehand)
    compute_length_of_subtree(&skeleton_, RootV_);  // LengthOfSubtree
//    std::cout << "subtree length computed" << std::endl;
    compute_graph_edges_weight(&skeleton_); // nWeight based on LengthOfSubtree
//    std::cout << "edge weight computed" << std::endl;
    compute_all_edges_radius(TrunkRadius_); // nRadius based on nWeight
//    std::cout << "edge radius computed" << std::endl;

    //generate branches
    if (!compute_branch_radius()) {
        std::cerr << "failed computing branch radius" << std::endl;
        return false;
    }

//    std::cout << "branch radius computed" << std::endl;

    //smooth branches
    if (!smooth_skeleton()) {
        std::cerr << "failed smoothing branches" << std::endl;
        return false;
    }

//    std::cout << "smoothed skeleton" << std::endl;

    //extract surface model
    if (!extract_branch_surfaces(mesh_result)) {
        std::cerr << "failed extracting branches" << std::endl;
        return false;
    }

//    std::cout << "extracted surfaces" << std::endl;

    return true;
}


/// --- ADDITIONAL
void CylFit::obtain_initial_radius(const GraphB *graph) {
//    std::cout << "\ntrunk fit statistics" << std::endl;

    //get the 2D bbox of the tree
    double min_x = DBL_MAX;
    double max_x = -DBL_MAX;
    double min_y = DBL_MAX;
    double max_y = -DBL_MAX;
    //get the lowest root point from the point cloud
    easy3d::vec3 pLowest(0, 0, FLT_MAX), pOther;
    std::pair<VertexIteratorGraphB, VertexIteratorGraphB> vt = vertices(*graph);
    for (VertexIteratorGraphB vit = vt.first; vit != vt.second; ++vit){
        //root
        pOther = (*graph)[*vit].coords;
        if (pOther.z < pLowest.z)
            pLowest = pOther;
        //bbox
        if (pOther.x < min_x){
            min_x = pOther.x;
        }
        if (pOther.x > max_x){
            max_x = pOther.x;
        }
        if (pOther.y < min_y){
            min_y = pOther.y;
        }
        if (pOther.y > max_y){
            max_y = pOther.y;
        }
    }

    RootPos_.x = pLowest.x;
    RootPos_.y = pLowest.y;
    RootPos_.z = pLowest.z;

//    std::cout << "the root vertex coordinate is:" << std::endl;
//    std::cout << RootPos_.x << " " << RootPos_.y << " " << RootPos_.z << std::endl;

    //get the tree height and the bounding distance
    for (VertexIteratorGraphB vit = vt.first; vit != vt.second; ++vit){
        pOther = (*graph)[*vit].coords;
        if ((pOther.z - pLowest.z) > TreeHeight_)
            TreeHeight_ = pOther.z - pLowest.z;
        if (std::sqrt(pOther.distance2(pLowest)) > BoundingDistance_)
            BoundingDistance_ = std::sqrt(pOther.distance2(pLowest));
    }

    //get all points that lie within 2% of the tree height
    std::vector<easy3d::vec3> trunkList;
    double epsiony = 0.02;
    for (VertexIteratorGraphB vit = vt.first; vit != vt.second; ++vit){
        pOther = (*graph)[*vit].coords;
        if ((pOther.z - pLowest.z) <= epsiony * TreeHeight_)
            trunkList.push_back(pOther);
    }

    //project the trunk points on the xy plane and get the bounding box
    double minX = DBL_MAX;
    double maxX = -DBL_MAX;
    double minY = DBL_MAX;
    double maxY = -DBL_MAX;
    for (int nP = 0; nP < trunkList.size(); nP++) {
        if (minX > trunkList[nP].x)
            minX = trunkList[nP].x;
        if (maxX < trunkList[nP].x)
            maxX = trunkList[nP].x;
        if (minY > trunkList[nP].y)
            minY = trunkList[nP].y;
        if (maxY < trunkList[nP].y)
            maxY = trunkList[nP].y;
    }

//    std::cout << "trunk box: x min: " << minX << ", x max: " << maxX << "; y min: " << minY << ", y max: " << maxY << std::endl;
//    std::cout << "crown box: x min: " << min_x << ", x max: " << max_x << "; y min: " << min_y << ", y max: " << max_y << std::endl;

    //assign the raw radius value and return
    TrunkRadius_ = std::max((maxX - minX), (maxY - minY)) / 2.0;

//    std::cout << "the initial radius is:" << std::endl;
//    std::cout << TrunkRadius_ << std::endl;

    /// bind trunk radius to reasonable maximum
    double est_crown_rad = (((max_x - min_x) / 2) + ((max_y - min_y) / 2)) / 2;
    double est_max_trunk_rad = est_crown_rad / 30;  // trunk can max be 10% of the crown radius todo: guessed this

//    std::cout << "tree height: " << TreeHeight_ << ", crown radius: " << est_crown_rad << std::endl;
//    std::cout << "estimated max trunk radius: " << est_max_trunk_rad  << std::endl;

    if (TrunkRadius_ > est_max_trunk_rad){
        TrunkRadius_ = est_max_trunk_rad;

//        std::cout << "adapted trunk radius to estimate of maximum: " << TrunkRadius_  << "\n" << std::endl;
    }
    // correct unrealistically small trunks
    if (TrunkRadius_ < (est_max_trunk_rad / 3)){
        float est_mean_trunk_rad = est_max_trunk_rad / 1.1; // based on guesses
        TrunkRadius_ = est_mean_trunk_rad;

//        std::cout << "adapted trunk radius to estimate of maximum: " << TrunkRadius_  << "\n" << std::endl;
    }

//    std::cout << "!!!!!!final trunk radius: " << TrunkRadius_ << std::endl;

    return;
}


void CylFit::obtain_initial_radius_inter(double r_base, double r_target, int step, int nr_steps){
    float intensity = float(step) / float(nr_steps + 1);
    double r_diff = r_target - r_base;

    double r_inter = r_base + (r_diff * intensity);
    TrunkRadius_ = r_inter;
}


void CylFit::construct_kd_tree(const GraphB *graph){
    int nPt = num_vertices(*graph);
    if (Points_)
        delete Points_;
    Points_ = new Vector3D[nPt];

    int Count = 0;
    std::pair<VertexIteratorGraphB, VertexIteratorGraphB> vt = vertices(*graph);
    for (VertexIteratorGraphB vit = vt.first; vit != vt.second; ++vit){
        Points_[Count].x = (*graph)[*vit].coords.x;
        Points_[Count].y = (*graph)[*vit].coords.y;
        Points_[Count].z = (*graph)[*vit].coords.z;
        Count++;
    }
    KDtree_ = new KdTree(Points_, nPt, 16);
}


void CylFit::compute_length_of_subtree(GraphB *i_Graph, VertexDescriptorGraphB i_dVertex) {
    (*i_Graph)[i_dVertex].lengthOfSubtree = 0.0;
    std::pair<AdjacencyIteratorGraphB , AdjacencyIteratorGraphB > adjacency = adjacent_vertices(i_dVertex, *i_Graph);
    for (AdjacencyIteratorGraphB cIter = adjacency.first; cIter != adjacency.second; ++cIter) {
        if (*cIter != (*i_Graph)[i_dVertex].parent) {
            compute_length_of_subtree(i_Graph, *cIter);
            easy3d::vec3 pChild = (*i_Graph)[*cIter].coords;
            easy3d::vec3 pCurrent = (*i_Graph)[i_dVertex].coords;
            double distance = std::sqrt(pCurrent.distance2(pChild));
            double child_Length = (*i_Graph)[*cIter].lengthOfSubtree + distance;

            if ((*i_Graph)[i_dVertex].lengthOfSubtree < child_Length)
                (*i_Graph)[i_dVertex].lengthOfSubtree = child_Length;
        }
    }
    return;
}


void CylFit::compute_graph_edges_weight(GraphB *i_Graph) {
    std::pair<EdgeIteratorGraphB , EdgeIteratorGraphB > ep = edges(*i_Graph);
    for (EdgeIteratorGraphB eIter = ep.first; eIter != ep.second; ++eIter) {
        double subtreeWeight = (*i_Graph)[source(*eIter, *i_Graph)].lengthOfSubtree +
                               (*i_Graph)[target(*eIter, *i_Graph)].lengthOfSubtree;
        (*i_Graph)[*eIter].nWeight = subtreeWeight / 2.0;
    }

    return;
}


void CylFit::compute_all_edges_radius(double trunkRadius) {
    //find the trunk edge
    EdgeDescriptorGraphB trunkE;
    double maxweight = DBL_MIN;
    std::pair<EdgeOutIteratorGraphB , EdgeOutIteratorGraphB > listAdj = out_edges(RootV_, skeleton_);
    for (EdgeOutIteratorGraphB eIter = listAdj.first; eIter != listAdj.second; ++eIter) {
        if (skeleton_[*eIter].nWeight > maxweight ){
            trunkE = *eIter;
            maxweight = skeleton_[*eIter].nWeight;
        }
    }

    //assign the radius to the rest branches
    double avrRadius = trunkRadius / pow(skeleton_[trunkE].nWeight, 1.1);
    std::pair<EdgeIteratorGraphB , EdgeIteratorGraphB > ep = edges(skeleton_);
    for (EdgeIteratorGraphB eIter = ep.first; eIter != ep.second; ++eIter) {
        skeleton_[*eIter].nRadius = pow(skeleton_[*eIter].nWeight, 1.1) * avrRadius;
    }

    return;
}


/// --- STRUCTURAL
void CylFit::set_skeleton(GraphB graph){
    GraphB gres;

    // collapse all double vertices
    std::map<VertexDescriptorGraphB, VertexDescriptorGraphB> dupes_idx; // store duplicates & new indices in gres
    std::pair<VertexIteratorGraphB, VertexIteratorGraphB> vto = vertices(graph);

    // process root
    VertexDescriptorGraphB rootv_res = add_vertex(gres);
    gres[rootv_res].coords = graph[graph.rootv].coords;
    gres.rootv = rootv_res;
    dupes_idx[graph.rootv] = rootv_res;
    RootV_ = rootv_res;

    for (VertexIteratorGraphB vit = vto.first; vit != vto.second; ++vit){
        // root has already been inserted
        if (*vit != graph.rootv) {
            // check all found vertices to see if a duplicate already exists
            bool dupe_found = false;
            std::pair <VertexIteratorGraphB, VertexIteratorGraphB> vtn = vertices(gres);
            for (VertexIteratorGraphB vitn = vtn.first; vitn != vtn.second; ++vitn) {
                double dist = (gres[*vitn].coords - graph[*vit].coords).length();
                if (dist <= 0.0001) {
                    dupes_idx[*vit] = *vitn;
                    dupe_found = true;
//                    std::cout << "dupe found: " << *vit << " --> " << *vitn << std::endl;
                    break;
                }
            }

            // insert vertex if no duplicates were found
            if (!dupe_found) {
                SGraphVertexPropB vnew;
                vnew.coords = graph[*vit].coords;
                VertexDescriptorGraphB i_new = add_vertex(vnew, gres);

                dupes_idx[*vit] = i_new;
//                idx_map[*vit] = i_new;
            }
        }
    }


    std::pair<EdgeIteratorGraphB, EdgeIteratorGraphB> et = edges(graph);
    for (EdgeIteratorGraphB eit = et.first; eit != et.second; ++eit){
        // also reassign duplicates to existing indices
        VertexDescriptorGraphB vsource = dupes_idx[source(*eit, graph)];
        VertexDescriptorGraphB vtarget = dupes_idx[target(*eit, graph)];


        // check if it is not a fully collapsed edge (happens for original timestamp graphs when removing dupes)
        if (vsource != vtarget){
            // add edge at new indices
            add_edge(vsource, vtarget, gres);
        }
    }

    // set parents
    std::vector<VertexDescriptorGraphB> waitlist;
    waitlist.push_back(gres.rootv);
    gres[gres.rootv].parent = gres.rootv;
    while (!waitlist.empty()){
        VertexDescriptorGraphB v_curr = waitlist.back();
        waitlist.pop_back();

        // find and set children
        for (VertexDescriptorGraphB v_child: make_iterator_range(adjacent_vertices(v_curr, gres))){
            if (v_child != gres[v_curr].parent){
                gres[v_child].parent = v_curr;
                waitlist.push_back(v_child);
            }
        }
    }

    skeleton_ = gres;
}


bool CylFit::compute_branch_radius() {

    assign_points_to_edges();

    fit_trunk();

    compute_all_edges_radius(TrunkRadius_);

    return true;
}


void CylFit::assign_points_to_edges() {
    //check if the kd tree has been built
    if (!KDtree_) {
        std::cout << "Error in KD tree construction!" << std::endl;
        return;
    }

    //for each edge, find its corresponding points
    EdgeDescriptorGraphB currentE;
    std::pair<EdgeIteratorGraphB , EdgeIteratorGraphB > ep = edges(skeleton_);
    for (EdgeIteratorGraphB eIter = ep.first; eIter != ep.second; ++eIter) {
        //extract two end vertices of the current edge
        currentE = *eIter;
        skeleton_[currentE].vecPoints.clear();
        double currentR = skeleton_[currentE].nRadius;  // todo: ensure this radius is correct
        VertexDescriptorGraphB sourceV, targetV;
        if (source(currentE, skeleton_) ==
            skeleton_[target(currentE, skeleton_)].parent) {
            sourceV = source(currentE, skeleton_);
            targetV = target(currentE, skeleton_);
        } else {
            sourceV = target(currentE, skeleton_);
            targetV = source(currentE, skeleton_);
        }
        Vector3D pSource(skeleton_[sourceV].coords.x, skeleton_[sourceV].coords.y,
                         skeleton_[sourceV].coords.z);
        Vector3D pTarget(skeleton_[targetV].coords.x, skeleton_[targetV].coords.y,
                         skeleton_[targetV].coords.z);
        //query neighbor points from the kd tree
        KDtree_->queryLineIntersection(pSource, pTarget, 3.5 * currentR, true, true);
        int neighbourSize = KDtree_->getNOfFoundNeighbours();
        for (int i = 0; i < neighbourSize; i++) {
            //get the current neighbor point and check if it lies within the cylinder
            int ptIndex = KDtree_->getNeighbourPositionIndex(i);
            Vector3D pCurrent = Points_[ptIndex];
            Vector3D cDirPoint = pCurrent - pSource;
            Vector3D cDirCylinder = pTarget - pSource;
            double nLengthPoint = cDirPoint.normalize();
            double nLengthCylinder = cDirCylinder.normalize();
            double cosAlpha = Vector3D::dotProduct(cDirCylinder, cDirPoint);
            //if the angle is smaller than 90 and the projection is less than the axis length
            if (cosAlpha >= 0 && nLengthPoint * cosAlpha <= nLengthCylinder)
                skeleton_[currentE].vecPoints.push_back(ptIndex);
        }
    }

    return;
}


void CylFit::fit_trunk() {
    //find the trunk edge
    EdgeDescriptorGraphB trunkE;
    std::pair<EdgeOutIteratorGraphB , EdgeOutIteratorGraphB > listAdj = out_edges(RootV_, skeleton_);
    for (EdgeOutIteratorGraphB eIter = listAdj.first; eIter != listAdj.second; ++eIter) {
        trunkE = *eIter;
        break;
    }

    //if the points attached are not enough, then don't conduct fitting
    std::size_t pCount = skeleton_[trunkE].vecPoints.size();
    if (pCount <= 20) {
        std::cout << "trunk fitting: the least squares fails because of not enough points" << std::endl;
        return;
    }

    //construct the initial cylinder
    VertexDescriptorGraphB sourceV, targetV;
    if (source(trunkE, skeleton_) == skeleton_[target(trunkE, skeleton_)].parent) {
        sourceV = source(trunkE, skeleton_);
        targetV = target(trunkE, skeleton_);
    } else {
        sourceV = target(trunkE, skeleton_);
        targetV = source(trunkE, skeleton_);
    }
    //Vector3D pSource(simplified_skeleton_[sourceV].cVert.x, simplified_skeleton_[sourceV].cVert.y, simplified_skeleton_[sourceV].cVert.z);
    //Vector3D pTarget(simplified_skeleton_[targetV].cVert.x, simplified_skeleton_[targetV].cVert.y, simplified_skeleton_[targetV].cVert.z);
    //Cylinder currentC = Cylinder(pSource, pTarget, simplified_skeleton_[trunkE].nRadius);

    //initialize the mean, the point cloud matrix
    Vector3D pTop(0.0, 0.0, -DBL_MAX);
    Vector3D pBottom(0.0, 0.0, DBL_MAX);

    easy3d::PrincipalAxes<3, double> pca;
    pca.begin();
    //extract the corresponding point cloud
    std::vector<std::vector<double> > ptlist;
    for (int np = 0; np < pCount; np++) {
        int npIndex = skeleton_[trunkE].vecPoints.at(np);
        const Vector3D &pt = Points_[npIndex];
        pca.add(easy3d::dvec3(pt.x, pt.y, pt.z));

        std::vector<double> ptemp;
        ptemp.push_back(pt.x);
        ptemp.push_back(pt.y);
        ptemp.push_back(pt.z);
        ptemp.push_back(1.0); //weights are set to 1
        ptlist.push_back(ptemp);
        if (pt.z < pBottom.z)
            pBottom = pt;
        if (pt.z > pTop.z)
            pTop = pt;
    }
    pca.end();

    //initialize the cylinder with the positions computed from points
    const auto &center = pca.center();
    Vector3D pMean(center.x, center.y, center.z); //get the mean point
    const auto &ev = pca.axis(0);   //the largest eigen vector
    Vector3D cDir(ev.x, ev.y, ev.z);
    if (cDir.z < 0)
        cDir = -cDir;
    Vector3D cDirTop = pTop - pMean;
    Vector3D cDirBottom = pBottom - pMean;
    float nLengthTop = cDirTop.normalize();
    float nLengthBottom = cDirBottom.normalize();
    double cosineTop = Vector3D::dotProduct(cDir, cDirTop);
    double cosineBottom = Vector3D::dotProduct(cDir, cDirBottom);
    Vector3D pSource = pMean + nLengthBottom * cosineBottom * cDir;
    Vector3D pTarget = pMean + nLengthTop * cosineTop * cDir;
    Cylinder currentC = Cylinder(pSource, pTarget, skeleton_[trunkE].nRadius);

    //non linear leastsquares adjustment
    if (currentC.LeastSquaresFit(ptlist.begin(), ptlist.end())) {
        Vector3D pSourceAdjust = currentC.GetAxisPosition1();
        Vector3D pTargetAdjust = currentC.GetAxisPosition2();
        double radiusAdjust = currentC.GetRadius();

        //prepare for the weighted non linear least squares
        currentC.SetAxisPosition1(pSourceAdjust);
        currentC.SetAxisPosition2(pTargetAdjust);
        currentC.SetRadius(radiusAdjust);
        double maxDis = -DBL_MAX;
        std::vector<double> disList;
        for (int np = 0; np < pCount; np++) {
            Vector3D pt(ptlist[np][0], ptlist[np][1], ptlist[np][2]);
            //Compute the distance from current pt to the line formed by source and target vertex
            double dis = (Vector3D::crossProduct(pt - pSourceAdjust, pt - pTargetAdjust)).normalize()
                         / ((pSourceAdjust - pTargetAdjust).normalize());
            //Substract the distance with the radius
            dis = abs(dis - radiusAdjust);
            if (dis > maxDis) maxDis = dis;
            disList.push_back(dis);
        }

        //update the weights
        for (int np = 0; np < pCount; np++)
            ptlist[np][3] = 1.0 - disList[np] / maxDis;

        //conduct the second round of weighted least squares
        if (currentC.LeastSquaresFit(ptlist.begin(), ptlist.end())) {
//            std::cout << "successfully conduct the non linear least squares!" << std::endl;
            pSourceAdjust = currentC.GetAxisPosition1();
            pTargetAdjust = currentC.GetAxisPosition2();
            radiusAdjust = currentC.GetRadius();
        }

        // truncate the new cylinder to the points
        Vector3D axis = pTargetAdjust - pSourceAdjust;
        Vector3D cDirSource = pSource - pSourceAdjust;
        Vector3D cDirTarget = pTarget - pSourceAdjust;
        axis.normalize();
        float nLengthSource = cDirSource.normalize();
        float nLengthTarget = cDirTarget.normalize();
        double alphaSource = Vector3D::dotProduct(axis, cDirSource);
        double alphaTarget = Vector3D::dotProduct(axis, cDirTarget);
        Vector3D pSourceNew = pSourceAdjust + nLengthSource * alphaSource * axis;
        Vector3D pTargetNew = pSourceAdjust + nLengthTarget * alphaTarget * axis;

        skeleton_[sourceV].coords = easy3d::vec3(pSourceNew.x, pSourceNew.y, pSourceNew.z);
        skeleton_[targetV].coords = easy3d::vec3(pTargetNew.x, pTargetNew.y, pTargetNew.z);
        skeleton_[trunkE].nRadius = radiusAdjust;
        TrunkRadius_ = radiusAdjust;

        return;
    } else {
        std::cout << "the non linear least squares is unsuccessful!" << std::endl;
        return;
    }
}


/// --- SMOOTHING
bool CylFit::smooth_skeleton() {
    if (num_edges(skeleton_) < 2) {
        std::cout << "skeleton does not exist!" << std::endl;
        return false;
    }

    skeleton_smooth_.clear();

//    std::cout << "smoothing skeleton..." << std::endl;

    // get paths
    std::vector<Path> pathList;
    get_graph_for_smooth(pathList);

//    std::cout << "found graph for smooth, num paths: " << pathList.size() << std::endl;


    // for each path get its coordinates and generate a smooth curve
    for (std::size_t n_path = 0; n_path < pathList.size(); ++n_path) {
//        std::cout << "- path " << n_path << ", size: " << pathList[n_path].size() << std::endl;

        Path currentPath = pathList[n_path];
        std::vector<easy3d::vec3> interpolatedPoints;
        std::vector<double> interpolatedRadii;
        static int numOfSlices = 20; //3 //20
        std::vector<int> numOfSlicesCurrent;
        // retrieve the current path and its vertices
        for (std::size_t n_node = 0; n_node < currentPath.size() - 1; ++n_node) {
//            std::cout << "\tnode " << n_node << std::endl;

            VertexDescriptorGraphB sourceV = currentPath[n_node];
            VertexDescriptorGraphB targetV = currentPath[n_node + 1];
            easy3d::vec3 pSource = skeleton_[sourceV].coords;
            easy3d::vec3 pTarget = skeleton_[targetV].coords;
            float branchlength = easy3d::distance(pSource, pTarget);
            numOfSlicesCurrent.push_back(std::max(static_cast<int>(branchlength * numOfSlices), 2));

            // compute the tangents
            easy3d::vec3 tangentOfSorce;
            easy3d::vec3 tangentOfTarget;
            // if the source vertex is the root
            if (sourceV == skeleton_[sourceV].parent)
                tangentOfSorce = (pTarget - pSource).normalize();
            else {
                VertexDescriptorGraphB parentOfSource = skeleton_[sourceV].parent;
                tangentOfSorce = (pTarget - skeleton_[parentOfSource].coords).normalize();
            }
            // if the target vertex is leaf
            if ((out_degree(targetV, skeleton_) == 1) && (targetV != skeleton_[targetV].parent))
                tangentOfTarget = (pTarget - pSource).normalize();
            else {
                VertexDescriptorGraphB childOfTarget = currentPath[n_node + 2];
                tangentOfTarget = (skeleton_[childOfTarget].coords - pSource).normalize();
            }

            tangentOfSorce *= branchlength;
            tangentOfTarget *= branchlength;

            //fit hermite curve
            easy3d::vec3 A = tangentOfTarget + tangentOfSorce + 2 * (pSource - pTarget);
            easy3d::vec3 B = 3 * (pTarget - pSource) - 2 * tangentOfSorce - tangentOfTarget;
            easy3d::vec3 C = tangentOfSorce;
            easy3d::vec3 D = pSource;
            EdgeDescriptorGraphB currentE = edge(sourceV, targetV, skeleton_).first;
            double sourceRadius = skeleton_[currentE].nRadius;
            double targetRadius = sourceRadius;
            VertexDescriptorGraphB ParentVert = skeleton_[sourceV].parent;
            if (ParentVert != sourceV) {
                EdgeDescriptorGraphB ParentEdge = edge(ParentVert, sourceV, skeleton_).first;
                sourceRadius = skeleton_[ParentEdge].nRadius;
            }
            double deltaOfRadius = (sourceRadius - targetRadius) / numOfSlicesCurrent[numOfSlicesCurrent.size() - 1];
            //generate interpolated points
            for (std::size_t n = 0; n < numOfSlicesCurrent[numOfSlicesCurrent.size() - 1]; ++n) {
                double t = static_cast<double>(static_cast<double>(n) /
                                               numOfSlicesCurrent[numOfSlicesCurrent.size() - 1]);
                easy3d::vec3 point = A * t * t * t + B * t * t + C * t + D;

                if (n == 0) {
                    interpolatedPoints.push_back(point);
                    interpolatedRadii.push_back(sourceRadius - n * deltaOfRadius);
                } else {
                    const easy3d::vec3 &prev = interpolatedPoints.back();
                    if (distance2(prev, point) >
                        easy3d::epsilon<float>() * 10) { // in case of duplicated points (tiny cylinder)
                        interpolatedPoints.push_back(point);
                        interpolatedRadii.push_back(sourceRadius - n * deltaOfRadius);
                    }
                }
            }
        }
        //push back the last vertex
        VertexDescriptorGraphB endV = currentPath.back();
        const easy3d::vec3 &prev = interpolatedPoints.back();
        const easy3d::vec3 &point = skeleton_[endV].coords;
        if (distance2(prev, point) > easy3d::epsilon<float>() * 10) { // in case of duplicated points (tiny cylinder)
            interpolatedPoints.push_back(point);
            interpolatedRadii.push_back(0);
        }

//        std::cout << "- pushed back last vertex" << std::endl;

        if (interpolatedPoints.size() < 2)
            continue; // Too few points to construct a cylinder

        // add vertices
        std::vector<VertexDescriptorGraphB> vertices;
        for (std::size_t np = 0; np < interpolatedPoints.size(); np++) {
            SGraphVertexPropB vp;
            vp.coords = interpolatedPoints[np];
            vp.radius = interpolatedRadii[np];
            VertexDescriptorGraphB v = add_vertex(vp, skeleton_smooth_);
            vertices.push_back(v);
        }

//        std::cout << "- added verts" << std::endl;

        // add edges
        for (std::size_t np = 0; np < vertices.size() - 1; np++) {
            add_edge(vertices[np], vertices[np + 1], SGraphEdgePropB(), skeleton_smooth_);
        }

//        std::cout << "- added edges" << std::endl;
    }

//    std::cout << "set curves" << std::endl;

    return true;
}


void CylFit::get_graph_for_smooth(std::vector<Path> &pathList) {
    pathList.clear();
    Path currentPath;
    int cursor = 0;
    //insert the root vertex to the current path
    currentPath.push_back(RootV_);
    pathList.push_back(currentPath);
    //retrieve the path list
    while (cursor < pathList.size()) {
        currentPath = pathList[cursor];
        VertexDescriptorGraphB endV = currentPath.back();
        // if the current path has reached the leaf
        if ((out_degree(endV, skeleton_) == 1) && (endV != skeleton_[endV].parent))
            cursor++;
        else {
            //find the fastest child vertex
            double maxR = -1;
            int isUsed = -1;
            VertexDescriptorGraphB fatestChild;
            std::vector<VertexDescriptorGraphB> notFastestChildren;
            std::pair<AdjacencyIteratorGraphB , AdjacencyIteratorGraphB > adjacencies = adjacent_vertices(endV, skeleton_);
            for (AdjacencyIteratorGraphB cIter = adjacencies.first; cIter != adjacencies.second; ++cIter) {
                if (*cIter != skeleton_[endV].parent) {
                    EdgeDescriptorGraphB currentE = edge(endV, *cIter,skeleton_).first;
                    double radius = skeleton_[currentE].nRadius;
                    if (maxR < radius) {
                        maxR = radius;
                        if (isUsed > -1)
                            notFastestChildren.push_back(fatestChild);
                        else
                            isUsed = 0;
                        fatestChild = *cIter;
                    } else
                        notFastestChildren.push_back(*cIter);
                }
            }

            // organize children vertices into a new path
            for (int nChild = 0; nChild < notFastestChildren.size(); ++nChild) {
                Path newPath;
                newPath.push_back(endV);
                newPath.push_back(notFastestChildren[nChild]);
                pathList.push_back(newPath);
            }
            //put the fatest children into the path list
            pathList[cursor].push_back(fatestChild);
        }
    }

    return;
}


/// --- MESH
bool CylFit::extract_branch_surfaces(easy3d::SurfaceMesh *mesh_result) {
    const std::vector<Branch> &branches = get_branches_parameters();
    if (branches.empty())
        return false;

    static const int slices = 10;
    for (const auto &branch : branches)
        add_generalized_cylinder_to_model(mesh_result, branch, slices);



//    for (auto f : mesh_result->faces()){
//        if (mesh_result->is_degenerate(f)){
//            mesh_result->delete_face(f);
//        }
//    }
//
//    mesh_result->collect_garbage();

    // remove isolated vertices
    for (auto v : mesh_result->vertices()) {
        if (mesh_result->is_isolated(v))
            mesh_result->delete_vertex(v);
    }
    mesh_result->collect_garbage();



    return true;
}


std::vector<CylFit::Branch> CylFit::get_branches_parameters() const {
    std::vector<CylFit::Branch> branches;

    GraphB &graph = *(const_cast<GraphB *>(&skeleton_smooth_));

    if (boost::num_edges(skeleton_smooth_) == 0)
        return branches;

    //-----------------------------------------------------------------------
    //  traverse all the vertices of a graph
    //-----------------------------------------------------------------------
    std::pair<VertexIteratorGraphB, VertexIteratorGraphB> vi = boost::vertices(graph);
    for (VertexIteratorGraphB vit = vi.first; vit != vi.second; ++vit) {
        VertexDescriptorGraphB cur_vd = *vit;
        SGraphVertexPropB &vp = graph[cur_vd];
        vp.visited = false;
    }

    for (VertexIteratorGraphB vit = vi.first; vit != vi.second; ++vit) {
        VertexDescriptorGraphB cur_vd = *vit;
        SGraphVertexPropB &vp = graph[cur_vd];
        auto deg = boost::degree(cur_vd, graph);
        if (vp.visited)
            continue;
        if (deg != 1)
            continue;

        Branch branch;
        vp.visited = true;

        if (branch.points.empty() || easy3d::distance(branch.points.back(), vp.coords) >= easy3d::epsilon<float>()) {
            branch.points.push_back(vp.coords);
            branch.radii.push_back(vp.radius);
        }

        bool reached_end = false;
        do {
            std::pair<AdjacencyIteratorGraphB , AdjacencyIteratorGraphB > adj_v_iter = boost::adjacent_vertices(cur_vd, skeleton_smooth_);
            for (AdjacencyIteratorGraphB  ait = adj_v_iter.first; ait != adj_v_iter.second; ++ait) {
                VertexDescriptorGraphB next_vd = *ait;

                SGraphVertexPropB &next_vp = graph[next_vd];
                if (!next_vp.visited) {

                    if (branch.points.empty() ||
                        easy3d::distance(branch.points.back(), next_vp.coords) >= easy3d::epsilon<float>()) {
                        branch.points.push_back(next_vp.coords);
                        branch.radii.push_back(next_vp.radius);
                    }

                    cur_vd = next_vd;
                    next_vp.visited = true;

                    if (boost::degree(cur_vd, graph) == 1) {
                        reached_end = true;
                        break;
                    }
                }
            }
        } while (!reached_end);

        branches.push_back(branch);
    }

    return branches;
}


void CylFit::add_generalized_cylinder_to_model(easy3d::SurfaceMesh *mesh, const Branch &branch, int slices) {
    const std::vector<double> &radius = branch.radii;
    const std::vector<easy3d::vec3> &points = branch.points;

    if (points.size() < 2) {
        //std::cerr << "two few points to represent a generalized cylinder" << std::endl;
        return;
    }
    typedef std::vector<easy3d::SurfaceMesh::Vertex> CrossSection;
    std::vector<CrossSection> crosssections;
    easy3d::vec3 perp;
    for (std::size_t np = 0; np < points.size() - 1; np++) {
        easy3d::vec3 s = points[np];
        easy3d::vec3 t = points[np + 1];
        if (has_nan(s) || has_nan(t))
            std::cerr << "file: " << __FILE__ << "\t" << "line: " << __LINE__ << "\n"
                      << "\ts: " << s << ";  t: " << t << std::endl;
        double r = radius[np];

        //find a vector perpendicular to the direction
        const easy3d::vec3 offset = t - s;
        const easy3d::vec3 axis = normalize(offset);
        if (np == 0) {
            easy3d::vec3 tmp = orthogonal(axis);
            tmp.normalize();
            perp = tmp;
            if (has_nan(perp))
                std::cerr << "file: " << __FILE__ << "\t" << "line: " << __LINE__ << "\n"
                          << "\taxis: " << axis << std::endl
                          << "\tperp: " << perp << std::endl;
        } else {
            const easy3d::vec3 p = easy3d::Plane3(s, axis).projection(s + perp);
            perp = p - s;
            perp.normalize();
            if (has_nan(perp))
                std::cerr << "file: " << __FILE__ << "\t" << "line: " << __LINE__ << "\n"
                          << "\ts:    " << s << std::endl
                          << "\taxis: " << axis << std::endl
                          << "\tperp: " << perp << std::endl;
        }

        const easy3d::vec3 p = s + perp * r;
        const double angle_interval = 2.0 * M_PI / slices;
        //find the points for all slices
        CrossSection cs;
        for (int sli = 0; sli < slices; ++sli) {
            double angle = sli * angle_interval;
            const easy3d::vec3 a = s + easy3d::mat4::rotation(axis, static_cast<float>(angle)) * (p - s);
            easy3d::SurfaceMesh::Vertex v = mesh->add_vertex(a);
            cs.push_back(v);
        }
        crosssections.push_back(cs);
    }

#if 0
    // the last vertex
    const vec3& t = points.back();
    SurfaceMesh::Vertex v = mesh->add_vertex(t);
    CrossSection cs;
    for (int sli = 0; sli < slices; ++sli)
        cs.push_back(v);
    crosssections.push_back(cs);
#endif

    if (crosssections.size() < 2)
        return;

    for (std::size_t nx = 0; nx < crosssections.size() - 1; ++nx) {
        const CrossSection &cs_curr = crosssections[nx];
        const CrossSection &cs_next = crosssections[nx + 1];
        for (std::size_t ny = 0; ny < cs_curr.size(); ++ny) {
            mesh->add_triangle(cs_curr[ny], cs_curr[(ny + 1) % cs_curr.size()], cs_next[ny]);
            mesh->add_triangle(cs_next[ny], cs_curr[(ny + 1) % cs_curr.size()], cs_next[(ny + 1) % cs_curr.size()]);
        }
    }
}