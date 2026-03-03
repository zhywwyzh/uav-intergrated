#include <traj_utils/planning_visualization.h>

using std::cout;
using std::endl;
namespace ego_planner
{
  PlanningVisualization::PlanningVisualization(ros::NodeHandle &nh)
  {
    node = nh;
    goal_point_pub = nh.advertise<visualization_msgs::Marker>("goal_point", 2);
    global_list_pub = nh.advertise<visualization_msgs::Marker>("global_list", 2);
    init_list_pub = nh.advertise<visualization_msgs::Marker>("init_list", 2);
    initofinit_list_pub = nh.advertise<visualization_msgs::Marker>("init_of_init_list", 2);
    optimal_list_pub = nh.advertise<visualization_msgs::Marker>("optimal_list", 2);
    failed_list_pub = nh.advertise<visualization_msgs::Marker>("failed_list", 2);
    a_star_list_pub = nh.advertise<visualization_msgs::Marker>("a_star_list", 20);

    intermediate_pt0_pub = nh.advertise<visualization_msgs::Marker>("pt0_dur_opt", 10);
    intermediate_grad0_pub = nh.advertise<visualization_msgs::MarkerArray>("grad0_dur_opt", 10);
    intermediate_pt1_pub = nh.advertise<visualization_msgs::Marker>("pt1_dur_opt", 10);
    intermediate_grad1_pub = nh.advertise<visualization_msgs::MarkerArray>("grad1_dur_opt", 10);
    intermediate_grad_smoo_pub = nh.advertise<visualization_msgs::MarkerArray>("smoo_grad_dur_opt", 10);
    intermediate_grad_dist_pub = nh.advertise<visualization_msgs::MarkerArray>("dist_grad_dur_opt", 10);
    intermediate_grad_feas_pub = nh.advertise<visualization_msgs::MarkerArray>("feas_grad_dur_opt", 10);
    intermediate_grad_swarm_pub = nh.advertise<visualization_msgs::MarkerArray>("swarm_grad_dur_opt", 10);

    // visualization publishers
    frontier_pub = node.advertise<visualization_msgs::Marker>("frontier", 10000);
    relevent_pub = node.advertise<visualization_msgs::Marker>("relevent", 10000);
    hgrid_pub = node.advertise<visualization_msgs::Marker>("hgrid", 10000);
    text_pub = node.advertise<visualization_msgs::Marker>("hgrid_text", 10000);
    viewpoint_pub = node.advertise<visualization_msgs::Marker>("viewpoints_vis", 10000);
    topo_blacklist_pub_ = node.advertise<visualization_msgs::Marker>("topo_blacklist", 10000);
    hgrid_info_pub_ = node.advertise<visualization_msgs::MarkerArray>("hgrid_info", 1000);
    frontier_pub_   = node.advertise<visualization_msgs::MarkerArray>("frontier_info", 1000);
  }

  void PlanningVisualization::fillBasicInfo(visualization_msgs::Marker& mk, const Eigen::Vector3d& scale,
                                          const Eigen::Vector4d& color, const string& ns, const int& id,
                                          const int& shape) {
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = id;
  mk.ns = ns;
  mk.type = shape;

  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = scale[0];
  mk.scale.y = scale[1];
  mk.scale.z = scale[2];
}

void PlanningVisualization::fillGeometryInfo(visualization_msgs::Marker& mk,
                                             const vector<Eigen::Vector3d>& list) {
  geometry_msgs::Point pt;
  for (const auto & i : list) {
    pt.x = i(0);
    pt.y = i(1);
    pt.z = i(2);
    mk.points.push_back(pt);
  }
}

void PlanningVisualization::fillGeometryInfo(visualization_msgs::Marker& mk,
                                             const vector<Eigen::Vector3d>& list1,
                                             const vector<Eigen::Vector3d>& list2) {
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
}

void PlanningVisualization::drawSpheres(const vector<Eigen::Vector3d>& list, const double& scale,
                                        const Eigen::Vector4d& color, const string& ns, const int& id,
                                        const int& pub_id) 
{
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                visualization_msgs::Marker::SPHERE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  viewpoint_pub.publish(mk);

  // pub new marker
  fillGeometryInfo(mk, list);
  mk.action = visualization_msgs::Marker::ADD;
  viewpoint_pub.publish(mk);
  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawLinesByMarkerArray(visualization_msgs::MarkerArray &mk_array,
                                                   const vector<Eigen::Vector3d>& list1,
                                                   const vector<Eigen::Vector3d>& list2, const double& scale, const Eigen::Vector4d& color,
                                                   const string& ns, const int& id, const int& pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                visualization_msgs::Marker::LINE_LIST);
  if (list1.size() == 0) return;
  // pub new marker
  fillGeometryInfo(mk, list1, list2);
  mk.action = visualization_msgs::Marker::ADD;
  mk_array.markers.push_back(mk);
  hgrid_info_pub_.publish(mk_array);
  ros::Duration(0.0001).sleep();
}

void PlanningVisualization::drawExploreBoxByMarkerArray(visualization_msgs::MarkerArray &mk_array,
                                                        const Eigen::Vector3d &min_pt, const Eigen::Vector3d &max_pt,
                                                        const double &scale, const Eigen::Vector4d &color,
                                                        const std::string &ns, const int &id, const int &pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                visualization_msgs::Marker::LINE_LIST);
  //Define the vertices of the cube
  std::vector<Eigen::Vector3d> vertices = {
          Eigen::Vector3d(min_pt.x(), min_pt.y(), min_pt.z()),
          Eigen::Vector3d(max_pt.x(), min_pt.y(), min_pt.z()),
          Eigen::Vector3d(max_pt.x(), max_pt.y(), min_pt.z()),
          Eigen::Vector3d(min_pt.x(), max_pt.y(), min_pt.z()),
          Eigen::Vector3d(min_pt.x(), min_pt.y(), max_pt.z()),
          Eigen::Vector3d(max_pt.x(), min_pt.y(), max_pt.z()),
          Eigen::Vector3d(max_pt.x(), max_pt.y(), max_pt.z()),
          Eigen::Vector3d(min_pt.x(), max_pt.y(), max_pt.z())
  };
  std::vector<std::pair<int, int>> edges = {
          {0, 1}, {1, 2}, {2, 3}, {3, 0},
          {4, 5}, {5, 6}, {6, 7}, {7, 4},
          {0, 4}, {1, 5}, {2, 6}, {3, 7}
  };
  vector<Eigen::Vector3d> list1, list2;
  // loooop all edges
  for (const auto& edge : edges) {
    list1.push_back(vertices[edge.first]);
    list2.push_back(vertices[edge.second]);
  }
  fillGeometryInfo(mk, list1, list2);
  mk.action = visualization_msgs::Marker::ADD;
  mk_array.markers.push_back(mk);
  hgrid_info_pub_.publish(mk_array);
  ros::Duration(0.0001).sleep();
}

void PlanningVisualization::drawLines(const vector<Eigen::Vector3d>& list1,
    const vector<Eigen::Vector3d>& list2, const double& scale, const Eigen::Vector4d& color,
    const string& ns, const int& id, const int& pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
      visualization_msgs::Marker::LINE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  hgrid_pub.publish(mk);

  if (list1.size() == 0) return;

  // pub new marker
  fillGeometryInfo(mk, list1, list2);
  mk.action = visualization_msgs::Marker::ADD;
  hgrid_pub.publish(mk);
  ros::Duration(0.0001).sleep();
}

void PlanningVisualization::drawLines(const vector<Eigen::Vector3d>& list, const double& scale,
    const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id) 
{
  if (list.empty()) return;
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
      visualization_msgs::Marker::LINE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  hgrid_pub.publish(mk);

  if (list.size() == 0) return;

  // split the single list into two
  vector<Eigen::Vector3d> list1, list2;
  for (int i = 0; i < list.size() - 1; ++i) {
    list1.push_back(list[i]);
    list2.push_back(list[i + 1]);
  }

  // pub new marker
  fillGeometryInfo(mk, list1, list2);
  mk.action = visualization_msgs::Marker::ADD;
  hgrid_pub.publish(mk);
  ros::Duration(0.0001).sleep();
}

void PlanningVisualization::drawText(const Eigen::Vector3d& pos, const string& text,
    const double& scale, const Eigen::Vector4d& color, const string& ns, const int& id,
    const int& pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
      visualization_msgs::Marker::TEXT_VIEW_FACING);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  text_pub.publish(mk);
  ros::Duration(0.0001).sleep();

  // pub new marker
  mk.text = text;
  mk.pose.position.x = pos[0];
  mk.pose.position.y = pos[1];
  mk.pose.position.z = pos[2];
  mk.action = visualization_msgs::Marker::ADD;
  text_pub.publish(mk);
  ros::Duration(0.0001).sleep();
}

void PlanningVisualization::addHgridTextInfoMarkerToArray(visualization_msgs::MarkerArray &mk_array,
                                                          visualization_msgs::Marker &mk,
                                                          const Eigen::Vector3d &pos, const std::string &text, const int & id) {
  mk.pose.position.x = pos[0];
  mk.pose.position.y = pos[1];
  mk.pose.position.z = pos[2];
  mk.text            = text;
  mk.action          = visualization_msgs::Marker::ADD;
  mk.id              = id;
  mk_array.markers.push_back(mk);
}

void PlanningVisualization::drawHgridTextByMarkerArray(const visualization_msgs::MarkerArray &mk_array) {
  hgrid_info_pub_.publish(mk_array);
  ros::Duration(0.0001).sleep();
}

void PlanningVisualization::drawBlacklistText(const Eigen::Vector3d &pos, const std::string &text, const double &scale,
                                         const Eigen::Vector4d &color, const std::string &ns, const int &id,
                                         const int &pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                visualization_msgs::Marker::TEXT_VIEW_FACING);
  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  topo_blacklist_pub_.publish(mk);
  ros::Duration(0.0001).sleep();
  // pub new marker
  if (!text.empty()){
    mk.text = text;
    mk.pose.position.x = pos[0];
    mk.pose.position.y = pos[1];
    mk.pose.position.z = pos[2];
    mk.action = visualization_msgs::Marker::ADD;
    topo_blacklist_pub_.publish(mk);
    ros::Duration(0.0001).sleep();
  }
}

void PlanningVisualization::drawFrontierRangeBox(const Eigen::Vector3d &center, const Eigen::Vector3d &scale,
                                                  const Eigen::Vector4d &color, const std::string &ns,
                                                  const int &id, const int &pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, scale, color, ns, id, visualization_msgs::Marker::CUBE);
  mk.pose.position.x = center[0];
  mk.pose.position.y = center[1];
  mk.pose.position.z = center[2];
  mk.action = visualization_msgs::Marker::DELETE;
  relevent_pub.publish(mk);
  ros::Duration(0.0001).sleep();

  mk.action = visualization_msgs::Marker::ADD;
  frontier_pub.publish(mk);
  ros::Duration(0.0001).sleep();
}

void PlanningVisualization::deleteFrontierRangeBox(const Eigen::Vector3d &center, const Eigen::Vector3d &scale,
                                                   const Eigen::Vector4d &color, const std::string &ns,
                                                   const int &id, const int &pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, scale, color, ns, id, visualization_msgs::Marker::CUBE);
  mk.action = visualization_msgs::Marker::DELETE;
  relevent_pub.publish(mk);
  ros::Duration(0.0001).sleep();
}

void PlanningVisualization::drawHgridActiveInfoByMarkerArray(const visualization_msgs::MarkerArray &mk_array) {
  hgrid_info_pub_.publish(mk_array);
  ros::Duration(0.0001).sleep();
}

void PlanningVisualization::addHgridActiveInfoMarkerToArray(visualization_msgs::MarkerArray &mk_array,
                                                            visualization_msgs::Marker &mk,
                                                            const Eigen::Vector3d &center, const Eigen::Vector4d &color,
                                                            const int &id, const int &action) {
  mk.pose.position.x = center[0];
  mk.pose.position.y = center[1];
  mk.pose.position.z = center[2];
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.action = action;
  mk.id = id;
  mk_array.markers.push_back(mk);
}

void PlanningVisualization::drawBox(const Eigen::Vector3d& center, const Eigen::Vector3d& scale,
    const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, scale, color, ns, id, visualization_msgs::Marker::CUBE);
  mk.pose.position.x = center[0];
  mk.pose.position.y = center[1];
  mk.pose.position.z = center[2];
  mk.action = visualization_msgs::Marker::DELETE;
  relevent_pub.publish(mk);
  ros::Duration(0.0001).sleep();

  mk.action = visualization_msgs::Marker::ADD;
  relevent_pub.publish(mk);
  ros::Duration(0.0001).sleep();
}

void PlanningVisualization::deleteBox(const Eigen::Vector3d& center, const Eigen::Vector3d& scale,
    const Eigen::Vector4d& color, const string& ns, const int& id, const int& pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, scale, color, ns, id, visualization_msgs::Marker::CUBE);
  mk.action = visualization_msgs::Marker::DELETE;
  relevent_pub.publish(mk);

  ros::Duration(0.0001).sleep();
}

void PlanningVisualization::addFrontierCubesMarkerToArray(visualization_msgs::MarkerArray &mk_array,
                                                          visualization_msgs::Marker &mk,
                                                          const vector<Eigen::Vector3d> &list, const double& scale,
                                                          const Eigen::Vector4d& color, const string& ns, const int &id,
                                                          const int &action) {
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                visualization_msgs::Marker::CUBE_LIST);
  fillGeometryInfo(mk, list);
  mk.action = action;
  mk_array.markers.push_back(mk);
}

void PlanningVisualization::deleteAllFrontierCells() {
  visualization_msgs::Marker mk;
  visualization_msgs::MarkerArray mk_array;
  fillBasicInfo(mk, Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector4d(0, 0, 0, 0), "frontier_cell", 0,
                  visualization_msgs::Marker::CUBE_LIST);
  mk.action = visualization_msgs::Marker::DELETEALL;
  mk_array.markers.push_back(mk);
  frontier_pub_.publish(mk_array);
  ros::Duration(0.0001).sleep();
}

void PlanningVisualization::drawFrontierCubesByMarkerArray(const visualization_msgs::MarkerArray &mk_array) {
  frontier_pub_.publish(mk_array);
  ros::Duration(0.0001).sleep();
}

  void PlanningVisualization::drawCubes(const vector<Eigen::Vector3d>& list, const double& scale,
                                      const Eigen::Vector4d& color, const string& ns, const int& id,
                                      const int& pub_id) {
  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                visualization_msgs::Marker::CUBE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  frontier_pub.publish(mk);

  // pub new marker
  fillGeometryInfo(mk, list);
  mk.action = visualization_msgs::Marker::ADD;
  frontier_pub.publish(mk);
  ros::Duration(0.0005).sleep();
}

Eigen::Vector4d PlanningVisualization::getColor(const double& h, double alpha) {
  double h1 = h;
  if (h1 < 0.0 || h1 > 1.0) {
    std::cout << "h out of range" << std::endl;
    h1 = 0.0;
  }

  double lambda;
  Eigen::Vector4d color1, color2;
  if (h1 >= -1e-4 && h1 < 1.0 / 6) {
    lambda = (h1 - 0.0) * 6;
    color1 = Eigen::Vector4d(1, 0, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 1, 1);
  } else if (h1 >= 1.0 / 6 && h1 < 2.0 / 6) {
    lambda = (h1 - 1.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 0, 1, 1);
  } else if (h1 >= 2.0 / 6 && h1 < 3.0 / 6) {
    lambda = (h1 - 2.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 1, 1);
  } else if (h1 >= 3.0 / 6 && h1 < 4.0 / 6) {
    lambda = (h1 - 3.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 0, 1);
  } else if (h1 >= 4.0 / 6 && h1 < 5.0 / 6) {
    lambda = (h1 - 4.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 1, 0, 1);
  } else if (h1 >= 5.0 / 6 && h1 <= 1.0 + 1e-4) {
    lambda = (h1 - 5.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 0, 1);
  }

  Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
  fcolor(3) = alpha;

  return fcolor;
}
  // // real ids used: {id, id+1000}
  void PlanningVisualization::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                                Eigen::Vector4d color, int id, bool show_sphere /* = true */ )
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 2;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      if (show_sphere) sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    if (show_sphere) pub.publish(sphere);
    pub.publish(line_strip);
  }

  // real ids used: {id, id+1}
  void PlanningVisualization::generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                                       const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 3;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    array.markers.push_back(sphere);
    array.markers.push_back(line_strip);
  }

  // real ids used: {1000*id ~ (arrow nums)+1000*id}
  void PlanningVisualization::generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                                        const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    arrow.header.stamp = ros::Time::now();
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    // geometry_msgs::Point start, end;
    // arrow.points

    arrow.color.r = color(0);
    arrow.color.g = color(1);
    arrow.color.b = color(2);
    arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    arrow.scale.x = scale;
    arrow.scale.y = 2 * scale;
    arrow.scale.z = 2 * scale;

    geometry_msgs::Point start, end;
    for (int i = 0; i < int(list.size() / 2); i++)
    {
      // arrow.color.r = color(0) / (1+i);
      // arrow.color.g = color(1) / (1+i);
      // arrow.color.b = color(2) / (1+i);

      start.x = list[2 * i](0);
      start.y = list[2 * i](1);
      start.z = list[2 * i](2);
      end.x = list[2 * i + 1](0);
      end.y = list[2 * i + 1](1);
      end.z = list[2 * i + 1](2);
      arrow.points.clear();
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      arrow.id = i + id * 1000;

      array.markers.push_back(arrow);
    }
  }

  void PlanningVisualization::displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id)
  {
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = goal_point(0);
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = goal_point(2);

    goal_point_pub.publish(sphere);
  }

  void PlanningVisualization::displayGlobalPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (global_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0.5, 0.5, 1);
    displayMarkerList(global_list_pub, init_pts, scale, color, id);
  }

  void PlanningVisualization::displayMultiInitPathList(vector<vector<Eigen::Vector3d>> init_trajs, const double scale)
  {

    if (init_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    static int last_nums = 0;

    for ( int id=0; id<last_nums; id++ )
    {
      Eigen::Vector4d color(0, 0, 0, 0);
      vector<Eigen::Vector3d> blank;
      displayMarkerList(init_list_pub, blank, scale, color, id, false);
      ros::Duration(0.001).sleep();
    }
    last_nums = 0;

    for ( int id=0; id<(int)init_trajs.size(); id++ )
    {
      Eigen::Vector4d color(0, 0, 1, 0.7);
      displayMarkerList(init_list_pub, init_trajs[id], scale, color, id, false);
      ros::Duration(0.001).sleep();
      last_nums++;
    }

  }

  void PlanningVisualization::displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (init_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0, 1, 1);
    displayMarkerList(init_list_pub, init_pts, scale, color, id);
  }

  void PlanningVisualization::displayInitOfInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (initofinit_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0.2, 0.8, 1);
    displayMarkerList(initofinit_list_pub, init_pts, scale, color, id);
  }

  void PlanningVisualization::displayMultiOptimalPathList(vector<vector<Eigen::Vector3d>> optimal_trajs, const double scale) // zxzxzx
  {

    if (optimal_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    static int last_nums = 0;

    for ( int id=0; id<last_nums; id++ )
    {
      Eigen::Vector4d color(0, 0, 0, 0);
      vector<Eigen::Vector3d> blank;
      displayMarkerList(optimal_list_pub, blank, scale, color, id + 10, false);
      ros::Duration(0.001).sleep();
    }
    last_nums = 0;

    for ( int id=0; id<(int)optimal_trajs.size(); id++ )
    {
      Eigen::Vector4d color(1, 0, 0, 0.7);
      displayMarkerList(optimal_list_pub, optimal_trajs[id], scale, color, id + 10, false);
      ros::Duration(0.001).sleep();
      last_nums++;
    }

  }

  void PlanningVisualization::displayOptimalList(Eigen::MatrixXd optimal_pts, int id)
  {

    if (optimal_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> list;
    for (int i = 0; i < optimal_pts.cols(); i++)
    {
      Eigen::Vector3d pt = optimal_pts.col(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(1, 0, 0, 1);
    displayMarkerList(optimal_list_pub, list, 0.15, color, id);
  }

  void PlanningVisualization::displayFailedList(Eigen::MatrixXd failed_pts, int id)
  {

    if (failed_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> list;
    for (int i = 0; i < failed_pts.cols(); i++)
    {
      Eigen::Vector3d pt = failed_pts.col(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(0.3, 0, 0, 1);
    displayMarkerList(failed_list_pub, list, 0.15, color, id);
  }

  void PlanningVisualization::displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id /* = Eigen::Vector4d(0.5,0.5,0,1)*/)
  {

    if (a_star_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    int i = 0;
    vector<Eigen::Vector3d> list;

    Eigen::Vector4d color = Eigen::Vector4d(0.5 + ((double)rand() / RAND_MAX / 2), 0.5 + ((double)rand() / RAND_MAX / 2), 0, 1); // make the A star pathes different every time.
    double scale = 0.05 + (double)rand() / RAND_MAX / 10;

    for (auto block : a_star_paths)
    {
      list.clear();
      for (auto pt : block)
      {
        list.push_back(pt);
      }
      //Eigen::Vector4d color(0.5,0.5,0,1);
      displayMarkerList(a_star_list_pub, list, scale, color, id + i); // real ids used: [ id ~ id+a_star_paths.size() ]
      i++;
    }
  }

  void PlanningVisualization::displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::MarkerArray array;
    // clear
    pub.publish(array);

    generateArrowDisplayArray(array, list, scale, color, id);

    pub.publish(array);
  }

  void PlanningVisualization::displayIntermediatePt(std::string type, const Eigen::MatrixXd &pts, int id, Eigen::Vector4d color)
  {
    std::vector<Eigen::Vector3d> pts_;
    pts_.reserve(pts.cols());
    for ( int i=0; i<pts.cols(); i++ )
    {
      pts_.emplace_back(pts.col(i));
    }

    if ( !type.compare("0") )
    {
      displayMarkerList(intermediate_pt0_pub, pts_, 0.1, color, id);
    }
    else if ( !type.compare("1") )
    {
      displayMarkerList(intermediate_pt1_pub, pts_, 0.1, color, id);
    }
  }

  void PlanningVisualization::displayIntermediateGrad(std::string type, const Eigen::MatrixXd &pts, const Eigen::MatrixXd &grad, int id, Eigen::Vector4d color)
  {
    if ( pts.cols() != grad.cols() )
    {
      ROS_ERROR("pts.cols() != grad.cols()");
      return;
    }
    std::vector<Eigen::Vector3d> arrow_;
    arrow_.reserve(pts.cols()*2);
    if ( !type.compare("swarm") )
    {
      for ( int i=0; i<pts.cols(); i++ )
      {
        arrow_.emplace_back(pts.col(i));
        arrow_.emplace_back(grad.col(i));
      }
    }
    else
    {
      for ( int i=0; i<pts.cols(); i++ )
      {
        arrow_.emplace_back(pts.col(i));
        arrow_.emplace_back(pts.col(i)+grad.col(i));
      }
    }
    

    if ( !type.compare("grad0") )
    {
      displayArrowList(intermediate_grad0_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("grad1") )
    {
      displayArrowList(intermediate_grad1_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("dist") )
    {
      displayArrowList(intermediate_grad_dist_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("smoo") )
    {
      displayArrowList(intermediate_grad_smoo_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("feas") )
    {
      displayArrowList(intermediate_grad_feas_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("swarm") )
    {
      displayArrowList(intermediate_grad_swarm_pub, arrow_, 0.02, color, id);
    }
    
  }

  // PlanningVisualization::
} // namespace ego_planner