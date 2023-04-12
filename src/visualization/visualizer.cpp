// Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
//
// GNU-GPL licence that follows one of libQGLViewer.

#include "./visualizer.h"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <limits>
#include <string>
#include <math.h>
#include <vector>

namespace depth_clustering {

using std::array;
using std::string;
using std::to_string;
using std::vector;

using std::lock_guard;
using std::map;
using std::mutex;
using std::string;
using std::thread;
using std::unordered_map;
using std::vector;


static vector<array<int, 3>> COLORS;

Visualizer::Visualizer(QWidget* parent)
    : QGLViewer(parent), AbstractClient<Cloud>(), _updated{false} {
  _cloud_obj_storer.SetUpdateListener(this);
}

void Visualizer::draw() {
  PubCube();
}

void Visualizer::init() {
  setSceneRadius(100.0);
  camera()->showEntireScene();
  glDisable(GL_LIGHTING);
}

void Visualizer::DrawCloud(const Cloud& cloud) {
  glPushMatrix();
  glBegin(GL_POINTS);
  glColor3f(1.0f, 1.0f, 1.0f);
  for (const auto& point : cloud.points()) {
    glVertex3f(point.x(), point.y(), point.z());
  }
  glEnd();
  glPopMatrix();
}

void Visualizer::DrawCube(const Eigen::Vector3f& center,
                          const Eigen::Vector3f& scale) {

  glPushMatrix();
  glTranslatef(center.x(), center.y(), center.z());
  glScalef(scale.x(), scale.y(), scale.z());
  float volume = scale.x() * scale.y() * scale.z();
    if (volume < 30.0f && scale.x() < 6 && scale.y() < 6 && scale.z() < 6) {
      glColor3f(0.0f, 0.2f, 0.8f);
      glLineWidth(4.0f);
      } 
    else {
      glColor3f(0.3f, 0.3f, 0.3f);
      glLineWidth(1.0f);
    }
  // }
  glBegin(GL_LINE_STRIP);

  // Bottom of Box
  glVertex3f(-0.5, -0.5, -0.5);
  glVertex3f(-0.5, -0.5, 0.5);
  glVertex3f(0.5, -0.5, 0.5);
  glVertex3f(0.5, -0.5, -0.5);
  glVertex3f(-0.5, -0.5, -0.5);

  // Top of Box
  glVertex3f(-0.5, 0.5, -0.5);
  glVertex3f(-0.5, 0.5, 0.5);
  glVertex3f(0.5, 0.5, 0.5);
  glVertex3f(0.5, 0.5, -0.5);
  glVertex3f(-0.5, 0.5, -0.5);

  glEnd();

  glBegin(GL_LINES);
  // For the Sides of the Box

  glVertex3f(-0.5, 0.5, -0.5);
  glVertex3f(-0.5, -0.5, -0.5);

  glVertex3f(-0.5, -0.5, 0.5);
  glVertex3f(-0.5, 0.5, 0.5);

  glVertex3f(0.5, -0.5, 0.5);
  glVertex3f(0.5, 0.5, 0.5);

  glVertex3f(0.5, -0.5, -0.5);
  glVertex3f(0.5, 0.5, -0.5);

  glEnd();
  glPopMatrix();
}

Visualizer::~Visualizer() {}


void Visualizer::PubCube(){
  lock_guard<mutex> guard(_cloud_mutex);
  DrawCloud(_cloud);
  visualization_msgs::MarkerArray bbox_markers;
  bbox_markers.markers.clear();
  int clusterId = 0;
  for (const auto& kv : _cloud_obj_storer.object_clouds()) 
  {
    visualization_msgs::Marker marker;
    const auto& cluster = kv.second;
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    Eigen::Vector3f extent = Eigen::Vector3f::Zero();
    
    Eigen::Vector3f max_point(std::numeric_limits<float>::lowest(),
                              std::numeric_limits<float>::lowest(),
                              std::numeric_limits<float>::lowest());
    Eigen::Vector3f min_point(std::numeric_limits<float>::max(),
                              std::numeric_limits<float>::max(),
                              std::numeric_limits<float>::max());
    
    for (const auto& point : cluster.points()) {
      center = center + point.AsEigenVector();
      min_point << std::min(min_point.x(), point.x()),
          std::min(min_point.y(), point.y()),
          std::min(min_point.z(), point.z());
      max_point << std::max(max_point.x(), point.x()),
          std::max(max_point.y(), point.y()),
          std::max(max_point.z(), point.z());
    }
    center /= cluster.size();
    if (min_point.x() < max_point.x()) {
      extent = max_point - min_point;
    }
    float volume = extent.x() * extent.y() * extent.z();
    // DrawCube(center, extent);
    if (extent.z() > 0.3 && extent.x() < 2)
    // if (center.z() < 0.6 && extent.z() > 0.25 && extent.z() < 2.9 && extent.y() > 0.2  && fabs(extent.z() - extent.y() ) < 1.4  && extent.x() - extent.z() < 1.5)
    {

    DrawCube(center, extent);
    
    marker.header.frame_id = "rslidar";
    marker.header.stamp = ros::Time::now();
    marker.id = clusterId;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.1);
    marker.pose.position.x = center.x();
    marker.pose.position.y = center.y();
    marker.pose.position.z = center.z();
   
    marker.pose.orientation.w = 1.0;
    marker.scale.x = extent.x();
    marker.scale.y = extent.y();
    marker.scale.z = extent.z(); 
    marker.color.a = 1;
    marker.color.r = 0.1;
    marker.color.g = 0.1;
    marker.color.b = 0.9;
    bbox_markers.markers.push_back(marker);
    clusterId += 1;
    }
  }
  
 
  pub_bbox.publish(bbox_markers);
   
 
  bbox_markers.markers.clear();
}
void Visualizer::OnNewObjectReceived(const Cloud& cloud, const int) {
  lock_guard<mutex> guard(_cloud_mutex);
  _cloud = cloud;
}

void Visualizer::onUpdate() { this->update(); }

unordered_map<uint16_t, Cloud> ObjectPtrStorer::object_clouds() const {
  lock_guard<mutex> guard(_cluster_mutex);
  return _obj_clouds;
}

void ObjectPtrStorer::OnNewObjectReceived(
    const unordered_map<uint16_t, Cloud>& clouds, const int) {

  lock_guard<mutex> guard(_cluster_mutex);
  _obj_clouds = clouds;

  if (_update_listener) {
    _update_listener->onUpdate();
  }
}

}  // namespace depth_clustering