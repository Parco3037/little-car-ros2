#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/footprint.hpp"

namespace raicom_nav2_workspace {
constexpr double body_margin_m=0.005; // Explicit operator request: 0.5 cm.
// Never clear the swept/future footprint: those cells may be real obstacles.
inline bool body_contains(double x, double y,
    const std::vector<geometry_msgs::msg::Point> &polygon) {
  if (!std::isfinite(x) || !std::isfinite(y) || polygon.size()<3) return false;
  bool inside=false;
  for (std::size_t i=0,j=polygon.size()-1;i<polygon.size();j=i++) {
    const auto &a=polygon[j],&b=polygon[i];
    const double cross=(b.x-a.x)*(y-a.y)-(b.y-a.y)*(x-a.x);
    if (std::abs(cross)<=1e-12 && x>=std::min(a.x,b.x) && x<=std::max(a.x,b.x) &&
        y>=std::min(a.y,b.y) && y<=std::max(a.y,b.y)) return false;
    if ((a.y>y)!=(b.y>y) && x<(b.x-a.x)*(y-a.y)/(b.y-a.y)+a.x) inside=!inside;
  }
  return inside;
}

inline bool body_mask_contains(double x,double y,
    const std::vector<geometry_msgs::msg::Point> &polygon) {
  if (!std::isfinite(x) || !std::isfinite(y) || polygon.size()<3) return false;
  if (body_contains(x,y,polygon)) return true;
  for (std::size_t i=0,j=polygon.size()-1;i<polygon.size();j=i++) {
    const auto &a=polygon[j],&b=polygon[i];
    const double dx=b.x-a.x,dy=b.y-a.y,len2=dx*dx+dy*dy;
    const double t=len2>0. ? std::clamp(((x-a.x)*dx+(y-a.y)*dy)/len2,0.,1.) : 0.;
    if (std::hypot(x-a.x-t*dx,y-a.y-t*dy)<=body_margin_m+1e-12) return true;
  }
  return false;
}

class BodyClearLayer : public nav2_costmap_2d::Layer {
 public:
  void onInitialize() override {enabled_=true;current_=true;}
  void reset() override {polygon_.clear();previous_.clear();current_=true;}
  bool isClearable() override {return false;}
  void updateBounds(double x,double y,double yaw,double *min_x,double *min_y,
      double *max_x,double *max_y) override {
    // Include the last body bounds so preceding layers restore cells that
    // have left the body. The exclusion must not leave a free-space trail.
    auto include=[&](const auto &points) {
      for (const auto &p:points) {
        *min_x=std::min(*min_x,p.x-body_margin_m);*min_y=std::min(*min_y,p.y-body_margin_m);
        *max_x=std::max(*max_x,p.x+body_margin_m);*max_y=std::max(*max_y,p.y+body_margin_m);
      }
    };
    include(previous_);polygon_.clear();
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw) && getFootprint().size()>=3)
      nav2_costmap_2d::transformFootprint(x,y,yaw,getFootprint(),polygon_);
    current_=polygon_.size()>=3;
    include(polygon_);previous_=polygon_;
  }
  void updateCosts(nav2_costmap_2d::Costmap2D &map,int min_i,int min_j,int max_i,int max_j) override {
    if (!current_ || polygon_.empty()) return;
    double left=polygon_[0].x,right=left,bottom=polygon_[0].y,top=bottom;
    for (const auto &p:polygon_) {
      left=std::min(left,p.x);right=std::max(right,p.x);
      bottom=std::min(bottom,p.y);top=std::max(top,p.y);
    }
    int x0,y0,x1,y1;
    map.worldToMapEnforceBounds(left-body_margin_m,bottom-body_margin_m,x0,y0);
    map.worldToMapEnforceBounds(right+body_margin_m,top+body_margin_m,x1,y1);
    for (int j=std::max(min_j,y0);j<std::min(max_j,y1+1);++j)
      for (int i=std::max(min_i,x0);i<std::min(max_i,x1+1);++i) {
        double x,y;map.mapToWorld(i,j,x,y);
        if (body_mask_contains(x,y,polygon_)) map.setCost(i,j,nav2_costmap_2d::FREE_SPACE);
      }
  }
 private:
  std::vector<geometry_msgs::msg::Point> polygon_,previous_;
};
}
