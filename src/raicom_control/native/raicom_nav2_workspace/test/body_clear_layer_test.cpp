#include <cassert>
#include <memory>
#include "body_clear_layer.hpp"

using raicom_nav2_workspace::body_contains;
using raicom_nav2_workspace::BodyClearLayer;
class StoredObstacles : public nav2_costmap_2d::Layer {
 public:
  void onInitialize() override {enabled_=true;current_=true;}
  void reset() override {}
  bool isClearable() override {return false;}
  void updateBounds(double,double,double,double*,double*,double*,double*) override {}
  void updateCosts(nav2_costmap_2d::Costmap2D &map,int x0,int y0,int x1,int y1) override {
    for (int y=y0;y<y1;++y) for (int x=x0;x<x1;++x)
      map.setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
  }
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<nav2_util::LifecycleNode>("body_clear_layer_test");
  nav2_costmap_2d::LayeredCostmap layers("map",false,true);
  layers.resizeMap(100,100,.02,-1.,-1.);
  std::vector<geometry_msgs::msg::Point> body(4);
  body[0].x=-.1;body[0].y=-.08;body[1].x=.14;body[1].y=-.08;
  body[2].x=.14;body[2].y=.08;body[3].x=-.1;body[3].y=.08;
  layers.setFootprint(body);
  auto group=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto stored=std::make_shared<StoredObstacles>();
  auto clear=std::make_shared<BodyClearLayer>();
  layers.addPlugin(stored);layers.addPlugin(clear);
  stored->initialize(&layers,"stored",nullptr,node,group);
  clear->initialize(&layers,"body",nullptr,node,group);
  assert(body_contains(0.,0.,body));
  assert(!body_contains(.14,0.,body)); // boundary retained
  assert(!body_contains(.140001,0.,body));
  assert(!body_contains(NAN,0.,body));
  using raicom_nav2_workspace::body_mask_contains;
  assert(body_mask_contains(.145,0.,body)); // 5 mm boundary included
  assert(!body_mask_contains(.145001,0.,body)); // no 5 cm/radial expansion
  assert(body_mask_contains(.143,.083,body)); // rounded corner < 5 mm
  assert(!body_mask_contains(.144,.084,body)); // corner > 5 mm
  auto *map=layers.getCostmap();
  auto cost=[&](double x,double y) {unsigned int i,j;assert(map->worldToMap(x,y,i,j));return map->getCost(i,j);};
  layers.updateMap(0.,0.,0.);
  assert(cost(0.,0.)==nav2_costmap_2d::FREE_SPACE);
  assert(cost(.13,.01)==nav2_costmap_2d::FREE_SPACE);
  assert(cost(.15,.01)!=nav2_costmap_2d::FREE_SPACE); // outside thin obstacle retained
  layers.updateMap(.4,0.,0.);
  assert(cost(0.,0.)==nav2_costmap_2d::LETHAL_OBSTACLE); // no free trail
  assert(cost(.4,0.)==nav2_costmap_2d::FREE_SPACE);
  layers.updateMap(.4,0.,1.5707963267948966);
  assert(cost(.51,.01)==nav2_costmap_2d::LETHAL_OBSTACLE); // rotated out of body
  assert(cost(.41,.11)==nav2_costmap_2d::FREE_SPACE);
  // A missing/invalid pose never clears arbitrary cells or claims current data.
  layers.updateMap(NAN,0.,0.);
  assert(!clear->isCurrent());
  assert(cost(.4,0.)==nav2_costmap_2d::LETHAL_OBSTACLE);
  rclcpp::shutdown();
}
