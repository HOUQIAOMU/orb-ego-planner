#include "test_support.h"
#include <ego_planner/bspline.h>
#include <ego_planner/polynomial_traj.h>
#include <cmath>
#include <limits>
using namespace ego_planner;
using namespace planning_test;
int main(){
 try{
  Eigen::MatrixXd points(3,7);
  for(int i=0;i<7;++i) points.col(i)=Eigen::Vector3d(i,2*i,-i);
  Bspline line(points,3,.5);
  CHECK(std::abs(line.duration()-2.0)<1e-12);
  CHECK((line.evaluate(0)-Eigen::Vector3d(1,2,-1)).norm()<1e-12);
  CHECK((line.evaluate(2)-Eigen::Vector3d(5,10,-5)).norm()<1e-12);
  CHECK((line.derivative().evaluate(.7)-Eigen::Vector3d(2,4,-2)).norm()<1e-12);
  CHECK(line.derivative().derivative().evaluate(.7).norm()<1e-10);
  CHECK((line.evaluate(-1)-line.evaluate(0)).norm()<1e-12);
  CHECK((line.evaluate(100)-line.evaluate(2)).norm()<1e-12);
  auto slow=line.scaledTime(2);
  CHECK((slow.evaluate(1.4)-line.evaluate(.7)).norm()<1e-10);
  CHECK((slow.derivative().evaluate(1.4)-line.derivative().evaluate(.7)/2).norm()<1e-10);
  CHECK(std::abs(line.normBounds().first-std::sqrt(24.0))<1e-10);
  // 分量都小于5，但向量范数大于4；防止误用逐轴上限。
  CHECK(line.normBounds().first>4.0);
  for(int i=0;i<7;++i) points.col(i)=Eigen::Vector3d(i*i*i,std::sin(i),i*i);
  Bspline curve(points,3,.4);
  const double t=.63,h=1e-5;
  CHECK(((curve.evaluate(t+h)-curve.evaluate(t-h))/(2*h)-curve.derivative().evaluate(t)).norm()<1e-5);
  CHECK(((curve.evaluate(t+h)-2*curve.evaluate(t)+curve.evaluate(t-h))/(h*h)-curve.derivative().derivative().evaluate(t)).norm()<.002);
  auto slowed=curve.scaledTime(3);
  CHECK((slowed.derivative().derivative().evaluate(t*3)-curve.derivative().derivative().evaluate(t)/9).norm()<1e-9);
  std::vector<Eigen::Vector3d> samples;
  for(int i=0;i<8;++i) samples.emplace_back(i*.2,0,0);
  const std::array<Eigen::Vector3d,4> boundary={Eigen::Vector3d(1,0,0),Eigen::Vector3d(1,0,0),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()};
  auto fit=Bspline::Parameterize(samples,boundary,.2);
  CHECK((fit.evaluate(.6)-Eigen::Vector3d(.6,0,0)).norm()<1e-9);
  CHECK((fit.derivative().evaluate(0)-Eigen::Vector3d(1,0,0)).norm()<1e-9);
  auto poly=PolynomialTraj::Quintic(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d(1,2,3),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),2);
  CHECK(poly.evaluate(0).norm()<1e-12);
  CHECK((poly.evaluate(2)-Eigen::Vector3d(1,2,3)).norm()<1e-12);
  CHECK((poly.evaluate(1)-Eigen::Vector3d(.5,1,1.5)).norm()<1e-12);
  CHECK((poly.evaluate(1,1)-Eigen::Vector3d(.9375,1.875,2.8125)).norm()<1e-12);
  CHECK(poly.evaluate(0,1).norm()<1e-12 && poly.evaluate(2,1).norm()<1e-12);
  CHECK(poly.evaluate(0,2).norm()<1e-12 && poly.evaluate(2,2).norm()<1e-12);
  const Eigen::Vector3d p0(1,-2,.3),v0(.4,.2,-.1),a0(.2,-.3,.1),p1(2,3,1),v1(-.1,.3,.4),a1(.1,.2,.3);
  auto general=PolynomialTraj::Quintic(p0,v0,a0,p1,v1,a1,1.7);
  CHECK((general.evaluate(0)-p0).norm()<1e-12);
  CHECK((general.evaluate(1.7)-p1).norm()<1e-10);
  CHECK((general.evaluate(0,1)-v0).norm()<1e-10);
  CHECK((general.evaluate(1.7,1)-v1).norm()<1e-10);
  CHECK((general.evaluate(1.7,2)-a1).norm()<1e-10);
  Reject([&]{Bspline bad(points,3,0.0);});
  Reject([&]{Bspline bad(points,7,.1);});
  Reject([&]{line.evaluate(std::numeric_limits<double>::quiet_NaN());});
  Reject([&]{line.scaledTime(0);});
  Reject([&]{Bspline::Parameterize({},boundary,.1);});
  Reject([&]{PolynomialTraj::Quintic(p0,v0,a0,p1,v1,a1,0);});
  Reject([&]{poly.evaluate(0,4);});
  std::cout<<"PASS math: analytic curves, derivatives, scaling, 3D norms, boundaries, invalid inputs\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
