#include <catch2/catch_test_macros.hpp>
#include <ceres/ceres.h>
#include <gtsam/geometry/Pose2.h>
#include <Eigen/Core>
#include <cmath>

namespace {
struct Residual {
  template<class T> bool operator()(const T* const x,T* r) const {
    r[0]=x[0]-T(3.0); return true;
  }
};
}

TEST_CASE("Ceres, GTSAM, Catch2, Eigen load and execute") {
  Eigen::Vector2d v{1.,2.};
  REQUIRE(v.squaredNorm() == 5.0);
  gtsam::Pose2 a(0.,0.,0.), b(2.,0.,0.);
  REQUIRE(std::abs(a.between(b).x()-2.)<1e-12);
  double x=0.;
  ceres::Problem problem;
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Residual,1,1>(new Residual()),nullptr,&x);
  ceres::Solver::Options o;
  o.max_num_iterations=15;
  o.minimizer_progress_to_stdout=false;
  o.linear_solver_type=ceres::DENSE_QR;
  ceres::Solver::Summary summary;
  ceres::Solve(o,&problem,&summary);
  REQUIRE(summary.IsSolutionUsable());
  REQUIRE(std::abs(x-3.)<1e-5);
}
