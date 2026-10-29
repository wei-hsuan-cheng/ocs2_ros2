#include <gtest/gtest.h>

#include <cmath>
#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_robotic_tools/common/So3Tangent.h>

namespace ocs2 {
namespace {

TEST(So3Tangent, InverseRightJacobian) {
  EXPECT_TRUE(so3RightJacobianInverse(Eigen::Vector3d::Zero().eval()).isIdentity(1e-14));
  for (const Eigen::Vector3d& phi : {Eigen::Vector3d(1e-8, -2e-8, 3e-8), Eigen::Vector3d(0.2, -0.4, 0.3),
                                    Eigen::Vector3d(2.8, 0.2, -0.1)}) {
    const double theta = phi.norm();
    const Eigen::Matrix3d skew = skewSymmetricMatrix(phi);
    const double a = theta < 1e-4 ? 0.5 - theta * theta / 24.0 : (1.0 - std::cos(theta)) / (theta * theta);
    const double b = theta < 1e-4 ? 1.0 / 6.0 - theta * theta / 120.0
                                  : (theta - std::sin(theta)) / (theta * theta * theta);
    const Eigen::Matrix3d right = Eigen::Matrix3d::Identity() - a * skew + b * skew * skew;
    EXPECT_TRUE((right * so3RightJacobianInverse(phi)).isIdentity(1e-10));
  }
}

TEST(So3Tangent, GeneratedCompleteFlowAtZero) {
  CppAdInterface flow([](const ad_vector_t& x, ad_vector_t& y) {
    const Eigen::Matrix<ad_scalar_t, 3, 1> phi = x.head<3>();
    y = so3RightJacobianInverse(phi) * x.tail<3>();
  }, 6, "so3_complete_flow", "/tmp/ocs2_tangent_math_tests");
  flow.createModels(CppAdInterface::ApproximationOrder::Second, false);
  for (double scale : {0.0, 1e-20, 1e-10, 1.0}) {
    vector_t x = vector_t::Zero(6);
    const Eigen::Vector3d omega = scale * Eigen::Vector3d(0.3, -0.2, 0.4);
    x.tail<3>() = omega;
    const auto jacobian = flow.getJacobian(x);
    EXPECT_TRUE(flow.getFunctionValue(x).isApprox(omega, 1e-12));
    ASSERT_TRUE(jacobian.allFinite());
    EXPECT_TRUE(jacobian.rightCols(3).isIdentity(1e-12));
    EXPECT_LT((jacobian.leftCols(3) + 0.5 * skewSymmetricMatrix(omega)).norm(), 1e-12);
    for (size_t row = 0; row < 3; ++row) EXPECT_TRUE(flow.getHessian(row, x).allFinite());
  }
}

TEST(So3Tangent, GeneratedAtan2AxesAndQuadrants) {
  CppAdInterface angle([](const ad_vector_t& x, ad_vector_t& y) {
    y.resize(1);
    y(0) = atan2(x(0), x(1));
  }, 2, "atan2_axes", "/tmp/ocs2_tangent_math_tests");
  angle.createModels(CppAdInterface::ApproximationOrder::Second, false);
  for (const Eigen::Vector2d& yx : {Eigen::Vector2d(0, 1), Eigen::Vector2d(1, 0), Eigen::Vector2d(0, -1),
                                   Eigen::Vector2d(-1, 0), Eigen::Vector2d(0.2, 0.8), Eigen::Vector2d(-0.2, 0.8),
                                   Eigen::Vector2d(0.2, -0.8), Eigen::Vector2d(-0.2, -0.8),
                                   Eigen::Vector2d(0.5, 0.5), Eigen::Vector2d(-0.5, -0.5)}) {
    const double y = yx(0), x = yx(1), r2 = yx.squaredNorm();
    const Eigen::RowVector2d expected(x / r2, -y / r2);
    Eigen::Matrix2d hessian;
    hessian << -2 * x * y, y * y - x * x, y * y - x * x, 2 * x * y;
    hessian /= r2 * r2;
    EXPECT_NEAR(angle.getFunctionValue(yx)(0), std::atan2(y, x), 1e-12);
    EXPECT_LT((angle.getJacobian(yx) - expected).norm(), 1e-10);
    EXPECT_LT((angle.getHessian(size_t(0), yx) - hessian).norm(), 1e-10);
  }
}

}  // namespace
}  // namespace ocs2
