#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/joint/joint-translation.hpp>
#include <pinocchio/multibody/model.hpp>

#include <gtest/gtest.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/urdf.h>
#include "CartPoleUrdf.h"
#include "MimicJointUrdf.h"

TEST(testPinocchioInterface, buildFromXml) {
  auto pinocchio = ocs2::getPinocchioInterfaceFromUrdfString(cartPoleUrdf);
  std::cout << pinocchio;
}

TEST(testPinocchioInterface, mimicJointParsingIsOptIn) {
  const auto legacyInterface = ocs2::getPinocchioInterfaceFromUrdfString(mimicJointUrdf);
  const auto explicitLegacyInterface = ocs2::getPinocchioInterfaceFromUrdfString(mimicJointUrdf, false);
  const auto mimicInterface = ocs2::getPinocchioInterfaceFromUrdfString(mimicJointUrdf, true);

  EXPECT_EQ(legacyInterface.getModel().nq, 2);
  EXPECT_EQ(explicitLegacyInterface.getModel().nq, 2);
  EXPECT_EQ(mimicInterface.getModel().nq, 1);
  EXPECT_EQ(mimicInterface.getModel().nv, 1);
  ASSERT_EQ(mimicInterface.getModel().mimicking_joints.size(), 1);

  const auto mimicJointId = mimicInterface.getModel().getJointId("mimic_joint");
  EXPECT_EQ(mimicInterface.getModel().nqs[mimicJointId], 0);
  EXPECT_EQ(mimicInterface.getModel().nvs[mimicJointId], 0);

  const auto mimicCppAdInterface = mimicInterface.toCppAd();
  EXPECT_EQ(mimicCppAdInterface.getModel().nq, 1);
  EXPECT_EQ(mimicCppAdInterface.getModel().nv, 1);
  EXPECT_EQ(mimicCppAdInterface.getModel().mimicking_joints.size(), 1);
}

TEST(testPinocchioInterface, mimicJointFollowsPrimaryCoordinate) {
  auto pinocchioInterface = ocs2::getPinocchioInterfaceFromUrdfString(mimicJointUrdf, true);
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();

  ocs2::vector_t q(1);
  q << 0.2;
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  const auto mimicLinkId = model.getBodyId("mimic_link");
  EXPECT_NEAR(data.oMf[mimicLinkId].translation().x(), 0.7, 1e-12);
}

TEST(testPinocchioInterface, mimicJointParsingSupportsRootJoint) {
  pinocchio::JointModelTranslation rootJoint;
  const auto pinocchioInterface =
      ocs2::getPinocchioInterfaceFromUrdfString(mimicJointUrdf, rootJoint, true);

  EXPECT_EQ(pinocchioInterface.getModel().nq, 4);
  EXPECT_EQ(pinocchioInterface.getModel().nv, 4);
  EXPECT_EQ(pinocchioInterface.getModel().mimicking_joints.size(), 1);
}
