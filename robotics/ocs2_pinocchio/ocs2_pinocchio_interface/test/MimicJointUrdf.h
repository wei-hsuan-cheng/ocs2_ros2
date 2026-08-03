#pragma once

// clang-format off
/* Two serial prismatic joints where the second joint mimics the first. */
static constexpr auto mimicJointUrdf = R"(
<?xml version="1.0"?>
<robot name="mimic_joint_test">
  <link name="base"/>
  <link name="primary_link"/>
  <link name="mimic_link"/>

  <joint name="primary_joint" type="prismatic">
    <parent link="base"/>
    <child link="primary_link"/>
    <axis xyz="1 0 0"/>
    <limit effort="10" lower="-1" upper="1" velocity="1"/>
  </joint>

  <joint name="mimic_joint" type="prismatic">
    <parent link="primary_link"/>
    <child link="mimic_link"/>
    <axis xyz="1 0 0"/>
    <limit effort="10" lower="-1" upper="1" velocity="1"/>
    <mimic joint="primary_joint" multiplier="2.0" offset="0.1"/>
  </joint>
</robot>
)";  // clang-format on
