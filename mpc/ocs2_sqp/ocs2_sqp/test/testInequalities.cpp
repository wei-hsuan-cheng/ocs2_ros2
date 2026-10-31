#include <gtest/gtest.h>

#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_oc/test/testProblemsGeneration.h>
#include <ocs2_sqp/SqpSolver.h>

namespace ocs2 {
namespace {

TEST(SqpInequalities, FilterIncludesInequalityViolationsOnlyWhenRequested) {
  FilterLinesearch filter;
  filter.g_max = 0.01;
  PerformanceIndex baseline, trial;
  baseline.cost = 10.0;
  baseline.merit = 10.0;
  trial.cost = 0.0;
  trial.inequalityConstraintsSSE = 1.0;
  EXPECT_TRUE(filter.acceptStep(baseline, trial, -1.0).first);
  EXPECT_FALSE(filter.acceptStep(baseline, trial, -1.0, true).first);
  EXPECT_DOUBLE_EQ(FilterLinesearch::totalConstraintViolation(trial, true), 1.0);
  EXPECT_DOUBLE_EQ(FilterLinesearch::totalConstraintViolation(trial), 0.0);
}

TEST(HpipmInequalities, InitialStateEliminationAndUpperMaskReset) {
  std::vector<VectorFunctionLinearApproximation> dynamics(1, VectorFunctionLinearApproximation::Zero(1, 1, 1));
  dynamics[0].dfdx(0, 0) = dynamics[0].dfdu(0, 0) = 1.0;
  std::vector<ScalarFunctionQuadraticApproximation> cost(2);
  cost[0] = ScalarFunctionQuadraticApproximation::Zero(1, 1);
  cost[0].dfduu(0, 0) = 1.0;
  cost[0].dfdu(0) = -2.0;
  cost[1] = ScalarFunctionQuadraticApproximation::Zero(1, 0);
  cost[1].dfdxx(0, 0) = 0.001;
  std::vector<VectorFunctionLinearApproximation> bounds(2);
  bounds[0] = VectorFunctionLinearApproximation::Zero(1, 1, 1);
  bounds[0].f(0) = 0.5;
  bounds[0].dfdx(0, 0) = bounds[0].dfdu(0, 0) = -1.0;
  bounds[1] = VectorFunctionLinearApproximation::Zero(0, 1, 0);
  auto size = extractSizesFromProblem(dynamics, cost, &bounds);
  HpipmInterface qp(size);
  vector_array_t x, u;
  ASSERT_EQ(qp.solveWithInequalities(vector_t::Constant(1, 0.3), dynamics, cost, bounds, x, u),
            hpipm_status::SUCCESS);
  EXPECT_NEAR(u.front()(0), 0.2, 1e-6);  // x0 + u <= 0.5

  // Same dimensions, first use a one-sided lower bound, then a legacy equality.
  bounds[0].f(0) = -0.1;
  bounds[0].dfdx.setZero();
  bounds[0].dfdu(0, 0) = 1.0;
  ASSERT_EQ(qp.solveWithInequalities(vector_t::Zero(1), dynamics, cost, bounds, x, u), hpipm_status::SUCCESS);
  EXPECT_GT(u.front()(0), 1.0);
  ASSERT_EQ(qp.solve(vector_t::Zero(1), dynamics, cost, &bounds, x, u), hpipm_status::SUCCESS);
  EXPECT_NEAR(u.front()(0), 0.1, 1e-6);
}

TEST(SqpInequalities, ProjectedBoundsAndRejectionOfUnprojectedEqualities) {
  for (bool project : {true, false}) {
    SCOPED_TRACE(project);
    OptimalControlProblem problem;
    auto flow = VectorFunctionLinearApproximation::Zero(1, 1, 2);
    flow.dfdu(0, 0) = 1.0;
    problem.dynamicsPtr = getOcs2Dynamics(flow);
    auto cost = ScalarFunctionQuadraticApproximation::Zero(1, 2);
    cost.dfdxx(0, 0) = 0.001;
    cost.dfduu.setIdentity();
    problem.costPtr->add("tracking", getOcs2Cost(cost));
    auto equality = VectorFunctionLinearApproximation::Zero(1, 1, 2);
    equality.dfdu << 1.0, -1.0;
    problem.equalityConstraintPtr->add("coupledInputs", getOcs2Constraints(equality));
    auto bounds = VectorFunctionLinearApproximation::Zero(2, 1, 2);
    bounds.f.setConstant(0.25);
    bounds.dfdu.col(0) << 1.0, -1.0;
    problem.inequalityConstraintPtr->add("inputBounds", getOcs2Constraints(bounds));
    auto terminal = VectorFunctionLinearApproximation::Zero(1, 1, 0);
    terminal.f(0) = 0.02;
    terminal.dfdx(0, 0) = -1.0;
    problem.finalInequalityConstraintPtr->add("terminalBound", getOcs2StateOnlyConstraints(terminal));
    auto references = std::make_shared<ReferenceManager>(
        TargetTrajectories({0.0}, {vector_t::Zero(1)}, {vector_t::Constant(2, 2.0)}));
    sqp::Settings settings;
    settings.dt = 0.1;
    settings.sqpIteration = 2;
    settings.nThreads = 1;
    settings.enableLogging = false;
    settings.useFeedbackPolicy = false;
    settings.enforceInequalityConstraints = true;
    settings.projectStateInputEqualityConstraints = project;
    DefaultInitializer initializer(2);
    if (!project) {
      EXPECT_THROW((SqpSolver(settings, problem, initializer)), std::invalid_argument);
      continue;
    }
    SqpSolver solver(settings, problem, initializer);
    solver.setReferenceManager(references);
    ASSERT_NO_THROW(solver.run(0.0, vector_t::Zero(1), 0.3));
    const auto result = solver.primalSolution(0.3);
    for (const auto& input : result.inputTrajectory_) {
      ASSERT_TRUE(input.allFinite());
      EXPECT_LE(input(0), 0.250001);
      EXPECT_GE(input(0), -0.250001);
      EXPECT_NEAR(input(0), input(1), 1e-6);
    }
    EXPECT_LE(result.stateTrajectory_.back()(0), 0.020001);
    EXPECT_GT(result.stateTrajectory_.back()(0), 0.019);
  }
}

}  // namespace
}  // namespace ocs2
