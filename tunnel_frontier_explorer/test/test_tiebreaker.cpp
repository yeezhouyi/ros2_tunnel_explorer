#include "tunnel_frontier_explorer/frontier_scorer.hpp"
#include "tunnel_frontier_explorer/tunnel_geometry_grid.hpp"
#include <gtest/gtest.h>

using tunnel_frontier_explorer::FrontierScorer;
using tunnel_frontier_explorer::FrontierScorerConfig;
using tunnel_frontier_explorer::ScoredGoal;
using tunnel_frontier_explorer::TunnelGrid;
using tunnel_frontier_explorer::TunnelGeometryGrid;

static TunnelGeometryGrid make_varying_geometry() {
  TunnelGrid g;
  g.width = 50; g.height = 50; g.resolution = 0.05;
  g.origin_x = -1.25; g.origin_y = -1.25;
  g.data.resize(2500, 0);
  for (int y = 0; y < 50; ++y)
    for (int x = 0; x < 50; ++x)
      g.data[y * 50 + x] = static_cast<int8_t>(10 + x + y);
  TunnelGeometryGrid geo;
  geo.distance_map = g;
  geo.risk_map = g;
  return geo;
}

static ScoredGoal make_goal(double x, double y, double score) {
  ScoredGoal sg;
  sg.cluster.representative_world = {x, y};
  sg.score = score;
  return sg;
}

TEST(Tiebreaker, DoesNotChangeClearWinner) {
  auto geo = make_varying_geometry();
  FrontierScorerConfig cfg; cfg.tiebreaker_epsilon = 0.10;
  FrontierScorer scorer(cfg);
  std::vector<ScoredGoal> scored;
  scored.push_back(make_goal(0.0, 0.0, 0.85));
  scored.push_back(make_goal(0.5, 0.5, 0.40));
  scorer.applyTunnelRiskTiebreaker(scored, geo);
  EXPECT_NEAR(scored[0].score, 0.85, 1e-6);
}

TEST(Tiebreaker, PromotesLowerRiskWithinEpsilon) {
  auto geo = make_varying_geometry();
  FrontierScorerConfig cfg; cfg.tiebreaker_epsilon = 0.10;
  FrontierScorer scorer(cfg);
  std::vector<ScoredGoal> scored;
  // (1,1) has higher risk than (0,0) in varying geometry
  scored.push_back(make_goal(1.0, 1.0, 0.85));
  scored.push_back(make_goal(0.0, 0.0, 0.82));
  scorer.applyTunnelRiskTiebreaker(scored, geo);
  EXPECT_NEAR(scored[0].score, 0.82, 1e-6);
}

TEST(Tiebreaker, FallbackMatchesStage3D) {
  FrontierScorerConfig cfg; cfg.tiebreaker_epsilon = 0.10;
  FrontierScorer scorer(cfg);
  std::vector<ScoredGoal> scored;
  scored.push_back(make_goal(0.0, 0.0, 0.85));
  scored.push_back(make_goal(0.5, 0.0, 0.40));
  auto original = scored;
  TunnelGeometryGrid invalid;
  scorer.applyTunnelRiskTiebreaker(scored, invalid);
  ASSERT_EQ(scored.size(), original.size());
  for (size_t i = 0; i < scored.size(); ++i)
    EXPECT_NEAR(scored[i].score, original[i].score, 1e-6);
}

TEST(Tiebreaker, SingleCandidateNoOp) {
  auto geo = make_varying_geometry();
  FrontierScorerConfig cfg;
  FrontierScorer scorer(cfg);
  std::vector<ScoredGoal> scored;
  scored.push_back(make_goal(0.0, 0.0, 0.85));
  scorer.applyTunnelRiskTiebreaker(scored, geo);
  EXPECT_EQ(scored.size(), 1u);
  EXPECT_NEAR(scored[0].score, 0.85, 1e-6);
}
