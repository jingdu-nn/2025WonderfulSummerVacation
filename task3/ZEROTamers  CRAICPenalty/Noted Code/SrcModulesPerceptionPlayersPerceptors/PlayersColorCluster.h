#pragma once
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesImagePercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesPerceptorData.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CameraMatrix.h"
#include "Representations/Perception/FieldPercepts/FieldBoundary.h"
#include "Representations/Perception/FieldPercepts/BodyContour.h"
#include "Representations/Perception/FieldPercepts/BallSpots.h"
#include "Representations/Perception/FieldPercepts/BallSpecification.h"
#include "Representations/Perception/PlayersPerceptors/SegmentedObstacleImage.h"
#include "Representations/Perception/PlayersPerceptors/JerseyClassifier.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Modeling/OdometryData.h"
#include "Tools/Module/Module.h"

STREAMABLE(ColorConfig,
{
  ColorConfig() = default;
  (int)(80) yellowBoardCbMin;
  (int)(120) yellowBoardCbMax;
  (int)(140) yellowBoardCrMin;
  (int)(180) yellowBoardCrMax;
  // 可继续添加 blueBoard/redBoard 等
});

MODULE(PlayersColorCluster,
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FieldBoundary),
  REQUIRES(BodyContour),
  REQUIRES(ECImage),
  REQUIRES(BallSpecification),
  REQUIRES(MotionInfo),
  REQUIRES(OdometryData),
  PROVIDES(ObstaclesFieldPercept),
  PROVIDES(ObstaclesImagePercept),
  PROVIDES(ObstaclesPerceptorData),
  LOADS_PARAMETERS_FROM(ColorConfig, ColorConfig.cfg)
);

class PlayersColorCluster : public PlayersColorClusterBase
{
public:
  PlayersColorCluster();
  void update(ObstaclesFieldPercept& theObstaclesFieldPercept) override;
  void update(ObstaclesImagePercept& theObstaclesImagePercept) override;
  void update(ObstaclesPerceptorData& theObstaclesPerceptorData) override;

private:
  ColorConfig colorConfig; // 自动加载的参数
  // 这里可以添加成员变量和辅助函数声明
}; 