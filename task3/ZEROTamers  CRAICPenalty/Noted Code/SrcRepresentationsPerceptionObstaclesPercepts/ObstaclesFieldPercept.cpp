/**
 * @file ObstaclesFieldPercept.cpp
 *
 * 本文件实现了一个表示，在当前图像中检测到的障碍物列表，
 * 这些障碍物的坐标为机器人相对场地坐标。只有下端可见的障碍物会被表示。
 *
 * @author Andre Mühlenbrock
 * @author Tim Laue
 */

#include "ObstaclesFieldPercept.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Debugging/DebugDrawings.h"
#include "Tools/Math/Transformation.h"
#include "Framework/Blackboard.h"
#include <cmath>

void ObstaclesFieldPercept::draw() const
{
  static const ColorRGBA penColors[] =
  {
    ColorRGBA::gray,
    ColorRGBA::white,
    ColorRGBA::white
  };//用于不同障碍物的边框颜色

  // 定义绘图的标准颜色，尝试获取原始队伍颜色信息：
  const ColorRGBA teamColors[] =
  {
    ColorRGBA::white,
    ColorRGBA::fromTeamColor(static_cast<int>(Blackboard::getInstance().exists("GameState") ?
        static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).ownTeam.fieldPlayerColor : GameState::Team::Color::black)),
    ColorRGBA::fromTeamColor(static_cast<int>(Blackboard::getInstance().exists("GameState") ?
        static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).opponentTeam.fieldPlayerColor : GameState::Team::Color::red)),
    ColorRGBA::fromTeamColor(static_cast<int>(Blackboard::getInstance().exists("GameState") ?
        static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).ownTeam.goalkeeperColor : GameState::Team::Color::purple)),
    ColorRGBA::fromTeamColor(static_cast<int>(Blackboard::getInstance().exists("GameState") ?
        static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).opponentTeam.goalkeeperColor : GameState::Team::Color::blue))
  };

  // 在场地视图中绘制机器人：
  DEBUG_DRAWING("representation:ObstaclesFieldPercept:field", "drawingOnField")
  {
    for(const Obstacle& obstacle : obstacles) //遍历当前帧检测到的所有障碍物。

    {
      const float radius = std::max((obstacle.left - obstacle.right).norm() / 2.f, 50.f);//左右差值取模的一半，最小值为50mm
      CIRCLE("representation:ObstaclesFieldPercept:field", obstacle.center.x(), obstacle.center.y(), radius, 10, Drawings::solidPen,
             penColors[obstacle.type], Drawings::solidBrush, teamColors[obstacle.type]);        //绘制障碍物中心点，并根据类型设置颜色。
    }
  }//

  // 在图像中绘制（这里真的有用吗？）
  DEBUG_DRAWING("representation:ObstaclesFieldPercept:image", "drawingOnImage")
  {
    if(Blackboard::getInstance().exists("CameraInfo") && Blackboard::getInstance().exists("CameraMatrix")
       && Blackboard::getInstance().exists("ImageCoordinateSystem"))//检查是否存在必要的相机参数和坐标系统。
    {
      const CameraInfo& cameraInfo = static_cast<CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);//获取相机信息
      const CameraMatrix& cameraMatrix = static_cast<CameraMatrix&>(Blackboard::getInstance()["CameraMatrix"]);//获取相机矩阵
      const ImageCoordinateSystem& imageCoordinateSystem = static_cast<ImageCoordinateSystem&>(Blackboard::getInstance()["ImageCoordinateSystem"]);//获取图像坐标系统

      for(const Obstacle& obstacle : obstacles)//遍历当前帧检测到的所有障碍物。
      {
        Vector2f pointsInImage[3];//用于存储障碍物在图像中的三个关键点。
        if(Transformation::robotToImage(obstacle.left, cameraMatrix, cameraInfo, pointsInImage[0])//
           && Transformation::robotToImage(obstacle.right, cameraMatrix, cameraInfo, pointsInImage[1])
           && Transformation::robotToImage(Vector3f(obstacle.left.x(), obstacle.left.y(), 580.f), cameraMatrix, cameraInfo, pointsInImage[2]))//把障碍物的左边界、右边界和顶部（高度580mm）从机器人相对场地坐标系变换到图像像素坐标系。
        {
          for(int i = 0; i < 3; ++i)//将三个点从相机坐标系转换为图像坐标系。
            pointsInImage[i] = imageCoordinateSystem.fromCorrected(pointsInImage[i]);
          if(obstacle.type == unknown)
          {
            RECTANGLE("representation:ObstaclesFieldPercept:image", pointsInImage[0].x() - 2.5f, pointsInImage[0].y() + 2.5f, pointsInImage[1].x() + 2.5f, pointsInImage[2].y() - 2.5f, 1, Drawings::solidPen, ColorRGBA::red);
            RECTANGLE("representation:ObstaclesFieldPercept:image", pointsInImage[0].x() - 1.5f, pointsInImage[0].y() + 1.5f, pointsInImage[1].x() + 1.5f, pointsInImage[2].y() - 1.5f, 1, Drawings::solidPen, ColorRGBA::orange);
            RECTANGLE("representation:ObstaclesFieldPercept:image", pointsInImage[0].x() - 0.5f, pointsInImage[0].y() + 0.5f, pointsInImage[1].x() + 0.5f, pointsInImage[2].y() - 0.5f, 1, Drawings::solidPen, ColorRGBA::yellow);
            RECTANGLE("representation:ObstaclesFieldPercept:image", pointsInImage[0].x() + 0.5f, pointsInImage[0].y() - 0.5f, pointsInImage[1].x() - 0.5f, pointsInImage[2].y() + 0.5f, 1, Drawings::solidPen, ColorRGBA::green);
            RECTANGLE("representation:ObstaclesFieldPercept:image", pointsInImage[0].x() + 1.5f, pointsInImage[0].y() - 1.5f, pointsInImage[1].x() - 1.5f, pointsInImage[2].y() + 1.5f, 1, Drawings::solidPen, ColorRGBA::blue);
            RECTANGLE("representation:ObstaclesFieldPercept:image", pointsInImage[0].x() + 2.5f, pointsInImage[0].y() - 2.5f, pointsInImage[1].x() - 2.5f, pointsInImage[2].y() + 2.5f, 1, Drawings::solidPen, ColorRGBA::violet);
          }
          else
          {
            RECTANGLE("representation:ObstaclesFieldPercept:image",
                      pointsInImage[0].x(), pointsInImage[0].y(), pointsInImage[1].x(), pointsInImage[2].y(),
                      6, Drawings::solidPen, teamColors[obstacle.type]);
          }
          //DRAW_TEXT("representation:ObstaclesFieldPercept:image", pointsInImage[0].x() + 2, pointsInImage[0].y() + 14, 10, teamColors[obstacle.type], "Team Conf: " << std::to_string(obstacle.confidence).substr(0, 4));
        }
      }
    }
  }
}

void ObstaclesFieldPercept::verify() const
{
  for([[maybe_unused]] const Obstacle& obstacle : obstacles)
  {
    ASSERT(std::isfinite(obstacle.center.x()));
    ASSERT(std::isfinite(obstacle.center.y()));
    ASSERT(std::isfinite(obstacle.left.x()));
    ASSERT(std::isfinite(obstacle.left.y()));
    ASSERT(std::isfinite(obstacle.right.x()));
    ASSERT(std::isfinite(obstacle.right.y()));
  }
}//该函数用于数据有效性检查，确保所有障碍物的关键坐标值都是有限数值（不是NaN或无穷大），防止后续处理出错。



//CIRCLE("representation:ObstaclesFieldPercept:field", obstacle.center.x(), obstacle.center.y(), radius, penWidth, Drawings::solidPen,
//penColor, Drawings::solidBrush, teamColors[obstacle.type]);

//障碍物绘制（draw函数）
// 场地视图绘制
// 在场地平面上，根据障碍物的类型和位置，用不同颜色的圆圈表示障碍物的位置和大小。
// 图像视图绘制
// 将障碍物的左右边界和顶部（高度580mm）从机器人坐标系转换到图像像素坐标系，在图像上画出障碍物的矩形框。
// 对于未知类型的障碍物，会用多种颜色画出多层矩形框，使其在图像上非常醒目，便于调试和观察。
// 对于已知类型的障碍物，则用对应队伍颜色画出矩形框。
// 数据有效性检查（verify函数）
// 检查每个障碍物的关键坐标值（中心点、左右边界）是否为有限数值（不是NaN或无穷大），防止后续处理出错。