/**
 * @file ObstaclesFieldPercept.h
 *
 * 本文件声明了一个表示，在当前图像中检测到的障碍物列表，
 * 这些障碍物的坐标为机器人相对场地坐标。只有下端可见的障碍物会被表示。
 *
 * @author Andre Mühlenbrock
 * @author Tim Laue
 * @author Thomas Röfer
 */

#pragma once

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"

STREAMABLE(ObstaclesFieldPercept,
{
  /** 检测到的障碍物类型 */
  ENUM(Type,
  {,
    unknown,            /**< 未检测到球衣。 */
    ownPlayer,          /**< 检测到己方队员的球衣。 */
    opponentPlayer,     /**< 检测到对方队员的球衣。 */
    ownGoalkeeper,      /**< 检测到己方守门员的球衣。 */
    opponentGoalkeeper, /**< 检测到对方守门员的球衣。 */
  });

  /** 单个障碍物的表示 */
  STREAMABLE(Obstacle,
  {,
    (Vector2f)(Vector2f::Zero()) center,     /**< 障碍物中心点，机器人相对坐标（单位：毫米） */
    (Vector2f)(Vector2f::Zero()) left,       /**< 障碍物左边界，机器人相对坐标（单位：毫米） */
    (Vector2f)(Vector2f::Zero()) right,      /**< 障碍物右边界，机器人相对坐标（单位：毫米） */
    (bool)(false) fallen,                    /**< 该障碍物是否为倒地球员？ */
    (Type)(Type::unknown) type,              /**< 障碍物的类型 */
    (float)(0.f) confidence,                 /**< 类型识别的置信度 */
    (Matrix2f)(Matrix2f::Zero()) covariance, /**< 中心点测量的协方差矩阵 */
  });

  /** 绘制该感知结果。 */
  void draw() const;

  /** 检查所有坐标是否为有限值。 */
  void verify() const,

  (std::vector<Obstacle>) obstacles, /**< 当前图像中检测到的、下端可见的障碍物列表。 */
});


//STREAMABLE(ObstaclesFieldPercept,
//STREAMABLE(Obstacle,