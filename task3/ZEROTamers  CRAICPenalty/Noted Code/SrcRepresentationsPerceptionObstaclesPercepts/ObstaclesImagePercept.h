/**
 * @file ObstaclesImagePercept.h
 *
 * 本文件声明了一个表示，用于列出在当前图像中检测到的障碍物。
 *
 * @author Michel Bartsch
 * @author Andre Mühlenbrock
 * @author Thomas Röfer
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Math/Eigen.h"

STREAMABLE(ObstaclesImagePercept,
{
  STREAMABLE(Obstacle,
  {,
    (int) top, /**< 障碍物在图像中的顶部边界（估算值）。*/
    (int) bottom, /**< 障碍物在图像中的底部边界。*/
    (int) left, /**< 障碍物在图像中的左边界，通常只包含底端的宽度。*/
    (int) right, /**< 障碍物在图像中的右边界，通常只包含底端的宽度。*/
    (bool) bottomFound, /**< 是否找到了障碍物的底端？否则被图像下边界遮挡。*/
    (bool) fallen, /**< 该障碍物是否为倒地球员？*/
    (float)(1.f) confidence, /**< 物体预测的置信度 */
    (float)(-1.f) distance,
  });

  /** Draws this percept. */
  /** 绘制该感知结果。*/
  void draw() const,   //是函数的声明，表示 ObstaclesImagePercept 这个类/结构体有一个名为 draw 的常成员函数。（const：// 不允许修改成员变量）

  (std::vector<Obstacle>) obstacles, /**< 当前图像中检测到的所有障碍物。*/
});

//STREAMABLE(ObstaclesImagePercept, {...}) 
//STREAMABLE(Obstacle,{。。。})

