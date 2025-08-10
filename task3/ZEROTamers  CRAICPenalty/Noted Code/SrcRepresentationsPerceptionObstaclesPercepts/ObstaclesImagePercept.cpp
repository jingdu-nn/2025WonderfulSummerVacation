/**
 * @file ObstaclesImagePercept.cpp
 *
 * This file implements a representation that lists the obstacles that were detected in
 * the current image.
 *
 * @author Michel Bartsch
 * @author Andre Mühlenbrock
 * @author Thomas Röfer
 */

#include "ObstaclesImagePercept.h"
#include "Debugging/DebugDrawings.h"

void ObstaclesImagePercept::draw() const
{
  DEBUG_DRAWING("representation:ObstaclesImagePercept:image", "drawingOnImage")
  {
    for(const auto& obstacle : obstacles)
      RECTANGLE("representation:ObstaclesImagePercept:image", obstacle.left, obstacle.top, obstacle.right, obstacle.bottom, 4, Drawings::solidPen, ColorRGBA::white);
  }
}



//DEBUG_DRAWING
这一宏用于定义一个调试绘图区域。这表示在图像上创建一个名为 "representation:ObstaclesImagePercept:image" 的绘图区域，类型为 "drawingOnImage"。



// 矩形描边：
//RECTANGLE(id绘图 ID（字符串，需提前通过 DECLARE_DEBUG_DRAWING 注册）, x1, y1, x2, y2, penWidth线宽, penStyle线条样式, penColor颜色)
//FILLED_RECTANGLE 画实心框