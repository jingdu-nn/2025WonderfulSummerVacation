/**
 * @file Representations/Configuration/BehaviorParameters/BehaviorParameters.h
 *
 * 本文件声明了一个结构体，包含可修改的行为常用参数。
 *
 * @author Andreas Stolpmann
 */

#pragma once

#include "Math/Angle.h"

STREAMABLE(BehaviorParameters,
{,
  (bool) keeperJumpingOn, /**< 使用此配置参数可以防止守门员跳跃（以免受伤）- 默认值为 true */

  (float) penaltyStrikerWalkSpeed, /**< 定义点球射手走向球的速度百分比（0-100） */
  (Angle) penaltyStrikerAngleToLeftPostOffset,  /**< （值越大，射门越靠近球门中路） */
  (Angle) penaltyStrikerAngleToRightPostOffset,  /**< （值越大，射门越靠近球门中路） */
  (bool) penaltyStrikerUseObstacles, /**< 点球射手是否根据障碍物模型选择更空的角落射门 */

  (float) ballCatchMaxWalkDistance, /**< 扑球最大步行距离 */

  (float) standRadius, /**< 仅站立时能覆盖的范围。 */
  (Rangef) walkRadius, /**< 行走时能覆盖的范围。 */
  (Rangef) timeForIntercetionForMaxWalkRadius, /**< 球到达拦截点所需的时间，允许最大步行半径。 */
  (float) genuflectRadius, /**< 守门员下跪扑救时能覆盖的范围。 */
  (float) genuflectStandRadius, /**< 下跪时能覆盖的范围。 */
  (float) jumpRadius, /**< 守门员跳跃时能覆盖的范围。 */
  (float) timeForJump, /**< 跳跃后机器人开始控球所需的时间。 */
});
