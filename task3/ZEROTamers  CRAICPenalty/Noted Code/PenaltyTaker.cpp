#include "SkillBehaviorControl.h"

option((SkillBehaviorControl)PenaltyTaker,
       vars((std::array<unsigned, 3>)({})obstacleCellTimestamps)) /**< 每个格子中障碍物出现的时间戳。 */
{
  const float obstacleCenterThreshold = 180.f; // 距离球门中心小于该值的障碍物被认为在球门中心。
  const int obstacleCenterPriority = 500;      // 如果在该时长内检测到球门中心有障碍物，则忽略其他障碍物。
  const int obstacleMaxAge = 1000;             // 超过该时长的障碍物将被忽略。

  // 只要球门中心有障碍物，角落的感知将被忽略。
  const bool hasCenterObstacle = theFrameInfo.getTimeSince(obstacleCellTimestamps[1]) < obstacleCenterPriority;
  for (auto &percept : theObstaclesFieldPercept.obstacles)
  {
    // 只有在合理位置的感知才会被接受。
    const Vector2f perceptOnField = theRobotPose * percept.center;
    if (perceptOnField.x() < theFieldDimensions.xPosOpponentGoalArea - 300.f || perceptOnField.x() > theFieldDimensions.xPosOpponentGoalLine + 100.f || std::abs(perceptOnField.y()) > (hasCenterObstacle ? obstacleCenterThreshold : theFieldDimensions.yPosLeftGoal - 250.f))
      continue;
    // 由于球门线的存在，感知有时会非常宽，覆盖整个球门，这种情况下障碍物会被忽略。
    if ((percept.left - percept.right).squaredNorm() > sqr(1000.f))
      continue;

    if (std::abs(perceptOnField.y()) <= obstacleCenterThreshold)
      obstacleCellTimestamps[1] = theFrameInfo.time;
    else if (perceptOnField.y() > 0)
      obstacleCellTimestamps[0] = theFrameInfo.time;
    else
      obstacleCellTimestamps[2] = theFrameInfo.time;
  }

  auto isPenaltyShootoutObstacleInGoal = [&](const bool left)
  {
    return theFrameInfo.getTimeSince(obstacleCellTimestamps[left ? 0 : 2]) < obstacleMaxAge;
  }; // 判断障碍物是否在球门内

  // auto kickLeft = [&]
  // {
  //   if(theBehaviorParameters.penaltyStrikerUseObstacles)
  //   {
  //     if(isPenaltyShootoutObstacleInGoal(true))
  //       return false;
  //     else if(isPenaltyShootoutObstacleInGoal(false))
  //       return true;
  //   }
  //   return ((theFrameInfo.time >> 4) & 1) != 0;
  // };

  auto kickLeft = [&]
  {
    static int kickCount = 0;
    static bool flip = true;

    if (kickCount % 2 == 0 && kickCount != 0) // 反转bool变量
    {
      flip = !flip;
      kickCount = 0;
    }

    ++kickCount;

    return flip;
  }; // 左右脚交替踢球，虽然说叫kickLeft但其实是决定左踢还是右踢的bool变量。

  if (theGameState.isPushingFreeKick())
    Say({.text = "Taking Penalty"}); // 开机当比赛进入推球任意球阶段时，机器人主动播报‘Taking Penalty’。

  const int timeSincePenaltyShootoutStarted = theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted);
  const int timeUntilPenaltyShootoutEnds = -theFrameInfo.getTimeSince(theGameState.timeWhenStateEnds);

  common_transition
  {
    if (timeSincePenaltyShootoutStarted > 5000 && !theFieldBall.ballWasSeen(5000))
      goto goBehindPenaltyMark;
  } // 当比赛进行到5秒后，且球未被看到时，切换到goBehindPenaltyMark状态。（DSL特殊块）

  initial_state(initial) // 定义了行为树的初始状态，命名为 initial。
                         // 当 PenaltyTaker 这个 Option 第一次被激活时，机器人就会进入这个状态
  {
    transition // 定义了状态跳转条件
    {
      if (std::min(timeSincePenaltyShootoutStarted, state_time) > 3000 || timeUntilPenaltyShootoutEnds <= 10000) // 取比赛开始时间和状态持续时间的最小值 超过 3 秒，认为可以准备去踢球了
      {
        if (kickLeft())
          goto goToBallAndKickLeft;
        else
          goto goToBallAndKickRight;
      }
    }
    action // 定义了该状态下机器人持续执行的动作
    {
      LookLeftAndRight({.maxPan = 20_deg,
                        .tilt = 5.7_deg,
                        .speed = 30_deg});
      // 头部左右轻微扫动（最大左右 20°，俯仰 5.7°，速度 30°/s）
      Stand();
    }
  }

  // 整体逻辑总结
  // 机器人刚进入点球模式 → 先原地站立扫头

  //  等待 3 秒（或点球剩余时间不足 10 秒）

  //  根据 kickLeft() 判断左右脚

  //  跳转到对应踢球状态执行射门

  state(goToBallAndKickLeft)
  {
    action
    {
      const Vector2f goalPostOnField(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal);
      const Vector2f ballPositionOnField = theRobotPose * theBallModel.estimate.position;
      const Angle angle = (goalPostOnField - ballPositionOnField).angle() -
                          theBehaviorParameters.penaltyStrikerAngleToLeftPostOffset;

      // 右脚的射门方向会偏向右侧——更靠近球门中部。左脚偏向左侧——更靠近球门柱。我们不希望踢到球门柱！“！！”(小nao无法真正仿人用足弓踢球，而是用足尖踢球，右脚足尖的右侧碰触球，左脚足尖左侧出球，从而扩大射门角度)
      // KickInfo::KickType kickType = KickInfo::forwardFastRightLong;
      KickInfo::KickType kickType = KickInfo::forwardFastLeftLong; // 禁止用右脚射门，只能用左脚
      /*
      if(theDamageConfigurationBody.sides[Legs::left].weakLeg && !theDamageConfigurationBody.sides[Legs::right].weakLeg)
        kickType = KickInfo::forwardFastLeftLong;
      */

      if (theGameState.isPenaltyKick())
        GoToBallAndKick({.targetDirection = Angle::normalize(angle - theRobotPose.rotation),
                         .kickType = kickType,
                         .alignPrecisely = KickPrecision::precise,
                         .speed = {theBehaviorParameters.penaltyStrikerWalkSpeed, theBehaviorParameters.penaltyStrikerWalkSpeed, theBehaviorParameters.penaltyStrikerWalkSpeed},
                         .reduceWalkSpeedType = ReduceWalkSpeedType::slow}); // 比赛模式用球
      // gc操控
      else
      {
        Pose2f kickPoseOnField(angle, ballPositionOnField);
        kickPoseOnField.rotate(theKickInfo.kicks[kickType].rotationOffset);
        kickPoseOnField.translate(theKickInfo.kicks[kickType].ballOffset);
        PenaltyStrikerGoToBallAndKick({.kickPose = theRobotPose.inverse() * kickPoseOnField,
                                       .kickType = kickType,
                                       .walkSpeed = theBehaviorParameters.penaltyStrikerWalkSpeed}); // 调试模式用球,按脚操控。
      }
    }
  }

  state(goToBallAndKickRight)
  {
    action
    {
      const Vector2f goalPostOnField(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal);
      const Vector2f ballPositionOnField = theRobotPose * theBallModel.estimate.position;
      const Angle angle = (goalPostOnField - ballPositionOnField).angle() +
                          theBehaviorParameters.penaltyStrikerAngleToRightPostOffset;

      // 左脚的射门方向会偏向左侧——更靠近球门中部。右脚偏向右侧——更靠近球门柱。我们不希望踢到球门柱！（原理，代码实现同上）
      KickInfo::KickType kickType = KickInfo::forwardFastLeftLong;
      if (theDamageConfigurationBody.sides[Legs::right].weakLeg && !theDamageConfigurationBody.sides[Legs::left].weakLeg)
        kickType = KickInfo::forwardFastRightLong;

      if (theGameState.isPenaltyKick())
        GoToBallAndKick({.targetDirection = Angle::normalize(angle - theRobotPose.rotation),
                         .kickType = kickType,
                         .alignPrecisely = KickPrecision::precise,
                         .speed = {theBehaviorParameters.penaltyStrikerWalkSpeed, theBehaviorParameters.penaltyStrikerWalkSpeed, theBehaviorParameters.penaltyStrikerWalkSpeed},
                         .reduceWalkSpeedType = ReduceWalkSpeedType::slow});
      else
      {
        Pose2f kickPoseOnField(angle, ballPositionOnField);
        kickPoseOnField.rotate(theKickInfo.kicks[kickType].rotationOffset);
        kickPoseOnField.translate(theKickInfo.kicks[kickType].ballOffset);
        PenaltyStrikerGoToBallAndKick({.kickPose = theRobotPose.inverse() * kickPoseOnField,
                                       .kickType = kickType,
                                       .walkSpeed = theBehaviorParameters.penaltyStrikerWalkSpeed});
      }
    }
  }

  state(goBehindPenaltyMark) // 没找到球，回归点球点
  {
    transition
    {
      if (theFieldBall.ballWasSeen())
      {
        if (theBallModel.estimate.position.squaredNorm() < sqr(500.f)) // 与球距离小于一定阈值（0.5m）进入踢球状态。
        {
          if (kickLeft())
            goto goToBallAndKickLeft;
          else
            goto goToBallAndKickRight;
        }
        else
          goto initial; // 机器人与球大于0.5m，回归初始状态
      }
    }
    action
    {
      LookActive({.withBall = true,
                  .onlyOwnBall = true});
      const Vector2f target = theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOpponentPenaltyMark - 300.f, 0.f); // theRobotPose.inverse()=全场坐标系 → 转换到机器人坐标系的变换矩阵。
      WalkToPoint({.target = {-theRobotPose.rotation, target},
                   .speed = {theBehaviorParameters.penaltyStrikerWalkSpeed, theBehaviorParameters.penaltyStrikerWalkSpeed, theBehaviorParameters.penaltyStrikerWalkSpeed},
                   .rough = true,
                   .disableObstacleAvoidance = true});
    }
  }
}
