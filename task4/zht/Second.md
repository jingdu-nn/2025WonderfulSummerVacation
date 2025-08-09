### 模块架构
```
src/
├── coach/          # 教练模块
├── player/         # 球员模块
│   ├── basic_actions/  # 基础动作
│   ├── planner/        # 策略规划
│   └── setplay/        # 定位球策略
└── trainer/        # 训练师模块
```

首先我们来看player中的basic_actions模块
```
├── arm_off.h                       // 放下手臂（execute函数在basic_actions.cpp中实现，返回bool类型，判断能否放下手臂）
├── arm_point_to_point.h            // 手指向特定点（execute函数在basic_actions.cpp中实现，返回bool类型，判断能否指向特定点）
├── basic_actions.cpp               // 
├── basic_actions.h
├── bhv_before_kick_off.cpp         // 初始定位和朝向；位置无效重新扫描；进球后处理；开球越位规则；确定开球方；移动到指定位置；扫描场地
├── bhv_before_kick_off.h
├── bhv_body_neck_to_ball.h         // 计算球的下一个位置，身体和脖子朝向球，优先身体转动
├── bhv_body_neck_to_point.h        // 身体和脖子朝向特定方向，分为一次转向和两次转向，优先身体转动
├── bhv_emergency.cpp               // 紧急情况，重新扫描场地
├── bhv_emergency.h
├── bhv_go_to_point_look_ball.cpp   // 移动到指定位置并且看向球
├── bhv_go_to_point_look_ball.h
├── bhv_neck_body_to_ball.h         // 优先脖子转动
├── bhv_neck_body_to_point.h        // 优先脖子转动
├── bhv_scan_field.cpp              // 寻找球和场地扫描
├── bhv_scan_field.h
├── body_advance_ball2009.cpp       // 核心：带球前进动作，重点优化对象，长远球
├── body_advance_ball2009.h
├── body_advance_ball.h
├── body_clear_ball2009.cpp         // 核心：带球Clear动作，重点优化对象
├── body_clear_ball2009.h
├── body_clear_ball.h
├── body_dribble2008.cpp            // 核心：连续带球，重点优化对象，精细控球
├── body_dribble2008.h
├── body_dribble.h
├── body_go_to_point.cpp
├── body_go_to_point_dodge.cpp      // 绕开障碍物的路径计算
├── body_go_to_point_dodge.h
├── body_go_to_point.h              // 移动到指定位置，无障碍物
├── body_hold_ball2008.cpp          // 核心：控球保持，重点优化对象
├── body_hold_ball2008.h
├── body_hold_ball.h
├── body_intercept2018.cpp          // 核心：拦截球，重点优化对象
├── body_intercept2018.h
├── body_intercept.h
├── body_kick_one_step.cpp          // 快速踢球
├── body_kick_one_step.h
├── body_kick_to_relative.cpp       // 相对位置控球
├── body_kick_to_relative.h
├── body_pass.cpp                   // 传球传球
├── body_pass.h
├── body_smart_kick.cpp             // 多步踢球控制
├── body_smart_kick.h
├── body_stop_ball.cpp              // 停球
├── body_stop_ball.h
├── body_stop_dash.cpp
├── body_stop_dash.h
├── body_tackle_to_point.h
├── body_turn_to_angle.h
├── body_turn_to_ball.h
├── body_turn_to_point.h
├── focus_move_to_point.cpp
├── focus_move_to_point.h
├── focus_reset.cpp
├── focus_reset.h
├── intention_dribble2008.cpp
├── intention_dribble2008.h
├── intention_time_limit_action.cpp
├── intention_time_limit_action.h
├── intercept_evaluator.cpp
├── intercept_evaluator_factory.cpp
├── intercept_evaluator_factory.h
├── intercept_evaluator.h
├── kick_table.cpp
├── kick_table.h
├── Makefile
├── neck_scan_field.cpp
├── neck_scan_field.h
├── neck_scan_players.cpp
├── neck_scan_players.h
├── neck_turn_to_ball_and_player.cpp
├── neck_turn_to_ball_and_player.h
├── neck_turn_to_ball.h
├── neck_turn_to_ball_or_scan.cpp
├── neck_turn_to_ball_or_scan.h
├── neck_turn_to_goalie_or_scan.cpp
├── neck_turn_to_goalie_or_scan.h
├── neck_turn_to_low_conf_teammate.cpp
├── neck_turn_to_low_conf_teammate.h
├── neck_turn_to_player_or_scan.cpp
├── neck_turn_to_player_or_scan.h
├── neck_turn_to_point.h
├── neck_turn_to_relative.h
├── obsolete
│   ├── body_intercept2009.cpp
│   └── body_intercept2009.h
├── view_change_width.h
├── view_normal.h
├── view_synch.cpp
├── view_synch.h
└── view_wide.h
```