#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
一键流水线执行脚本（根据用户指定顺序串联所有模块）

顺序：
1) 执行 ./start_cameras.sh 启动相机
2) 运行 a_warehouse_mission.py（A库任务）
3) 运行 playback_runner.py（回放录制）
4) 运行 green_arrow_detector.py（识别箭头） -> 根据结果调用 arrow_nav_runner.py 执行“按箭头走->反向转->直行并黄框检测->继续直行”
5) 直行期间执行 detect_yellow_in_2png.py（已在 arrow_nav_runner 内封装）
6) 运行 eazy.py；当出现趴下阶段后，运行 say_ciallo.py 与 touch_interaction.py，完成触摸与播报后继续
7) 最后运行 yellow_run.py

注意：
- 本脚本尽量通过子进程管理各阶段，可通过输出关键字进行简单同步。
"""

import os
import sys
import time
import subprocess
import signal
import json

# 轻量级LCM站立发布器，避免各阶段结束后程序自行趴下
try:
    import lcm
    from robot_control_cmd_lcmt import robot_control_cmd_lcmt
except Exception:
    lcm = None
    robot_control_cmd_lcmt = None

class StandPublisher:
    def __init__(self):
        self.available = (lcm is not None and robot_control_cmd_lcmt is not None)
        if self.available:
            try:
                self.lc = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
                self.msg = robot_control_cmd_lcmt()
            except Exception:
                self.available = False

    def stand_burst(self, times=3, interval_s=0.2, duration_ms=1500):
        if not self.available:
            return
        try:
            for _ in range(max(1, times)):
                self.msg.mode = 12  # recovery stand
                self.msg.gait_id = 0
                self.msg.duration = duration_ms
                self.msg.life_count = (self.msg.life_count + 1) % 128
                self.lc.publish("robot_control_cmd", self.msg.encode())
                time.sleep(interval_s)
        except Exception:
            pass


ROOT = os.path.dirname(os.path.abspath(__file__))

def run_shell_script(path, wait=True):
    proc = subprocess.Popen(["bash", path], cwd=ROOT)
    if wait:
        return proc.wait()
    return 0


def run_python(script, args=None, wait=True, capture_output=False):
    cmd = [sys.executable, os.path.join(ROOT, script)]
    if args:
        cmd.extend(args)
    stdout = subprocess.PIPE if capture_output else None
    stderr = subprocess.STDOUT if capture_output else None
    proc = subprocess.Popen(cmd, cwd=ROOT, stdout=stdout, stderr=stderr, universal_newlines=True)
    return proc.wait() if wait else proc


def run_python_with_grepping(script, keywords, timeout_s=20, keep_stand=False):
    """运行脚本并在输出中等待任一关键字，直到超时或进程结束。返回 (found, last_line)。"""
    cmd = [sys.executable, os.path.join(ROOT, script)]
    proc = subprocess.Popen(cmd, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
    start = time.time()
    last_line = ""
    found = False
    sp = StandPublisher() if keep_stand else None
    last_keep_ts = 0.0
    try:
        while True:
            if proc.poll() is not None:
                break
            line = proc.stdout.readline() if proc.stdout else ""
            if line:
                last_line = line.strip()
                print(f"[{script}] {last_line}")
                if any(k in last_line for k in keywords):
                    found = True
                    break
            if time.time() - start > timeout_s:
                break
            # 保持站立心跳（检测期间）
            if sp and (time.time() - last_keep_ts) >= 0.6:
                sp.stand_burst(times=1, interval_s=0.0, duration_ms=1500)
                last_keep_ts = time.time()
    finally:
        try:
            proc.terminate()
        except Exception:
            pass
    return found, last_line


def main():
    stand_pub = StandPublisher()
  
    print("▶️ 运行 A 库任务 a_warehouse_mission.py ...")
    run_python("a_warehouse_mission.py", wait=True)
    # 阶段结束后，强制站立，防止脚本在 finally 中趴下
    stand_pub.stand_burst()

    # 3) 直接运行 S 弯（不再回放）
    print("▶️ 运行 S 弯 s_curve_runner.py ...")
    run_python("s_curve_runner.py", wait=True)
    stand_pub.stand_burst()

    # 4) 使用 Doubao 脚本进行箭头识别，依据结果选择 all_r.py 或 all_l.py
    print("▶️ 运行箭头/文字识别 doubao_seed_flash_ocr_qr.py (短时检测)")
    stand_pub.stand_burst()
    direction = "右箭头"
    try:
        proc = subprocess.run([sys.executable, os.path.join(ROOT, "doubao_seed_flash_ocr_qr.py")],
                              cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              universal_newlines=True, timeout=25)
        out = (proc.stdout or "").strip()
        if out:
            try:
                obj = json.loads(out.splitlines()[-1])
                arrow = str(obj.get("箭头识别结果", "无"))
                if "左" in arrow:
                    direction = "左箭头"
                elif "右" in arrow:
                    direction = "右箭头"
            except Exception:
                pass
    except Exception:
        pass

    target_script = "all_r.py" if direction == "右箭头" else "all_l.py"
    print(f"▶️ 按箭头执行 {target_script} （方向: {direction}）...")
    run_python(target_script, wait=True)
    stand_pub.stand_burst()

    # 5) 再次运行 S 弯（不再回放）
    print("▶️ 再次运行 S 弯 s_curve_runner_last.py ...")
    run_python("s_curve_runner_last.py", wait=True)
    stand_pub.stand_burst()

    # 6) 执行 a_hui_mission.py
    print("▶️ 运行 A 回任务 a_hui_mission.py ...")
    run_python("a_hui_mission.py", wait=True)
    stand_pub.stand_burst()

    print("🎉 全流程执行完成")


if __name__ == "__main__":
    main()

