#!/usr/bin/env python3
#coding: utf-8

"""
全流程跑点工具 (只跑点, 不跑识别/播报)

跑的顺序和正式比赛一致, 只是省掉 VLM 识别和语音播报, 任务点也不挑, 9个全跑:
    图像点1 → 图像点2 → 图像点3 → 图像点4
    → 任务点 31,32,33,40,41,42,49,50,51 (全部9个, 按编号)
    → 9号终点

⚠️ 默认带【吸附】(和正式脚本一致): 发目标前先问规划器"这个点到得了吗", 到不了就
   换个落点; 导航失败后也换落点重试一次。
   但【不发虚拟墙】—— 跑点时封路会影响后面几腿的判断, 所以这里关掉。
   想看最原始的通行能力(不换落点、不重试), 加 --plain。
   点位是从 navigation_multi_goals.py 里读的, 所以【调点位只改那一处】。

用法:
  rosrun robot_slam point_check.py            # ★ 全流程跑一遍(图像4点 → 9个任务点 → 终点)
  rosrun robot_slam point_check.py list       # 只体检(不导航): 列出所有点位 + 净空
  rosrun robot_slam point_check.py 31         # 只跑任务点31
  rosrun robot_slam point_check.py img2       # 只跑图像点2
  rosrun robot_slam point_check.py finish     # 只跑9号终点
  rosrun robot_slam point_check.py seq 31,32,50,41   # 按指定顺序跑任务点

  选项(加在任意命令后面):
    --plain      纯单次导航: 不换落点、不重试(最原始的通行能力, 最能暴露点位本身的问题)
    --no-finish  全流程时不跑终点

⚠️ 第一次用之前要给执行权限, 否则 rosrun 报 "Cannot locate node":
     chmod +x $(rospack find robot_slam)/scripts/point_check.py

"净空"是什么: 该点位离最近【真障碍(墙/挡板)】的距离。
     >= 0.17m (= 车半宽0.14 + TEB的min_obstacle_dist0.03) 才算"车停得下" ✓
     低于它, 这个点位就该往空旷处挪 —— 这就是调点位的直接依据。
"""

import sys
import time

import rospy

import navigation_multi_goals as nm  # 复用同一份点位定义和导航逻辑

MIN_CLEAR = nm.SNAP_MIN_CLEAR  # 合格净空, 和正式脚本一致
NAV_TIMEOUT = nm.NAV_TIMEOUT  # 单次导航超时(秒), 和正式脚本一致
SETTLE_SEC = 2.0  # 每到一个点后停几秒(让车彻底站稳, 再量落点误差、再走下一腿)


def wait_costmap(ctrl, timeout=10.0):  # 等代价地图到手(净空体检要用)
    t0 = time.time()
    while ctrl.cmap is None and not rospy.is_shutdown():
        if time.time() - t0 > timeout:
            return False
        rospy.sleep(0.1)
    return ctrl.cmap is not None


def robot_xy(ctrl):  # 取当前 map 系位置
    try:
        trans, _ = ctrl.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
        return trans[0], trans[1]
    except Exception:
        return None


def check_one(ctrl, name, point):  # 只体检(不导航)
    cl = ctrl._clearance(point[0], point[1], cap=0.60)  # 净空(到真障碍); 上限放到0.6才有区分度
    rospy.loginfo(
        "[体检] %-9s (%.3f, %.3f, %6.1f°)  净空 %.3fm  %s",
        name,
        point[0],
        point[1],
        point[2],
        cl,
        "✓ 车停得下" if cl >= MIN_CLEAR else "✗ 净空不足, 建议挪这个点"
    )
    return cl


def run_leg(ctrl, name, point, kind, fallback):  # 跑一腿
    """kind: img=图像点 / task=任务点 / finish=终点
       fallback=False(默认): 纯单次导航, 不吸附、不封路、不重试
       fallback=True: 用正式脚本那套(失败换落点/封路)"""
    t0 = time.time()
    if not fallback:  # ★ 默认: 纯导航, 不加任何吸附/封路
        if kind == 'finish':  # 终点用"不设超时"(和正式一致), 但失败就失败, 不换落点
            ok = ctrl.navi.goto_no_timeout(point)
        else:
            ok = ctrl.navi.goto_with_timeout(point, timeout=NAV_TIMEOUT)
        ctrl.stop_robot()
    elif kind == 'task':  # --fallback: 任务点用正式逻辑(失败→只换落点重试, 不封路)
        ok = ctrl.goto_task_point(int(name[4:]))
    elif kind == 'img':  # 图像点用正式逻辑(失败→换落点重试), 但【不发虚拟墙】use_seal=False
        ok = ctrl.goto_with_fallback(point, name, use_seal=False)
    else:  # --fallback: 终点, 失败→只换落点再试一次(不封路)
        ok = ctrl.navi.goto_no_timeout(point)
        if not ok:
            p2 = ctrl.snap_goal(point)
            if p2 != point:
                rospy.logwarn("9号终点没走到, 换个落点再试一次")
                ok = ctrl.navi.goto_no_timeout(p2)
        ctrl.stop_robot()

    dt = time.time() - t0  # 用时(只算导航这一段, 不含下面的停顿)
    rospy.sleep(SETTLE_SEC)  # ★ 到点后停 2 秒, 让车彻底站稳再量落点、再走下一腿
    cl = ctrl._clearance(point[0], point[1], cap=0.60)  # 净空(上限放到0.6才有区分度)
    got = robot_xy(ctrl)  # 实际停在哪
    err = None
    if got is not None:
        err = ((got[0] - point[0]) ** 2 + (got[1] - point[1]) ** 2) ** 0.5
    rospy.loginfo(
        "[结果] %-9s %s  用时 %5.1fs  净空 %.3fm%s%s",
        name,
        "到达 ✓" if ok else "失败 ✗",
        dt,
        cl,
        "" if err is None else "  落点误差 %.2fm" % err,
        "" if cl >= MIN_CLEAR else "   ← 净空不足, 建议挪这个点"
    )
    return ok, dt, cl


def main():
    rospy.init_node('point_check', anonymous=True)

    # 过滤掉 --选项, 以及 ROS 自己加的改名参数(__name:=xxx / __log:=xxx), 否则会被当成点位名
    args = [a for a in sys.argv[1:] if not a.startswith('--') and ':=' not in a]
    fallback = '--plain' not in sys.argv  # 默认带吸附(和正式一致); 加 --plain 才回到纯单次导航
    no_finish = '--no-finish' in sys.argv  # 不跑终点

    navi = nm.navigation_demo()  # 导航对象(会等 move_base action server)
    ctrl = nm.CompetitionController(navi)  # 只为读点位/复用导航逻辑, 不会跑比赛流程

    images = [('img%d' % (i + 1), p, 'img') for i, p in enumerate(ctrl.image_points)]
    tasks = [('task%d' % k, p, 'task') for k, p in sorted(ctrl.task_points.items())]
    finish = [('finish', ctrl.finish_point, 'finish')]
    allpts = images + tasks + finish
    byname = dict((n, (p, k)) for n, p, k in allpts)

    if not wait_costmap(ctrl):
        rospy.logwarn("[体检] 没等到全局代价地图, 净空一列会全是 0(检查 costmap 话题)")

    # ---------- 决定要跑哪些 ----------
    if not args:  # 什么都不给 = 全流程
        todo = list(allpts)
        if no_finish:
            todo = [t for t in todo if t[2] != 'finish']
    elif args[0] == 'list':  # 只体检
        todo = []
    elif args[0] == 'seq':  # 指定任务点序列
        todo = []
        for i in (args[1].split(',') if len(args) > 1 else []):
            k = 'task' + i.strip()
            if k in byname:
                todo.append((k, byname[k][0], 'task'))
            else:
                rospy.logwarn("[体检] 没有这个任务点: %s", i)
    else:  # 一个或多个点位名
        todo = []
        for a in args:
            k = a if a.startswith(('img', 'task', 'finish')) else 'task' + a
            if k in byname:
                todo.append((k, byname[k][0], byname[k][1]))
            else:
                rospy.logwarn("[体检] 没有这个点位: %s", a)

    navigate = len(todo) > 0
    # 全流程模式: 和正式脚本一样, 先等语音唤醒再开始
    if navigate and (not args or args[0] == 'all'):
        rospy.loginfo("等待语音唤醒，请说：你好小创")
        if not ctrl.wait_for_wakeup():
            rospy.logwarn("没等到唤醒, 退出")
            return
        rospy.loginfo("已唤醒, 开始跑点")
    rospy.loginfo("=" * 64)
    if navigate:
        rospy.loginfo(
            "准备跑 %d 腿%s%s",
            len(todo),
            "(正式逻辑: 失败换落点/封路)" if fallback else "(纯导航: 不吸附、不封路、不重试)",
            "  不跑终点" if no_finish else ""
        )
    else:
        rospy.loginfo("只做体检, 不导航")
    rospy.loginfo("=" * 64)

    # ---------- 跑 ----------
    rows = []
    for idx, (name, p, kind) in enumerate(todo if navigate else allpts, start=1):
        if navigate:
            rospy.loginfo("--- [%d/%d] %s ---", idx, len(todo), name)
            ok, dt, cl = run_leg(ctrl, name, p, kind, fallback)
            rows.append((cl, name, p, ok, dt))
        else:
            rows.append((check_one(ctrl, name, p), name, p, None, None))

    # ---------- 汇总(按净空从小到大, 最该调的排最前) ----------
    rospy.loginfo("=" * 64)
    rospy.loginfo("汇总(按净空从小到大, 最该调的点位排最前):")
    n_ok = 0
    for cl, name, p, ok, dt in sorted(rows, key=lambda r: r[0]):
        if ok:
            n_ok += 1
        rospy.loginfo(
            "  %-9s (%.3f, %.3f)  净空 %.3fm %s%s",
            name,
            p[0],
            p[1],
            cl,
            "✓" if cl >= MIN_CLEAR else "✗ 净空不足",
            "" if ok is None else "  导航%s %5.1fs" % ("成功" if ok else "失败", dt)
        )
    if navigate:
        rospy.loginfo("合计: %d/%d 腿走到", n_ok, len(todo))
    rospy.loginfo("=" * 64)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
