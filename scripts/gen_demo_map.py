#!/usr/bin/env python3
"""生成 demo_warehouse 测试地图 (map.pgm) —— 可复现的地图工件构建脚本。

为什么需要脚本而不是直接改二进制
--------------------------------
地图必须容纳**两条互不相交的通道**，才能在 e2e 中证明"两台车真正同时在跑"：
  * lane1（原有 L 形走廊）：station_pickup_A(1.5,2.0) -> station_dropoff_B(8.0,5.5)
  * lane2（右侧竖直通道）：station_lane2_pickup_C -> station_lane2_dropoff_D
两条通道的**任务路径**互不相交：A->B 只走"横向段西半 + 车道1 竖直段"，
C->D 只走"车道2 竖直段"，两者不共享任何路权资源（无共享路口节点、
无共享边），静态排他锁模式下也能真正并行——两台车物理上各走各的通道。

几何约定
--------
  * 分辨率 0.05 m/px，原点 [0, 0, 0]，地图 260 x 140 px = 13.0 m x 7.0 m。
  * PGM 行号自上而下（row 0 = 地图顶部），世界坐标 y 自下而上：
        world_x = (col + 0.5) * resolution
        world_y = (HEIGHT - 1 - row + 0.5) * resolution
    （nav2_map_server 会把图像上下翻转成 OccupancyGrid，故 PGM 顶行对应最大 y。）
  * 旧地图（200 x 140）的像素**逐位复制**到新图左上角 —— 列方向扩展不改变
    任何既有内容的 (x, y) 世界坐标，旧站点坐标与旧路网几何**完全不变**
    （e2e 中钉死的 1.625/2.025、8.025/5.5 断言继续有效）。
  * 254 = 自由空间，0 = 障碍。

通道参数（矩形闭区间：col_start, col_end, row_start, row_end）
---------------------------------------------------------------
  lane1 横向通道   cols  20..228  rows  87..111   （world y 1.425..2.625）
  lane1 竖直通道   cols 148..172  rows   7..111   （world x 7.425..8.575）
  lane2 竖直通道   cols 204..228  rows   7..123   （world x 10.225..11.425）

横向通道一路延伸到 lane2（两者在右上角成 L 形交汇，25x25 的拐角块与原
L 走廊同构，骨架细化后可稳定提取）。交汇点 (10.825, 2.025) **不在**
A->B 的行驶路径上（A->B 在 x≈8.0 处转北上），因此两条任务路径不共享资源。

站点（markers.yaml）
-------------------
  station_pickup_A        (1.5, 2.0)     -> lane1 横向通道西端
  station_dropoff_B       (8.0, 5.5)     -> lane1 竖直通道
  station_lane2_pickup_C  (10.825, 1.025) -> lane2 竖直通道南段
  station_lane2_dropoff_D (10.825, 4.975) -> lane2 竖直通道北段

lane2 的 C->D 里程（约 3.55 m）与 lane1 的 A->B 里程（约 9.82 m）同量级，
两台车才能在**同一时间窗**内同时行驶（本地双模拟车并发验收的时窗重叠断言）。
"""

import os

RESOLUTION = 0.05
WIDTH = 260
HEIGHT = 140
FREE = 254
OBSTACLE = 0

# ---- 通道矩形 (col_start, col_end, row_start, row_end)，闭区间 ----
CORRIDORS = [
    (20, 172, 87, 111),   # lane1 横向通道
    (148, 172, 7, 111),   # lane1 竖直通道
    (172, 228, 7, 31),    # 顶部连接段（lane1 竖腿 <-> lane2 竖腿）
    (204, 228, 7, 123),   # lane2 竖直通道
]


def legacy_map() -> list:
    """旧版 200x140 地图（仅 lane1 几何），逐像素复用到新图左上角。"""
    grid = [[OBSTACLE] * 200 for _ in range(140)]
    rects = [
        (20, 172, 87, 111),   # 横向通道
        (148, 172, 7, 111),   # 竖直通道
    ]
    for c0, c1, r0, r1 in rects:
        for row in range(r0, r1 + 1):
            for col in range(c0, c1 + 1):
                grid[row][col] = FREE
    return grid


def build() -> list:
    grid = [[OBSTACLE] * WIDTH for _ in range(HEIGHT)]
    old = legacy_map()
    for row in range(len(old)):
        for col in range(len(old[0])):
            grid[row][col] = old[row][col]
    for c0, c1, r0, r1 in CORRIDORS:
        for row in range(r0, r1 + 1):
            for col in range(c0, c1 + 1):
                grid[row][col] = FREE
    return grid


def write_pgm(grid: list, path: str) -> None:
    header = (
        "P5\n"
        "# demo_warehouse: L-corridor (lane1) + parallel vertical lane (lane2)\n"
        "# 260x140 px @ 0.05 m/px = 13m x 7m; row 0 = top = max y\n"
        f"{WIDTH} {HEIGHT}\n255\n"
    ).encode("ascii")
    body = bytearray()
    for row in grid:
        body.extend(row)
    with open(path, "wb") as handle:
        handle.write(header)
        handle.write(bytes(body))


def main() -> None:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = os.path.join(root, "test", "maps", "demo_warehouse", "map.pgm")
    write_pgm(build(), out)
    print(f"[gen_demo_map] wrote {out} ({WIDTH}x{HEIGHT})")


if __name__ == "__main__":
    main()
