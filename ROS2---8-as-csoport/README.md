# Pályatervezők összehasonlítása – KUKA KR 6 R900-2

ROS 2 Humble + MoveIt, szimulált (mock) robottal, RViz megjelenítéssel.
A felépítés a [kroshu/education moveit_example](https://github.com/kroshu/education/tree/master/VIIIAV55/moveit_example) példáját követi,
csak a robot (KR 6 R900-2) és a jelenet (a cella) más.

## Felépítés

`kr6_cell` csomag:

| Fájl | Tartalom |
|---|---|
| `launch/startup.launch.py` | KUKA driver (mock) + MoveIt `move_group` (a KUKA hivatalos `kuka_kr_moveit_config` konfigurációjával) + RViz |
| `rviz/kr6_cell.rviz` | RViz beállítás: egyetlen robot (RobotModel), a cella (PlanningScene) és a program markerei (MarkerArray) |
| `include/kr6_cell/utils.hpp` | `Setup()`: a cella (szerszámgép, asztal, munkadarab, futószalag) ütközési objektumként; `MoveToPose()`, `MoveToHome()`: mindkettő a `PlanAndExecute()`-ot hívja (felirat, pálya, célpóz, Next, végrehajtás) |
| `src/kr6_cell.cpp` | A program: home → szerszámgép elé → home → futószalag fölé → home |

## Indítás (a devcontainerben)

```bash
cd ~/ros2_ws
colcon build --packages-up-to kr6_cell
source install/setup.bash

# 1. terminál: robot + MoveIt + RViz
ros2 launch kr6_cell startup.launch.py

# 2. terminál: a robot aktiválása (enélkül a robot nem mozdul)
ros2 lifecycle set robot_manager configure
ros2 lifecycle set robot_manager activate

# 2. terminál: a program; az RViz-ben a "Next" gombbal lehet léptetni
ros2 run kr6_cell kr6_cell

# vagy automatikusan, "Next" nélkül
ros2 run kr6_cell kr6_cell --ros-args -p auto_run:=true
```

Minden mozgásnál ugyanaz történik: felül megjelenik a lépés neve, a tervezett pálya vonalként
és a célpóz (`target` tengelykereszt) kirajzolódik, majd "Next"-re (automata módban 1 s után) a robot végrehajtja.

Ha az RViz nem tud megnyílni (`could not connect to display`), a hoston: `xhost +local:`

## Pályatervezők

A tervezőt a `src/kr6_cell.cpp`-ben lehet beállítani:
```cpp
move_group_interface.setPlanningPipelineId("ompl");
move_group_interface.setPlannerId("RRTConnectkConfigDefault");
```

| Pipeline | Tervezők (`planner_id`) |
|---|---|
| `ompl` | `RRTConnectkConfigDefault`, `RRTkConfigDefault`, `RRTstarkConfigDefault`, `PRMkConfigDefault`, `PRMstarkConfigDefault`, `BiTRRTkConfigDefault`, `KPIECEkConfigDefault`, … (lásd `kuka_kr_moveit_config/config/ompl_planning.yaml`) |
| `pilz_industrial_motion_planner` | `PTP`, `LIN`, `CIRC` |
| `chomp` | `CHOMP` |

A pályaidőt a csuklók sebesség- és gyorsulási határai adják: `kuka_agilus_support/config/kr6_r900_2_joint_limits.yaml` (KUKA gyári értékek).
