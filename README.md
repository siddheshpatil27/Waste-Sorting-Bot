# Waste-Sorting-Bot
# Waste-sorting pick & place cell – CoppeliaSim EDU 4.1.0

| File | Runs where | Job |
|---|---|---|
| `scene_builder.lua` | CoppeliaSim (customization script, once) | Builds the waste plant, conveyor, 5-DOF arm + suction gripper, camera, proximity sensor, 3 bins |
| `robot_controller.lua` | CoppeliaSim (child script on `PickPlaceRobot`) | Robot / conveyor / gripper "firmware": joint-queue execution, belt, item spawner, sensors, signals |
| `pick_place_dashboard.py` | Your PC | Closed-form IK, pick & place state machine, camera classification, Tk dashboard |

## 1. Build the scene (once)
1. Put the three files in one folder; in `scene_builder.lua` set `CONTROLLER_FILE` to the full path of `robot_controller.lua`.
2. CoppeliaSim → new empty scene → *Add > Dummy* → right-click it → *Add > Associated customization script > Non-threaded*.
3. Paste `scene_builder.lua` into it, press **Restart script**. The console prints `[scene_builder] done`.
   (If auto-attach of the controller fails, add a non-threaded **child** script to `PickPlaceRobot` and paste `robot_controller.lua` into it.)
4. Delete the helper dummy, save as `waste_plant.ttt`.

## 2. Python side
Copy from `<CoppeliaSim>/programming/remoteApiBindings/python/python/` → `sim.py`, `simConst.py`
and from `.../lib/lib/<your OS>/` → `remoteApi.dll` / `.so` / `.dylib` next to `pick_place_dashboard.py`.
Check that `<CoppeliaSim>/remoteApiConnections.txt` has the continuous server on **19997** (the default).

```
python pick_place_dashboard.py            # CoppeliaSim
python pick_place_dashboard.py --mock     # no CoppeliaSim needed
```
Dashboard: **Connect → Start sim → START**. Pause freezes arm + belt, STOP finishes the current cycle,
E-STOP freezes and aborts immediately (Reset E-STOP, then START to recover, a held item is placed first).

## 3. How it works
* Items (blue = PLASTIC, yellow = METAL, green = ORGANIC) are spawned on the belt and carried kinematically.
  The ray proximity sensor across the belt stops the belt when an item reaches the station.
* Python grabs the top-down camera image, classifies by hue, converts the centroid to world x,y
  (image right = +x, image down = +y), and solves IK for hover → contact → lift → arc over bins → drop → ready pose.
* IK is analytic for a vertical tool: `q1 = atan2(dy,dx)`, planar 2-link for `q2,q3`, `q4 = π − q2 − q3`.
  Link lengths and joint limits are read from the simulation, so the scene can be edited freely.
* Python → sim: `arm_path` (packed float waypoints), `arm_path_id`, `grip_cmd`, `belt_*`, `spawn_*`, `estop`.
  Sim → Python: `arm_q`, `arm_done_id`, `grip_state`, `prox_detect`, `scene_cfg`, `hb`, counters.

## 4. If something is off
* Camera pick offset or mirrored: run with `--use-ground-truth` (uses the simulator's true item pose) and compare the
  "camera error" printed in the log; flip signs in `CellController.detect` if needed.
* `createVisionSensor` / `createProximitySensor` warnings in the console: add those two sensors by hand with the names,
  poses and parameters printed in the warning (the controller has a virtual proximity fallback).
