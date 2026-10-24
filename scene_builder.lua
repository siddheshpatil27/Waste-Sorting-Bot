-- =============================================================================
--  scene_builder.lua   --   CoppeliaSim EDU 4.1.0   (SCENERY + HARDWARE MODEL)
--
--  Builds, from primitives only (no external models needed):
--    * a waste-collection / sorting plant (floor, walls, tanks, chimney, waste
--      heaps, safety markings, control cabinet, warning signs, beacon light)
--    * a conveyor belt with a feed hopper
--    * a 5-DOF pick & place arm with a suction gripper (all joints are named
--      Arm_J1 ... Arm_J5, tool centre point dummy = Arm_Tip)
--    * a top-down orthographic vision sensor   (Belt_Camera)
--    * a ray proximity sensor across the belt   (Belt_ProxSensor)
--    * three sorting bins  (PLASTIC / METAL / ORGANIC) with drop-target dummies
--    * the robot controller child script, attached to dummy "PickPlaceRobot"
--
--  HOW TO RUN IT  (once; then save the scene as waste_plant.ttt)
--    1. Open an EMPTY scene (File > New scene).
--    2. Add > Dummy.  Right-click the dummy > Add > Associated customization
--       script > Non-threaded.  Double-click the little script icon, delete the
--       default text, paste this whole file, then press the "Restart script"
--       button of the script editor (or save + re-open the scene).
--    3. Look at the status bar / console: "[scene_builder] done".
--    4. Delete the helper dummy + its script (or leave it, it is harmless: the
--       builder does nothing when the scene already exists).
--    5. File > Save scene as... waste_plant.ttt
--
--  Set CONTROLLER_FILE to the absolute path of robot_controller.lua so the
--  builder can attach it automatically.  If that fails, it tells you to paste
--  robot_controller.lua manually into a child script of "PickPlaceRobot".
-- =============================================================================

-- >>> EDIT ME (use forward slashes, also on Windows) <<<
-- Leave empty to auto-discover robot_controller.lua in the same folder as the scene.
-- If your scene and controller are not in the same folder, set the full path here.
local CONTROLLER_FILE = ''

local function normalizePath(path)
    if not path or path == '' then return '' end
    return path:gsub('\\', '/')
end

local function candidatePaths()
    local out = {}
    local add = function(p)
        if not p or p == '' then return end
        p = normalizePath(p)
        if not table.contains then
            -- table.contains is not a standard Lua API in CoppeliaSim, so keep only unique paths manually.
        end
        local seen = {}
        for _, v in ipairs(out) do seen[v] = true end
        if not seen[p] then out[#out + 1] = p end
    end

    if CONTROLLER_FILE and CONTROLLER_FILE ~= '' then add(CONTROLLER_FILE) end

    if sim.getStringParam then
        local scenePath = sim.getStringParam(sim.stringparam_scene_path)
        if scenePath and scenePath ~= '' then
            local dir = scenePath:match('^(.*)[/\\][^/\\]+$')
            if dir and dir ~= '' then add(dir .. '/robot_controller.lua') end
            add(scenePath:match('^(.*)[/\\][^/\\]+$') .. '/robot_controller.lua')
        end

        local appPath = sim.getStringParam(sim.stringparam_app_path)
        if appPath and appPath ~= '' then
            add(appPath .. '/robot_controller.lua')
        end
    end

    return out
end

local function tryFindControllerFile()
    for _, p in ipairs(candidatePaths()) do
        local f = io.open(p, 'r')
        if f then f:close(); return p end
    end
    return nil
end

-- ------------------------------ layout (metres) ------------------------------
local BASE      = {-0.60, 0.55}      -- robot pedestal x,y
