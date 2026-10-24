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
local CONTROLLER_FILE = 'C:/BE/RA/FA2/PickPlaceRobot/robot_controller.lua'

-- ------------------------------ layout (metres) ------------------------------
local BASE      = {-0.60, 0.55}      -- robot pedestal x,y
local BELT_TOP  = 0.40               -- conveyor surface height
local BELT_X0, BELT_X1 = -2.00, 0.40 -- conveyor extents (x)
local BELT_W    = 0.50
local PICK_X    = -0.60              -- pick station (proximity beam + camera)
local PED_H     = 0.45               -- pedestal height  -> joint 1
local SHOULDER  = 0.55               -- joint 2 height
local L1, L2, LT = 0.40, 0.35, 0.15  -- link lengths (upper arm, forearm, tool)
local BIN_R     = 0.55               -- bin distance from robot axis
local BIN_H     = 0.30
local BIN_ANGLES = {PLASTIC = 30, METAL = 90, ORGANIC = 150}   -- deg about robot axis
local BIN_COLORS = {PLASTIC = {0.10,0.30,0.95}, METAL = {0.95,0.80,0.10}, ORGANIC = {0.10,0.70,0.20}}
local LIMITS = {{-3.1, 3.1}, {-1.4, 1.9}, {0.0, 2.8}, {-1.0, 3.0}, {-3.1, 3.1}}  -- rad

-- --------------------------------- helpers -----------------------------------
local function exists(name)
    local ok, h = pcall(sim.getObjectHandle, name)
    return ok and h ~= -1
end

-- type: 0 cuboid, 1 sphere, 2 cylinder.  size = {x,y,z}  (sphere/cyl: dia,dia,h)
local function shape(name, ptype, size, pos, color, static, respondable, euler)
    local opts = 0
    if respondable ~= false then opts = opts + 8 end
    if static ~= false then opts = opts + 16 end
    local h = sim.createPureShape(ptype, opts, size, 1.0, nil)
    sim.setObjectName(h, name)
    sim.setObjectPosition(h, -1, pos)
    if euler then sim.setObjectOrientation(h, -1, euler) end
    sim.setShapeColor(h, nil, 0, color)
    return h
end
local function box(n, s, p, c, st, rs, e) return shape(n, 0, s, p, c, st, rs, e) end
local function cyl(n, d, hgt, p, c, st, rs, e) return shape(n, 2, {d, d, hgt}, p, c, st, rs, e) end
local function ball(n, d, p, c, st, rs) return shape(n, 1, {d, d, d}, p, c, st, rs) end

local function dummy(name, pos, size, color, euler)
    local h = sim.createDummy(size or 0.05)
    sim.setObjectName(h, name)
    sim.setObjectPosition(h, -1, pos)
    if euler then sim.setObjectOrientation(h, -1, euler) end
    if color then sim.setObjectColor(h, 0, sim.colorcomponent_ambient_diffuse, color) end
    return h
end

local function parent(child, par) sim.setObjectParent(child, par, true) end

local function joint(name, pos, euler, lim)
    local j = sim.createJoint(sim.joint_revolute_subtype, sim.jointmode_passive, 0, {0.03, 0.10})
    sim.setObjectName(j, name)
    sim.setObjectPosition(j, -1, pos)
    sim.setObjectOrientation(j, -1, euler)
    sim.setJointInterval(j, false, {lim[1], lim[2] - lim[1]})
    return j
end

local function undetectable(h) pcall(sim.setObjectSpecialProperty, h, 0) end

-- ------------------------------ plant scenery --------------------------------
local function buildPlant()
    local grey, dark, steel = {0.55,0.56,0.58}, {0.18,0.18,0.20}, {0.62,0.65,0.70}
    -- floor + marked work cell
    box('WP_Floor', {9, 7, 0.10}, {0, 0, -0.05}, {0.45,0.46,0.48})
    box('WP_CellMark_N', {2.6, 0.04, 0.004}, {-0.7,  1.65, 0.002}, {0.95,0.80,0.05}, true, false)
    box('WP_CellMark_S', {2.6, 0.04, 0.004}, {-0.7, -0.75, 0.002}, {0.95,0.80,0.05}, true, false)
    box('WP_CellMark_E', {0.04, 2.4, 0.004}, { 0.6,  0.45, 0.002}, {0.95,0.80,0.05}, true, false)
    box('WP_CellMark_W', {0.04, 2.4, 0.004}, {-2.0,  0.45, 0.002}, {0.95,0.80,0.05}, true, false)
    -- walls (open towards +y... camera side) and roof beams
    box('WP_WallBack',  {9, 0.2, 4.0}, {0, -3.4, 2.0}, {0.74,0.75,0.72})
    box('WP_WallLeft',  {0.2, 7, 4.0}, {-4.4, 0, 2.0}, {0.70,0.72,0.70})
    box('WP_WallRight', {0.2, 7, 4.0}, { 4.4, 0, 2.0}, {0.70,0.72,0.70})
    for i = 0, 3 do   -- high windows on the back wall
        box('WP_Window'..i, {1.0, 0.05, 0.7}, {-3.0 + 2.0*i, -3.28, 2.9}, {0.45,0.65,0.85}, true, false)
    end
    for i = 0, 4 do   -- roof trusses
        box('WP_Truss'..i, {0.15, 7, 0.20}, {-4.0 + 2.0*i, 0, 4.0}, {0.30,0.32,0.36}, true, false)
    end
    for i = 0, 2 do
        box('WP_Purlin'..i, {9, 0.12, 0.12}, {0, -2.4 + 2.4*i, 3.85}, {0.30,0.32,0.36}, true, false)
    end
    -- hazard stripe along back wall
    for i = 0, 17 do
        local c = (i % 2 == 0) and {0.95,0.80,0.05} or {0.1,0.1,0.1}
        box('WP_Stripe'..i, {0.5, 0.02, 0.15}, {-4.25 + 0.5*i, -3.29, 0.30}, c, true, false)
    end
    -- storage tanks, chimney, pipes (background)
    cyl('WP_Tank1', 1.4, 3.0, {3.2, -2.3, 1.5}, {0.78,0.80,0.82})
    cyl('WP_Tank2', 1.1, 2.4, {1.8, -2.5, 1.2}, {0.70,0.74,0.80})
    cyl('WP_Chimney', 0.55, 4.0, {-3.6, -2.8, 2.0}, {0.55,0.30,0.25})
    cyl('WP_ChimneyTop', 0.62, 0.15, {-3.6, -2.8, 4.05}, {0.15,0.15,0.15}, true, false)
    cyl('WP_Pipe1', 0.18, 6.0, {1.0, -3.1, 2.3}, steel, true, false, {0, math.pi/2, 0})
    cyl('WP_Pipe2', 0.12, 5.0, {2.2, -3.1, 2.0}, {0.75,0.30,0.20}, true, false, {0, math.pi/2, 0})
    -- feed end: waste heap (static spheres) + skip container
    local math_r = math.random
    math.randomseed(7)
    local heap = {{0.45,0.35,0.25},{0.25,0.30,0.35},{0.55,0.50,0.30},{0.20,0.20,0.20},{0.60,0.25,0.20}}
    for i = 1, 22 do
        local d = 0.22 + 0.25*math_r()
        ball('WP_Heap'..i, d, {-2.75 + 0.9*(math_r()-0.5)*2, 0.2+1.4*(math_r()-0.5)*2, d*0.5 + 0.18*math_r()},
             heap[math_r(#heap)], true, false)
    end
    box('WP_Skip', {1.3, 1.9, 0.9}, {-3.5, 1.9, 0.45}, {0.15,0.35,0.55})
    box('WP_SkipRim', {1.4, 2.0, 0.06}, {-3.5, 1.9, 0.93}, {0.12,0.28,0.45}, true, false)
    -- waste bags near the right wall
    for i = 1, 8 do
        ball('WP_Bag'..i, 0.45, {3.6 + 0.25*(i%2), -1.0 + 0.5*i, 0.22}, {0.08,0.08,0.10}, true, false)
    end
    -- control cabinet + beacon
    box('WP_Cabinet', {0.6, 0.35, 1.5}, {0.85, 1.05, 0.75}, {0.85,0.86,0.88})
    box('WP_CabinetPanel', {0.5, 0.02, 0.5}, {0.85, 0.87, 1.0}, {0.10,0.35,0.15}, true, false)
    cyl('WP_BeaconPost', 0.04, 0.3, {0.85, 1.05, 1.65}, {0.2,0.2,0.2}, true, false)
    cyl('WP_Beacon', 0.12, 0.14, {0.85, 1.05, 1.85}, {1.0,0.45,0.05}, true, false)
    -- signs
    box('WP_SignWarn', {0.5, 0.03, 0.4}, {-1.2, -3.27, 1.4}, {0.95,0.80,0.05}, true, false)
    box('WP_SignPPE',  {0.5, 0.03, 0.4}, {-0.4, -3.27, 1.4}, {0.10,0.35,0.75}, true, false)
    box('WP_SignExit', {0.6, 0.03, 0.25}, {0.5, -3.27, 1.9}, {0.10,0.65,0.25}, true, false)
end

-- ------------------------------- conveyor ------------------------------------
local function buildConveyor()
    local len = BELT_X1 - BELT_X0
    local cx = (BELT_X0 + BELT_X1) * 0.5
    box('Conveyor_Belt', {len, BELT_W, 0.04}, {cx, 0, BELT_TOP - 0.02}, {0.17,0.17,0.19})
    for i = 0, 22 do   -- belt cleats (visual only)
        box('Conveyor_Cleat'..i, {0.03, BELT_W - 0.02, 0.006}, {BELT_X0 + 0.1 + i*0.1, 0, BELT_TOP + 0.003}, {0.25,0.25,0.28}, true, false)
    end
    for _, s in ipairs({-1, 1}) do   -- low side rails
        box('Conveyor_Rail'..s, {len, 0.02, 0.03}, {cx, s*(BELT_W*0.5 + 0.01), BELT_TOP + 0.015}, {0.60,0.60,0.64})
    end
    box('Conveyor_Frame', {len, BELT_W + 0.06, 0.10}, {cx, 0, BELT_TOP - 0.09}, {0.25,0.28,0.32})
    for _, x in ipairs({BELT_X0 + 0.1, cx, BELT_X1 - 0.1}) do
        for _, y in ipairs({-0.25, 0.25}) do
            cyl('Conveyor_Leg', 0.06, BELT_TOP - 0.14, {x, y, (BELT_TOP - 0.14)*0.5}, {0.3,0.3,0.34})
        end
    end
    cyl('Conveyor_RollerA', 0.10, BELT_W + 0.04, {BELT_X0, 0, BELT_TOP - 0.05}, {0.5,0.5,0.55}, true, false, {math.pi/2, 0, 0})
    cyl('Conveyor_RollerB', 0.10, BELT_W + 0.04, {BELT_X1, 0, BELT_TOP - 0.05}, {0.5,0.5,0.55}, true, false, {math.pi/2, 0, 0})
    -- feed hopper
    box('Hopper_Back',  {0.04, 0.60, 0.45}, {BELT_X0 - 0.05, 0, BELT_TOP + 0.22}, {0.60,0.62,0.66}, true, false)
    box('Hopper_Left',  {0.40, 0.03, 0.45}, {BELT_X0 + 0.15,  0.31, BELT_TOP + 0.22}, {0.60,0.62,0.66}, true, false)
    box('Hopper_Right', {0.40, 0.03, 0.45}, {BELT_X0 + 0.15, -0.31, BELT_TOP + 0.22}, {0.60,0.62,0.66}, true, false)
    -- markers used by the controller
    dummy('Belt_Spawn',    {BELT_X0 + 0.20, 0, BELT_TOP + 0.04}, 0.04, {0,1,0})
    dummy('Pick_Station',  {PICK_X, 0, BELT_TOP}, 0.04, {1,0,0})
    dummy('Belt_Limits',   {BELT_X1, 0, BELT_TOP}, 0.04, {1,0,1})
end

-- ----------------------------- sensors + camera ------------------------------
local function buildSensors()
    -- gantry for the camera (posts stay clear of the proximity ray)
    local camZ = 1.00
    for _, s in ipairs({-1, 1}) do
        cyl('Gantry_Post', 0.05, camZ, {PICK_X + 0.38, s*0.42, camZ*0.5}, {0.25,0.28,0.32})
    end
    box('Gantry_Beam', {0.05, 0.9, 0.05}, {PICK_X + 0.38, 0, camZ}, {0.25,0.28,0.32}, true, false)
    box('Gantry_Arm',  {0.40, 0.05, 0.05}, {PICK_X + 0.18, 0, camZ}, {0.25,0.28,0.32}, true, false)

    -- orthographic top-down vision sensor (looks along its local +z)
    local ok, cam = pcall(sim.createVisionSensor, 0, {128, 128, 0, 0},
                          {0.02, 2.0, 0.5, 0.05, 0.05, 0.05, 0, 0, 0, 0, 0})
    if not ok then
        print('[scene_builder] WARNING: createVisionSensor failed: ' .. tostring(cam))
        print('   -> add Add > Vision sensor > Orthographic manually, name it Belt_Camera,')
        print('      resolution 128x128, ortho size 0.5, pos ('..PICK_X..',0,'..camZ..'), orientation (180,0,0) deg.')
        return
    end
    sim.setObjectName(cam, 'Belt_Camera')
    pcall(sim.setObjectInt32Parameter, cam, sim.visionintparam_perspective_operation, 0)
    pcall(sim.setObjectFloatParameter, cam, sim.visionfloatparam_ortho_size, 0.5)
    pcall(sim.setObjectInt32Parameter, cam, sim.visionintparam_resolution_x, 128)
    pcall(sim.setObjectInt32Parameter, cam, sim.visionintparam_resolution_y, 128)
    sim.setObjectPosition(cam, -1, {PICK_X, 0, camZ})
    sim.setObjectOrientation(cam, -1, {math.pi, 0, 0})    -- image right = +x, image down = +y

    -- ray proximity sensor beside the belt, beam crosses the belt at z = BELT_TOP+0.05
    local ok2, ps = pcall(sim.createProximitySensor, sim.proximitysensor_ray_subtype, 16, 0,
        {1, 1, 1, 1, 1, 1, 1, 1},
        {0.0, 0.55, 0.01, 0.01, 0.01, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
    if not ok2 then
        print('[scene_builder] WARNING: createProximitySensor failed: ' .. tostring(ps))
        print('   -> add Add > Proximity sensor > Ray type manually, name it Belt_ProxSensor,')
        print('      range 0.55, pos ('..PICK_X..',0.33,'..(BELT_TOP+0.05)..'), orientation (90,0,0) deg.')
        print('      (The controller falls back to a virtual sensor meanwhile.)')
        return
    end
    sim.setObjectName(ps, 'Belt_ProxSensor')
    sim.setObjectPosition(ps, -1, {PICK_X, 0.33, BELT_TOP + 0.05})
    sim.setObjectOrientation(ps, -1, {math.pi/2, 0, 0})   -- local +z -> world -y
    box('Belt_ProxHousing', {0.05, 0.05, 0.05}, {PICK_X, 0.36, BELT_TOP + 0.05}, {0.10,0.10,0.10}, true, false)
end

-- ------------------------------- sorting bins --------------------------------
local function buildBins()
    local bx, by = BASE[1], BASE[2]
    for name, ang in pairs(BIN_ANGLES) do
        local a = math.rad(ang)
        local x, y = bx + BIN_R*math.cos(a), by + BIN_R*math.sin(a)
        local c = BIN_COLORS[name]
        local w, t = 0.30, 0.02
        box('Bin_'..name..'_Floor', {w, w, t}, {x, y, t*0.5}, c)
        box('Bin_'..name..'_W1', {w, t, BIN_H}, {x, y + w*0.5, BIN_H*0.5}, c)
        box('Bin_'..name..'_W2', {w, t, BIN_H}, {x, y - w*0.5, BIN_H*0.5}, c)
        box('Bin_'..name..'_W3', {t, w, BIN_H}, {x + w*0.5, y, BIN_H*0.5}, c)
        box('Bin_'..name..'_W4', {t, w, BIN_H}, {x - w*0.5, y, BIN_H*0.5}, c)
        dummy('Drop_'..name, {x, y, BIN_H + 0.14}, 0.04, c)
    end
end

-- --------------------------------- the arm -----------------------------------
local function buildArm()
    local bx, by = BASE[1], BASE[2]
    local root = dummy('PickPlaceRobot', {bx, by, 0.0}, 0.10, {0.1,0.6,1.0})
    local ped = cyl('Arm_Pedestal', 0.26, PED_H, {bx, by, PED_H*0.5}, {0.25,0.28,0.32})
    box('Arm_BasePlate', {0.5, 0.5, 0.03}, {bx, by, 0.015}, {0.2,0.22,0.25})
    parent(ped, root)

    local ex = {-math.pi/2, 0, 0}   -- joint z axis -> world +y  (positive q tilts the arm forward, +x)
    local j1 = joint('Arm_J1', {bx, by, PED_H}, {0, 0, 0}, LIMITS[1])
    local j2 = joint('Arm_J2', {bx, by, SHOULDER}, ex, LIMITS[2])
    local j3 = joint('Arm_J3', {bx, by, SHOULDER + L1}, ex, LIMITS[3])
    local j4 = joint('Arm_J4', {bx, by, SHOULDER + L1 + L2}, ex, LIMITS[4])
    local j5 = joint('Arm_J5', {bx, by, SHOULDER + L1 + L2 + 0.05}, {0, 0, 0}, LIMITS[5])

    local orange, grey = {0.95,0.45,0.10}, {0.80,0.80,0.82}
    local turret = cyl('Arm_Turret', 0.20, SHOULDER - PED_H, {bx, by, (PED_H + SHOULDER)*0.5}, grey, false, false)
    local link1  = box('Arm_Link1', {0.08, 0.08, L1}, {bx, by, SHOULDER + L1*0.5}, orange, false, false)
    local elbow  = cyl('Arm_ElbowCap', 0.10, 0.12, {bx, by, SHOULDER + L1}, grey, false, false, {math.pi/2, 0, 0})
    local link2  = box('Arm_Link2', {0.065, 0.065, L2}, {bx, by, SHOULDER + L1 + L2*0.5}, orange, false, false)
    local wrist  = cyl('Arm_WristCap', 0.08, 0.10, {bx, by, SHOULDER + L1 + L2}, grey, false, false, {math.pi/2, 0, 0})
    local link3  = box('Arm_Link3', {0.05, 0.05, 0.10}, {bx, by, SHOULDER + L1 + L2 + 0.05}, {0.3,0.3,0.34}, false, false)
    local cup    = cyl('Arm_SuctionCup', 0.06, 0.05, {bx, by, SHOULDER + L1 + L2 + LT - 0.025}, {0.1,0.1,0.1}, false, false)
    local tip    = dummy('Arm_Tip', {bx, by, SHOULDER + L1 + L2 + LT}, 0.03, {1,0,0})

    -- hierarchy:  root > J1 > (turret, J2 > (link1, elbow, J3 > (link2, wrist, J4 > (link3, J5 > (cup, tip)))))
    parent(j1, root)
    parent(turret, j1); parent(j2, j1)
    parent(link1, j2);  parent(elbow, j2);  parent(j3, j2)
    parent(link2, j3);  parent(wrist, j3);  parent(j4, j3)
    parent(link3, j4);  parent(j5, j4)
    parent(cup, j5);    parent(tip, j5)
    for _, h in ipairs({turret, link1, elbow, link2, wrist, link3, cup}) do undetectable(h) end
    return root
end

-- ------------------------------ controller script ----------------------------
local function attachController(root)
    local f = io.open(CONTROLLER_FILE, 'r')
    if not f then
        print('[scene_builder] Could not open ' .. CONTROLLER_FILE)
        print('   -> right-click dummy "PickPlaceRobot" > Add > Associated child script > Non-threaded')
        print('      and paste robot_controller.lua into it.')
        return
    end
    local text = f:read('*a'); f:close()
    local ok, err = pcall(function()
        local s = sim.addScript(sim.scripttype_childscript)
        sim.associateScriptWithObject(s, root)
        sim.setScriptStringParam(s, sim.scriptstringparam_text, text)
    end)
    if ok then print('[scene_builder] controller script attached to PickPlaceRobot')
    else print('[scene_builder] auto-attach failed (' .. tostring(err) .. '): paste robot_controller.lua manually.') end
end

-- ----------------------------------- main ------------------------------------
function sysCall_init()
    if exists('WP_Floor') then
        print('[scene_builder] scene already built - nothing to do.')
        return
    end
    print('[scene_builder] building waste plant ...')
    buildPlant()
    buildConveyor()
    buildSensors()
    buildBins()
    local root = buildArm()
    attachController(root)
    print('[scene_builder] done. Save the scene (File > Save scene as...).')
end

function sysCall_cleanup() end
