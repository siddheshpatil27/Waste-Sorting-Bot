-- =============================================================================
--  robot_controller.lua  --  CoppeliaSim EDU 4.1.0   (ROBOT + CONVEYOR "BOT" CODE)
--
--  NON-THREADED CHILD SCRIPT attached to the dummy "PickPlaceRobot"
--  (scene_builder.lua attaches it automatically, or paste it there by hand).
--
--  This script is the low-level "firmware" of the cell.  It does NOT solve
--  inverse kinematics - the Python program does that and streams joint-space
--  waypoints.  This script:
--     * executes the joint waypoint queue with a joint-speed limit
--     * runs the conveyor (items ride the belt kinematically, belt stops
--       automatically when the proximity sensor sees an item at the station)
--     * spawns coloured waste items (blue=PLASTIC, yellow=METAL, green=ORGANIC)
--     * implements the suction gripper (attach / release the nearest item)
--     * publishes telemetry and reads commands through CoppeliaSim signals
--
--  SIGNAL INTERFACE  (legacy remote API, port 19997 by default)
--   Commands  (Python -> sim)
--     belt_run (int 0/1)        belt_speed (float m/s)     spawn_enable (int 0/1)
--     spawn_interval (float s)  estop (int 0/1, freezes arm + belt)
--     arm_speed_scale (float)   arm_path (string: packed floats, 5 per waypoint)
--     arm_path_id (int)         grip_cmd (int 0 release / 1 grab)
--     discard_item (int, pulse) removes the item waiting at the station
--   Telemetry (sim -> Python)
--     hb (int heartbeat)  sim_time (float)  arm_q (string, 5 floats)
--     arm_done_id (int)   grip_state (int)  prox_detect (int)  belt_moving (int)
--     cnt_spawned / cnt_missed (int)        station_gt (string: x,y,z,class)
--     scene_cfg (string)  joint_limits (string)
-- =============================================================================

local VMAX      = {1.8, 1.4, 1.6, 2.2, 3.5}   -- joint speed limits [rad/s]
local ITEM_SIZE = 0.08                        -- waste cube edge [m]
local GRAB_RADIUS = 0.075                     -- suction reach [m]
local BELT_Y_HALF = 0.25
local CLASSES = {                             -- id, name, rgb
    {1, 'PLASTIC', {0.10, 0.30, 0.95}},
    {2, 'METAL',   {0.95, 0.80, 0.10}},
    {3, 'ORGANIC', {0.10, 0.70, 0.20}},
}

-- ---------------------------------------------------------------- helpers ---
local function gi(n, d) local v = sim.getIntegerSignal(n); if v == nil then return d end return v end
local function gf(n, d) local v = sim.getFloatSignal(n);   if v == nil then return d end return v end

local function try(name)
    local ok, h = pcall(sim.getObjectHandle, name)
    if ok then return h end
    return -1
end

function sysCall_init()
    J = {}
    for i = 1, 5 do J[i] = sim.getObjectHandle('Arm_J' .. i) end
    tip     = sim.getObjectHandle('Arm_Tip')
    cam     = try('Belt_Camera')
    prox    = try('Belt_ProxSensor')
    pickDum = sim.getObjectHandle('Pick_Station')
    spawnDum= sim.getObjectHandle('Belt_Spawn')
    limDum  = sim.getObjectHandle('Belt_Limits')

    local pp = sim.getObjectPosition(pickDum, -1)
    local sp = sim.getObjectPosition(spawnDum, -1)
    local lp = sim.getObjectPosition(limDum, -1)
    PICK_X, BELT_TOP = pp[1], pp[3]
    SPAWN_X, BELT_X1 = sp[1], lp[1]

    -- joint limits -> telemetry
    local lims = {}
    for i = 1, 5 do
        local _, iv = sim.getJointInterval(J[i])
        lims[#lims + 1] = iv[1]; lims[#lims + 1] = iv[1] + iv[2]
    end
    LIM = lims
    sim.setStringSignal('joint_limits', sim.packFloatTable(lims))

    -- scene geometry -> telemetry (the Python IK reads link lengths from here)
    local b  = sim.getObjectPosition(J[1], -1)
    local p2 = sim.getObjectPosition(J[2], -1)
    local p3 = sim.getObjectPosition(J[3], -1)
    local p4 = sim.getObjectPosition(J[4], -1)
    local tp = sim.getObjectPosition(tip,  -1)
    local cfg = {b[1], b[2], p2[3],
                 p3[3] - p2[3], p4[3] - p3[3], tp[3] - p4[3],
                 BELT_TOP, ITEM_SIZE, PICK_X, pp[2]}
    local cp = {0, 0}
    if cam ~= -1 then cp = sim.getObjectPosition(cam, -1) end
    cfg[#cfg + 1] = cp[1]; cfg[#cfg + 1] = cp[2]; cfg[#cfg + 1] = 0.5   -- ortho image size
    for _, n in ipairs({'PLASTIC', 'METAL', 'ORGANIC'}) do
        local d = sim.getObjectPosition(sim.getObjectHandle('Drop_' .. n), -1)
        cfg[#cfg + 1] = d[1]; cfg[#cfg + 1] = d[2]; cfg[#cfg + 1] = d[3]
    end
    sim.setStringSignal('scene_cfg', sim.packFloatTable(cfg))

    -- state
    q = {0, 0, 0, 0, 0}
    for i = 1, 5 do sim.setJointPosition(J[i], 0) end
    wps, wpi = {}, 1
    lastPathId = gi('arm_path_id', 0)
    doneId = lastPathId
    items, itemCount = {}, 0
    held = nil
    stationItem = nil
    spawnTimer = 2.0
    cntSpawned, cntMissed = 0, 0
    steps = 0
    lastDiscard = 0
    math.randomseed(12345)

    sim.setIntegerSignal('arm_done_id', doneId)
    sim.setIntegerSignal('grip_state', 0)
    sim.setIntegerSignal('prox_detect', 0)
    sim.setIntegerSignal('belt_moving', 0)
    sim.setIntegerSignal('cnt_spawned', 0)
    sim.setIntegerSignal('cnt_missed', 0)
    sim.setIntegerSignal('estop', 0)
    sim.setStringSignal('station_gt', sim.packFloatTable({0, 0, 0, 0}))
    print('[robot_controller] ready.  prox sensor: ' .. (prox ~= -1 and 'real' or 'virtual (fallback)')
          .. ', camera: ' .. (cam ~= -1 and 'yes' or 'MISSING'))
end

-- ------------------------------------------------------------------- items ---
local function spawnItem()
    local cls = CLASSES[math.random(#CLASSES)]
    itemCount = itemCount + 1
    local h = sim.createPureShape(0, 8 + 16, {ITEM_SIZE, ITEM_SIZE, ITEM_SIZE}, 0.2, nil)  -- respondable + static
    sim.setObjectName(h, 'Waste_' .. itemCount)
    sim.setShapeColor(h, nil, 0, cls[3])
    sim.setObjectPosition(h, -1, {SPAWN_X, (math.random() - 0.5) * 0.16, BELT_TOP + ITEM_SIZE * 0.5 + 0.002})
    items[h] = {h = h, cls = cls[1], name = cls[2], state = 'belt', t = 0}
    cntSpawned = cntSpawned + 1
end

local function removeItem(it)
    pcall(sim.removeObject, it.h)
    items[it.h] = nil
    if stationItem == it then stationItem = nil end
    if held == it then held = nil end
end

local function spawnerStep(dt)
    if gi('spawn_enable', 1) == 0 then return end
    spawnTimer = spawnTimer - dt
    if spawnTimer > 0 then return end
    -- only spawn if the spawn zone is free
    for _, it in pairs(items) do
        if it.state == 'belt' then
            local p = sim.getObjectPosition(it.h, -1)
            if math.abs(p[1] - SPAWN_X) < 0.25 then return end
        end
    end
    spawnItem()
    spawnTimer = gf('spawn_interval', 6.0)
end

local function conveyorStep(dt, estop)
    local run   = gi('belt_run', 1) == 1
    local speed = gf('belt_speed', 0.12)
    local halted = estop or (not run) or (stationItem ~= nil)
    local moving = false
    for _, it in pairs(items) do
        if it.state == 'belt' then
            if not halted then
                local p = sim.getObjectPosition(it.h, -1)
                p[1] = p[1] + speed * dt
                sim.setObjectPosition(it.h, -1, p)
                moving = true
                if p[1] > BELT_X1 then      -- fell off the end: missed
                    it.state = 'missed'; it.t = 0
                    sim.setObjectInt32Parameter(it.h, sim.shapeintparam_static, 0)
                    sim.resetDynamicObject(it.h)
                    cntMissed = cntMissed + 1
                end
            end
        elseif it.state == 'missed' or it.state == 'dropped' then
            it.t = it.t + dt
            if it.t > 3.0 then removeItem(it) end
        end
    end
    sim.setIntegerSignal('belt_moving', moving and 1 or 0)
end

-- proximity sensor -> hold the belt when an item reaches the station
local function stationLogic()
    local detected = false
    local hit = nil
    if prox ~= -1 then
        local r, dist, pt, hnd = sim.readProximitySensor(prox)
        if r > 0 and hnd and items[hnd] and items[hnd].state == 'belt' then
            detected = true; hit = items[hnd]
        end
    else  -- virtual fallback
        for _, it in pairs(items) do
            if it.state == 'belt' and math.abs(sim.getObjectPosition(it.h, -1)[1] - PICK_X) < 0.01 then
                detected = true; hit = it
            end
        end
    end
    if detected then stationItem = hit
    elseif stationItem and stationItem.state ~= 'belt' then stationItem = nil end
    sim.setIntegerSignal('prox_detect', stationItem and 1 or 0)
    if stationItem then
        local p = sim.getObjectPosition(stationItem.h, -1)
        sim.setStringSignal('station_gt', sim.packFloatTable({p[1], p[2], p[3], stationItem.cls}))
    end
end

-- ----------------------------------------------------------------- gripper ---
local function gripperStep()
    local cmd = gi('grip_cmd', 0)
    if cmd == 1 and held == nil then
        local tp = sim.getObjectPosition(tip, -1)
        local best, bd = nil, GRAB_RADIUS
        for _, it in pairs(items) do
            if it.state == 'belt' then
                local p = sim.getObjectPosition(it.h, -1)
                local d = math.sqrt((p[1]-tp[1])^2 + (p[2]-tp[2])^2 + (p[3]-tp[3])^2)
                if d < bd then best, bd = it, d end
            end
        end
        if best then
            sim.setObjectParent(best.h, tip, true)
            best.state = 'held'; held = best
            if stationItem == best then stationItem = nil end
        end
    elseif cmd == 0 and held ~= nil then
        sim.setObjectParent(held.h, -1, true)
        sim.setObjectInt32Parameter(held.h, sim.shapeintparam_static, 0)   -- now dynamic: falls into the bin
        sim.resetDynamicObject(held.h)
        held.state = 'dropped'; held.t = 0
        held = nil
    end
    sim.setIntegerSignal('grip_state', held and 1 or 0)
end

-- --------------------------------------------------------------------- arm ---
local function armStep(dt, estop)
    local id = gi('arm_path_id', 0)
    if id ~= lastPathId then
        lastPathId = id
        wps, wpi = {}, 1
        local s = sim.getStringSignal('arm_path')
        if s and #s >= 20 then
            local t = sim.unpackFloatTable(s)
            for k = 1, #t - 4, 5 do wps[#wps + 1] = {t[k], t[k+1], t[k+2], t[k+3], t[k+4]} end
        end
        if #wps == 0 then doneId = id end
    end
    if estop or wps[wpi] == nil then
        if wps[wpi] == nil and doneId ~= lastPathId then doneId = lastPathId end
    else
        local remain = dt * math.max(0.05, gf('arm_speed_scale', 1.0))
        while remain > 1e-9 and wps[wpi] do
            local tg = wps[wpi]
            local need = 0
            for i = 1, 5 do
                need = math.max(need, math.abs(tg[i] - q[i]) / VMAX[i])
            end
            if need <= remain then
                for i = 1, 5 do q[i] = tg[i] end
                remain = remain - need
                wpi = wpi + 1
            else
                local f = remain / need
                for i = 1, 5 do q[i] = q[i] + (tg[i] - q[i]) * f end
                remain = 0
            end
        end
        if wps[wpi] == nil then doneId = lastPathId end
    end
    for i = 1, 5 do
        q[i] = math.max(LIM[2*i-1], math.min(LIM[2*i], q[i]))
        sim.setJointPosition(J[i], q[i])
    end
    sim.setIntegerSignal('arm_done_id', doneId)
    sim.setStringSignal('arm_q', sim.packFloatTable(q))
end

-- ------------------------------------------------------------- sim callbacks ---
function sysCall_actuation()
    local dt = sim.getSimulationTimeStep()
    local estop = gi('estop', 0) == 1

    local d = gi('discard_item', 0)
    if d ~= lastDiscard then
        lastDiscard = d
        if stationItem then
            local it = stationItem
            it.state = 'missed'; it.t = 0
            sim.setObjectParent(it.h, -1, true)
            sim.setObjectInt32Parameter(it.h, sim.shapeintparam_static, 0)
            sim.resetDynamicObject(it.h)
            stationItem = nil
        end
    end

    armStep(dt, estop)
    if not estop then gripperStep() end
    conveyorStep(dt, estop)
    if not estop then spawnerStep(dt) end

    steps = steps + 1
    sim.setIntegerSignal('hb', steps)
    sim.setFloatSignal('sim_time', sim.getSimulationTime())
    sim.setIntegerSignal('cnt_spawned', cntSpawned)
    sim.setIntegerSignal('cnt_missed', cntMissed)
end

function sysCall_sensing()
    stationLogic()
end

function sysCall_cleanup()
    -- items created during the run are removed automatically by CoppeliaSim when the simulation ends
end
