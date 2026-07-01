local cfg = require("config")

rednet.open(cfg.modem)
rednet.host("joint", cfg.hostname)

local gear = peripheral.wrap(cfg.gearshift)
local bearing = peripheral.wrap(cfg.bearing)

-- Keep track of where we physically are
local currentAngle = bearing.getTargetAngle()

local function round(x)
    if x >= 0 then return math.floor(x + 0.5)
    else return math.ceil(x - 0.5) end
end

local function shortestDelta(target, current)
    return (target - current + 180) % 360 - 180
end

local function execute(targetAngle)
    if targetAngle == nil then targetAngle = 0 end
    targetAngle = round(targetAngle)
    
    -- ALWAYS read the true physical angle from the bearing to prevent software drift
    currentAngle = bearing.getTargetAngle()

    if targetAngle == currentAngle then
        return
    end
   
    -- Wait for any active movements to completely finish before pushing a new sequence
    while gear.isRunning() do
        sleep(0.05)
    end

    -- Re-read angle just in case it changed while waiting
    currentAngle = bearing.getTargetAngle()
    local delta = shortestDelta(targetAngle, currentAngle)

    -- If the delta is tiny (rounding noise), skip it to prevent jitter
    if math.abs(delta) < 1 then return end

    gear.setInstructions({
        {
            type = "turn_angle",
            value = math.abs(delta),
            speed_modifier = delta >= 0 and 1 or -1
        },
        {
            type = "end"
        }
    })
    
    gear.start()
    
    -- FIX: Wait for the physical movement to finish BEFORE updating state or exiting
    while gear.isRunning() do
        sleep(0.05)
    end
end

-- Initialize at home position
execute(cfg.home)

-- Main Listener Loop
while true do
    local _, msg = rednet.receive("joint.command")
    if msg and msg.angle then
        execute(msg.angle)
    end
end