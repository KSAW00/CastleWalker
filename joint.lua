local cfg = require("config")

rednet.open(cfg.modem)
rednet.host("joint", cfg.hostname)

local gear = peripheral.wrap(cfg.gearshift)
local bearing = peripheral.wrap(cfg.bearing)

currentAngle=bearing.getTargetAngle()

local function round(x)
    if x >= 0 then
        return math.floor(x + 0.5)
    else
        return math.ceil(x - 0.5)
    end
end

local function shortestDelta(target, current)
    local d = (target - current + 180) % 360 - 180
    return d
end

local function execute(targetAngle)
    if targetAngle==nil then
        targetAngle=0
    end
    targetAngle = round(targetAngle)
    if targetAngle == currentAngle then
        return
    end
   
    while gear.isRunning() do
        sleep(0.05)
    end

    local delta = shortestDelta(targetAngle, currentAngle)

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
    currentAngle = targetAngle
end

execute(cfg.home)
while true do
    local _, msg = rednet.receive("joint.command")
    execute(msg.angle)
end