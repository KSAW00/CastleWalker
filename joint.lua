local cfg = require("config")

rednet.open(cfg.modem)
rednet.host("joint", cfg.hostname)

local gear = peripheral.wrap(cfg.gearshift)
local currentAngle = cfg.home or 0

local function round(x)
    if x >= 0 then
        return math.floor(x + 0.5)
    else
        return math.ceil(x - 0.5)
    end
end

local function execute(targetAngle)
    targetAngle = round(targetAngle)
    if targetAngle == currentAngle then
        return
    end
    local delta = targetAngle - currentAngle
    while gear.isRunning() do
        sleep(0.05)
    end

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

while true do
    local _, msg = rednet.receive("joint.command")
    execute(msg.angle)
end