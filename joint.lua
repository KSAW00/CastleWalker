local cfg = require("config")

rednet.open(cfg.modem)
rednet.host("joint", cfg.hostname)

local gear = peripheral.wrap(cfg.gearshift)


local function execute(delta)

    if math.abs(delta) < 0.1 then
        return
    end

    -- Finish any current motion first
    while gear.isRunning() do
        sleep(0)
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

end


while true do

    local _, msg = rednet.receive("joint.command")

    execute(msg.delta)

end