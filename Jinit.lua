local filesToDownload = {
    { id = "bjTbXkSc", path = "startup.lua" }, -- Use 'startup.lua' to run automatically on boot
    { id = "H2CU9zfx", path = "config.lua" },
}

for _, file in ipairs(filesToDownload) do
    local url = "https://pastebin.com" .. file.id
    
    local response, err = http.get(url)
    if response then
        local contents = response.readAll()
        response.close()

        local f = fs.open(file.path, "w")
        f.write(contents)
        f.close()
    end
end


os.reboot() 