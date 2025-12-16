-- filepath: /home/vscode/workspace/devel/mc_rtc/.nvim.lua
-- Project-specific Neovim configuration

local schema_dir = "doc/_data/schemas/"
-- Get the absolute path to the current file (.nvim.lua)
local info = debug.getinfo(1, "S")
local file_path = vim.fn.fnamemodify(info.source:sub(2), ":p") -- Remove '@', get absolute path
local file_dir = vim.fn.fnamemodify(file_path, ":h")
print(file_dir)

local function schema_path(rel)
  local schema_dir = "file://" .. file_dir .. "/" .. schema_dir .. rel
  print(schema_dir)
  return schema_dir
end

vim.lsp.config("yamlls", {
  settings = {
    yaml = {
      schemas = {
        [schema_path("mc_rtc/mc_rtc.json")] = {
          "**/mc_rtc.yaml",
          "**/mc_rtc.in.yaml",
        },
        [schema_path("mc_control/FSMStates.json")] = {
          "src/mc_control/fsm/states/data/*.yaml",
        },
        [schema_path("mc_control/FSMController.json")] = {
          "src/mc_control/samples/Admittance/etc/AdmittanceSample.in.yaml",
          "src/mc_control/samples/Door/etc/DoorSample.in.yaml",
          "src/mc_control/samples/ExternalForces/etc/ExternalForces.in.yaml",
          "src/mc_control/samples/LIPMStabilizer/etc/LIPMStabilizer.in.yaml",
          "tests/controllers/TestFSMMetaContinuity.in.yaml",
          "tests/controllers/TestFSMStateOptions.in.yaml",
          "tests/controllers/TestPythonState.in.yaml",
          "tests/controllers/TestObserverConfigurationController.in.yaml",
        },
        validate = true,
        format = { enable = false },
        hover = true,
        completion = true,
      },
    },
  },
})
vim.cmd("LspRestart")
