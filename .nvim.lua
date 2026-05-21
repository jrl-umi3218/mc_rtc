-- filepath: /home/vscode/workspace/devel/mc_rtc/.nvim.lua
-- Project-specific Neovim configuration

local schema_dir = "doc/_data/schemas/"
-- Get the absolute path to the current file (.nvim.lua)
local info = debug.getinfo(1, "S")
local file_path = vim.fn.fnamemodify(info.source:sub(2), ":p") -- Remove '@', get absolute path
local file_dir = vim.fn.fnamemodify(file_path, ":h")
print(file_dir)

local function schema_path(rel)
  local rel_schema_dir = "file://" .. file_dir .. "/" .. schema_dir .. rel
  -- print(rel_schema_dir)
  return rel_schema_dir
end

local raw_schemas = {
  ["mc_rtc/mc_rtc.json"] = {
    "**/mc_rtc.yaml",
    "**/mc_rtc.in.yaml",
  },
  ["mc_control/FSMStates.json"] = {
    "src/mc_control/fsm/states/data/*.yaml",
  },
  ["mc_control/FSMController.json"] = {
    "src/mc_control/samples/Admittance/etc/AdmittanceSample.in.yaml",
    "src/mc_control/samples/Door/etc/DoorSample.in.yaml",
    "src/mc_control/samples/ExternalForces/etc/ExternalForces.in.yaml",
    "src/mc_control/samples/LIPMStabilizer/etc/LIPMStabilizer.in.yaml",
    "tests/controllers/TestFSMMetaContinuity.in.yaml",
    "tests/controllers/TestFSMStateOptions.in.yaml",
    "tests/controllers/TestPythonState.in.yaml",
    "tests/controllers/TestObserverConfigurationController.in.yaml",
  },
}

-- Transform schema paths to absolute paths to local schemas
local schemas = {}
for schema, patterns in pairs(raw_schemas) do
  schemas[schema_path(schema)] = patterns
end

vim.lsp.config("yamlls", {
  settings = {
    yaml = {
      schemas = schemas,
      validate = true,
      format = { enable = false },
      hover = true,
      completion = true,
    },
  },
})

if vim.fn.exists(':LspRestart') == 2 then
  vim.cmd("LspRestart")
end
