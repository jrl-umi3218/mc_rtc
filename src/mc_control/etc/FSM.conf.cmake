{
  // If true, the FSM transitions are managed by an external tool
  "Managed": false,
  // If true and the FSM is self-managed, transitions should be triggered
  "StepByStep": false,
  // Where to look for state libraries
  "StatesLibraries": ["@MC_CONTROLLER_INSTALL_PREFIX@/fsm_states"],
  // Where to look for state files
  "StatesFiles": ["@MC_CONTROLLER_INSTALL_PREFIX@/fsm_states/data"],
  // If true, state factory will be more verbose
  "VerboseStateFactory": false,
  // Additional robots to load
  "robots":
  {
    "ground":
    {
      "module": "env",
      "params": ["@AROBASE@MC_ENV_DESCRIPTION@AROBASE@", "ground"]
    }
  },
  // General constraints, always on
  "constraints":
  [
    {
      "type": "contact"
    },
    {
      "type": "dynamics",
      "robotIndex": 0,
      "damper": [0.1, 0.01, 0.5]
    }
  ],
  // Collision constraint
  "collisions":
  [
    {
      "type": "collision",
      "r1Index": 0,
      "r2Index": 0,
      "useMinimal": true
    }
  ],
  // Initial set of contacts
  "contacts":
  [
    {
      "r1": "hrp2_drc",
      "r2": "ground",
      "r1Surface": "LFullSole",
      "r2Surface": "AllGround"
    },
    {
      "r1": "hrp2_drc",
      "r2": "ground",
      "r1Surface": "RFullSole",
      "r2Surface": "AllGround"
    }
  ],
  // Some options for a specific robot
  "hrp2_drc":
  {
    "posture":
    {
      "stiffness": 2.0,
      "weight": 2.0
    }
  },
  // Initial state
  "init": "Pause",
  // Transitions map
  "transitions":
  [
    ["Pause", "OK", "Pause_1s", "Strict"],
    ["Pause_1s", "OK", "Pause_2s", "Auto"],
    ["Pause_2s", "OK", "Pause", "StepByStep"]
  ]
}
