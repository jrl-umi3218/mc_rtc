{
  // If true, the FSM transitions are managed by an external tool
  "Managed": false,
  // If true and the FSM is self-managed, transitions should be triggered
  "StepByStep": false,
  // Where to look for state libraries
  "StatesLibraries": ["@MC_CONTROLLER_INSTALL_PREFIX@/fsm/states"],
  // Where to look for state files
  "StatesFiles": ["@MC_CONTROLLER_INSTALL_PREFIX@/fsm/states/data"],
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
      "stiffness": 1.0,
      "weight": 10.0
    },
    "com":
    {
      "stiffness": 2.0,
      "weight": 100.0
    }
  },
  // Initial state
  "init": "Pause",
  // Test some states
  "states":
  {
    "CoM":
    {
      "base": "MetaTasks",
      "tasks":
      {
        "CoM":
        {
          "type": "com",
          "robotIndex": 0,
          "stiffness": 5.0,
          "weight": 1000,
          "completion": { "OR": [ { "eval": 1e-3 },
                                  {"AND": [ { "timeout": 1.0 }, { "speed": 1e-3 } ] } ] }
        }
      }
    },
    "HalfSitting":
    {
      "base": "CoM",
      "tasks":
      {
        "CoM": { "com": [0, 0, 0.87] },
        "Posture":
        {
          "type": "posture",
          "robotIndex": 0,
          "stiffness": 2.0,
          "weight": 100,
          "posture": [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.773], [0.0], [0.0], [-0.4537856055185257], [0.8726646259971648], [-0.41887902047863906], [0.0], [0.0], [0.0], [-0.4537856055185257], [0.8726646259971648], [-0.41887902047863906], [0.0], [0.0], [0.0], [0.0], [0.0], [0.7853981633974483], [-0.3490658503988659], [0.0], [-1.3089969389957472], [0.0], [0.0], [0.0], [0.3490658503988659], [-0.3490658503988659], [0.3490658503988659], [-0.3490658503988659], [0.3490658503988659], [-0.3490658503988659], [0.7853981633974483], [0.3490658503988659], [0.0], [-1.3089969389957472], [0.0], [0.0], [0.0], [0.3490658503988659], [-0.3490658503988659], [0.3490658503988659], [-0.3490658503988659], [0.3490658503988659], [-0.3490658503988659], []]
        }
      }
    },
    "HalfSitting2": { "base": "HalfSitting" },
    "LeftCoM":
    {
      "base": "CoM",
      "tasks":
      {
        "CoM":
        {
          "move_com": [0, 0.1, 0]
        }
      }
    },
    "RightCoM":
    {
      "base": "CoM",
      "tasks":
      {
        "CoM":
        {
          "move_com": [0, -0.1, 0]
        }
      }
    },
    "RemoveLFullSole":
    {
      "base": "RemoveContact",
      "contact":
      {
        "r1Surface": "LFullSole",
        "r2Surface": "AllGround",
        "isFixed": false
      }
    },
    "AddLFullSole":
    {
      "base": "AddContact",
      "contact":
      {
        "r1Surface": "LFullSole",
        "r2Surface": "AllGround",
        "isFixed": false
      }
    },
    "RemoveRFullSole":
    {
      "base": "RemoveContact",
      "contact":
      {
        "r1Surface": "RFullSole",
        "r2Surface": "AllGround",
        "isFixed": false
      }
    },
    "AddRFullSole":
    {
      "base": "AddContact",
      "contact":
      {
        "r1Surface": "RFullSole",
        "r2Surface": "AllGround",
        "isFixed": false
      }
    }
  },
  // Transitions map
  "transitions":
  [
    ["Pause", "OK", "HalfSitting", "Strict" ],
    ["HalfSitting", "OK", "LeftCoM"],
    ["HalfSitting2", "OK", "RightCoM"],
    ["LeftCoM", "OK", "RemoveRFullSole"],
    ["RemoveRFullSole", "OK", "AddRFullSole"],
    ["AddRFullSole", "OK", "HalfSitting2"],
    ["RightCoM", "OK", "RemoveLFullSole"],
    ["RemoveLFullSole", "OK", "AddLFullSole"],
    ["AddLFullSole", "OK", "HalfSitting"]
  ]
}
