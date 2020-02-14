{
  // If true, the FSM transitions are managed by an external tool
  "Managed": false,
  // If true and the FSM is self-managed, transitions should be triggered
  "StepByStep": false,
  // Change idle behaviour, if true the state is kept until transition,
  // otherwise the FSM holds the last state until transition
  "IdleKeepState": false,
  // Where to look for state libraries
  "StatesLibraries": ["@FSM_STATES_INSTALL_PREFIX@"],
  // Where to look for state files
  "StatesFiles": ["@FSM_STATES_DATA_INSTALL_PREFIX@"],
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
    },
    {
      "type": "compoundJoint",
      "robotIndex": 0
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
      "r1": "jvrc1",
      "r2": "ground",
      "r1Surface": "LeftFoot",
      "r2Surface": "AllGround"
    },
    {
      "r1": "jvrc1",
      "r2": "ground",
      "r1Surface": "RightFoot",
      "r2Surface": "AllGround"
    }
  ],
  // Some options for a specific robot
  "jvrc1":
  {
    "posture":
    {
      "stiffness": 1.0,
      "weight": 10.0
    },
    "ff":
    {
      "stiffness": 2.0,
      "weight": 100.0
    }
  },
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
          "stiffness": 10.0,
          "weight": 1000,
          "completion": { "OR": [ { "eval": 1e-3 },
                                  {"AND": [ { "timeout": 3.0 }, { "speed": 1e-2 } ] } ] }
        }
      }
    },
    "MoveFoot":
    {
      "base": "MetaTasks",
      "tasks":
      {
        "MoveFoot":
        {
          "type": "surfaceTransform",
          "robotIndex": 0,
          "stiffness": 5.0,
          "weight": 500,
          "completion": { "OR": [ { "eval": 1e-3 },
                                  {"AND": [ { "timeout": 5.0 }, { "speed": 1e-2 } ] } ] }
        }
      }
    },
    "GoHalfSitting":
    {
      "base": "Parallel",
      "states": ["HalfSitting", "MiddleCoM"],
      "configs":
      {
        "HalfSitting":
        {
          "stiffness": 10,
          "completion": 0.1
        }
      }
    },
    "PauseHalfSitting":
    {
      "base": "GoHalfSitting",
      "states": ["HalfSitting", "MiddleCoM", "Pause"],
      "configs":
      {
        "Pause":
        {
          "duration": 3,
          "completion": 0.5
        }
      }
    },
    "LeftCoM":
    {
      "base": "CoM",
      "tasks":
      {
        "CoM":
        {
          "above": ["LeftFoot"]
        }
      }
    },
    "RightCoM":
    {
      "base": "LeftCoM",
      "tasks":
      {
        "CoM":
        {
          "above": ["RightFoot"]
        }
      }
    },
    "MiddleCoM":
    {
      "base": "CoM",
      "tasks":
      {
        "CoM":
        {
          "above": ["LeftFoot", "RightFoot"]
        }
      }
    },
    "AddLeftFoot":
    {
      "base": "AddContact",
      "useCoM": false,
      "contact":
      {
        "r1Surface": "LeftFoot",
        "r2Surface": "AllGround",
        "isFixed": false
      }
    },
    "AddLeftFootCoM":
    {
      "base": "Parallel",
      "states": ["AddLeftFoot", "RightCoM"]
    },
    "AddRightFoot":
    {
      "base": "AddContact",
      "useCoM": false,
      "contact":
      {
        "r1Surface": "RightFoot",
        "r2Surface": "AllGround",
        "isFixed": false
      }
    },
    "AddRightFootCoM":
    {
      "base": "Parallel",
      "states": ["AddRightFoot", "LeftCoM"]
    },
    "MoveLeftFoot":
    {
      "base": "MoveFoot",
      "tasks":
      {
        "MoveFoot":
        {
          "surface": "LeftFoot",
          "moveWorld":
          {
            "translation": [0.0, 0.0, 0.1]
          }
        }
      },
      "RemoveContacts":
      [
        {
          "r1": "jvrc1",
          "r2": "ground",
          "r1Surface": "LeftFoot",
          "r2Surface": "AllGround",
          "isFixed": false
        }
      ]
    },
    "MoveLeftFootCoM":
    {
      "base": "Parallel",
      "states": ["MoveLeftFoot", "RightCoM", "HalfSitting"]
    },
    "MoveRightFoot":
    {
      "base": "MoveFoot",
      "tasks":
      {
        "MoveFoot":
        {
          "surface": "RightFoot",
          "moveWorld":
          {
            "translation": [0.0, 0.0, 0.1]
          }
        }
      },
      "RemoveContacts":
      [
        {
          "r1": "jvrc1",
          "r2": "ground",
          "r1Surface": "RightFoot",
          "r2Surface": "AllGround"
        }
      ]
    },
    "MoveRightFootCoM":
    {
      "base": "Parallel",
      "states": ["MoveRightFoot", "LeftCoM", "HalfSitting"]
    },
    "HeadUp":
    {
      "base": "MetaTasks",
      "tasks":
      {
        "HeadPosture":
        {
          "type": "posture",
          "robotIndex": 0,
          "stiffness": 0.0,
          "weight": 50,
          "jointGains": [
            {"jointName": "NECK_Y", "stiffness": 5.0},
            {"jointName": "NECK_P", "stiffness": 5.0}
          ],
          "target": { "NECK_P": [-0.5] },
          "completion": { "OR": [ { "eval": 1e-3 },
                                  {"AND": [ { "timeout": 1.0 }, { "speed": 5e-3 } ] } ] }
        }
      }
    },
    "HeadDown":
    {
      "base": "HeadUp",
      "tasks":
      {
        "HeadPosture":
        {
          "target": { "NECK_P": [0.5] }
        }
      }
    },
    "HeadZero":
    {
      "base": "HeadUp",
      "tasks":
      {
        "HeadPosture":
        {
          "target": { "NECK_P": [0.0] }
        }
      }
    },
    "WalkTwoSteps":
    {
      "base": "Meta",
      "Managed": false,
      "StepByStep": false,
      "transitions":
      [
        ["PauseHalfSitting", "OK", "LeftCoM"],
        ["LeftCoM", "OK", "MoveRightFootCoM"],
        ["MoveRightFootCoM", "OK", "AddRightFootCoM"],
        ["AddRightFootCoM", "OK", "RightCoM"],
        ["RightCoM", "OK", "MoveLeftFootCoM"],
        ["MoveLeftFootCoM", "OK", "AddLeftFootCoM"],
        ["AddLeftFootCoM", "OK", "PauseHalfSitting"]
      ]
    },
    "HeadFSM":
    {
      "base": "Meta",
      "Managed": false,
      "StepByStep": false,
      "transitions":
      [
        ["HeadDown", "OK", "HeadUp"],
        ["HeadUp", "OK", "HeadDown"]
      ]
    },
    "FullFSM":
    {
      "base": "Parallel",
      "states": ["HeadFSM", "WalkTwoSteps"]
    }
  },
  // Transitions map
  "transitions":
  [
    ["FullFSM", "OK", "FullFSM", "Strict" ]
  ],
  // Initial state
  "init": "FullFSM"
}
