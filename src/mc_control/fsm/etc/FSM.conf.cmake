{
  // If true, the FSM transitions are managed by an external tool
  "Managed": false,
  // If true and the FSM is self-managed, transitions should be triggered
  "StepByStep": true,
  // Change idle behaviour, if true the state is kept until transition,
  // otherwise the FSM holds the last state until transition
  "IdleKeepState": false,
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
          "stiffness": 5.0,
          "weight": 1000,
          "completion": { "OR": [ { "eval": 1e-3 },
                                  {"AND": [ { "timeout": 1.0 }, { "speed": 1e-2 } ] } ] }
        }
      }
    },
    "RelativeEndEffector":
    {
      "base": "CoM",
      "tasks":
      {
        "RelativeEndEffector":
        {
          "type": "relBody6d",
          "robotIndex": 0,
          "positionStiffness": 5.0,
          "positionWeight": 2000,
          "orientationStiffness": 100.0,
          "orientationWeight": 0,
          "completion": { "OR": [ { "eval": 1e-3 },
                                  {"AND": [ { "timeout": 1.0 }, { "speed": 1e-2 } ] } ] }
        },
        "Orientation":
        {
          "type": "orientation",
          "robotIndex": 0,
          "orientationStiffness": 100.0,
          "orientationWeight": 10000.0
        }
      }
    },
    "GoHalfSitting":
    {
      "base": "MiddleCoM",
      "tasks":
      {
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
    "GoHalfSitting2": { "base": "GoHalfSitting" },
    "LeftCoM":
    {
      "base": "CoM",
      "tasks":
      {
        "CoM":
        {
          "above": ["LFullSole"]
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
          "above": ["RFullSole"]
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
          "above": ["LFullSole", "RFullSole"]
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
    },
    "MoveLFullSole":
    {
      "base": "RelativeEndEffector",
      "tasks":
      {
        "RelativeEndEffector":
        {
          "body": "LLEG_LINK5",
          "relBody": "RLEG_LINK5",
          "position": [0.25, 0.19, 0.1]
        },
        "Orientation": { "body": "LLEG_LINK5" }
      }
    },
    "MoveRFullSole":
    {
      "base": "RelativeEndEffector",
      "tasks":
      {
        "RelativeEndEffector":
        {
          "body": "RLEG_LINK5",
          "relBody": "LLEG_LINK5",
          "position": [0.25, -0.19, 0.1]
        },
        "Orientation": { "body": "RLEG_LINK5" }
      }
    },
    "GoHalfSitting3": { "base": "GoHalfSitting2" },
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
            {"jointName": "HEAD_JOINT0", "stiffness": 5.0},
            {"jointName": "HEAD_JOINT1", "stiffness": 5.0}
          ],
          "target": { "HEAD_JOINT1": [-0.5] },
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
          "target": { "HEAD_JOINT1": [0.5] }
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
          "target": { "HEAD_JOINT1": [0.0] }
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
        ["Pause", "OK", "GoHalfSitting", "Strict"],
        ["GoHalfSitting", "OK", "LeftCoM"],
        ["GoHalfSitting2", "OK", "RightCoM"],
        ["LeftCoM", "OK", "RemoveRFullSole"],
        ["RemoveRFullSole", "OK", "MoveRFullSole"],
        ["MoveRFullSole", "OK", "AddRFullSole"],
        ["AddRFullSole", "OK", "GoHalfSitting2"],
        ["RightCoM", "OK", "RemoveLFullSole"],
        ["RemoveLFullSole", "OK", "MoveLFullSole"],
        ["MoveLFullSole", "OK", "AddLFullSole"],
        ["AddLFullSole", "OK", "GoHalfSitting3"],
        ["GoHalfSitting3", "OK", "Pause"]
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
    ["Pause", "OK", "FullFSM", "Strict" ]
  ],
  // Initial state
  "init": "Pause"
}
