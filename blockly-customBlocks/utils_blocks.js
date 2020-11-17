Blockly.defineBlocksWithJsonArray([{
  "type": "utils_position_in_pose",
  "message0": "Position in pose %1",
  "args0": [
    {
      "type": "input_value",
      "name": "POSE",
      "check": "pose",
      "align": "RIGHT"
    }
  ],
  "inputsInline": false,
  "output": "position",
  "colour": 200,
  "tooltip": "Extracts the position vector (x,y,z) from given pose(x,y,z,theta1,theta2,theta3))",
  "helpUrl": ""
},
{
  "type": "utils_orientation_in_pose",
  "message0": "Orientation in pose %1",
  "args0": [
    {
      "type": "input_value",
      "name": "POSE",
      "check": "pose",
      "align": "RIGHT"
    }
  ],
  "inputsInline": false,
  "output": "orientation",
  "colour": 200,
  "tooltip": "Extracts the orientation vector (theta1,theta2,theta3) from given pose(x,y,z,theta1,theta2,theta3)",
  "helpUrl": ""
},
{
  "type": "utils_pose_from_position_n_orientation",
  "message0": "Pose from Position: %1 and Orientation %2",
  "args0": [
    {
      "type": "input_value",
      "name": "POSITION",
      "check": "position",
      "align": "RIGHT"
    },
    {
      "type": "input_value",
      "name": "ORIENTATION",
      "check": "orientation",
      "align": "RIGHT"
    }
  ],
  "inputsInline": true,
  "output": "pose",
  "colour": 200,
  "tooltip": "Combines Position vector(x,y,z) with Orientation vector(theta1,theta2,theta3) as a pose vector (x,y,z,theta1,theta2,theta3)",
  "helpUrl": ""
},
{
  "type": "utils_position",
  "message0": "x %1 y %2 z %3 (m)",
  "args0": [
    {
      "type": "input_value",
      "name": "X",
      "check": "Number"
    },
    {
      "type": "input_value",
      "name": "Y",
      "check": "Number"
    },
    {
      "type": "input_value",
      "name": "Z",
      "check": "Number"
    }
  ],
  "inputsInline": true,
  "output": "position",
  "colour": 200,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "utils_orientation",
  "message0": "θx %1 θy %2 θz %3 (deg)",
  "args0": [
    {
      "type": "input_value",
      "name": "TX",
      "check": "Number"
    },
    {
      "type": "input_value",
      "name": "TY",
      "check": "Number"
    },
    {
      "type": "input_value",
      "name": "TZ",
      "check": "Number"
    }
  ],
  "inputsInline": true,
  "output": "orientation",
  "colour": 200,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "utils_arithmetic",
  "message0": "%1 %2 %3",
  "args0": [
    {
      "type": "input_value",
      "name": "A",
      "check": [
        "position",
        "orientation"
      ]
    },
    {
      "type": "field_dropdown",
      "name": "NAME",
      "options": [
        [
          "+",
          "ADD"
        ],
        [
          "-",
          "MINUS"
        ],
        [
          "×",
          "MULTIPLY"
        ],
        [
          "÷",
          "DIVIDE"
        ]
      ]
    },
    {
      "type": "input_value",
      "name": "B",
      "check": [
        "position",
        "orientation"
      ]
    }
  ],
  "inputsInline": true,
  "output": [
    "position",
    "orientation"
  ],
  "colour": 200,
  "tooltip": "Arithmetic operations for 2 vectors of Position or Orientation",
  "helpUrl": ""
}]);

Blockly.Python['utils_position_in_pose'] = function(block) {
  var value_pose = Blockly.Python.valueToCode(block, 'POSE', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = '...';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['utils_orientation_in_pose'] = function(block) {
  var value_pose = Blockly.Python.valueToCode(block, 'POSE', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = '...';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['utils_pose_from_position_n_orientation'] = function(block) {
  var value_position = Blockly.Python.valueToCode(block, 'POSITION', Blockly.Python.ORDER_ATOMIC);
  var value_orientation = Blockly.Python.valueToCode(block, 'ORIENTATION', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = '...';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['utils_position'] = function(block) {
  var value_x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_ATOMIC);
  var value_y = Blockly.Python.valueToCode(block, 'Y', Blockly.Python.ORDER_ATOMIC);
  var value_z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = '[' + value_x + ',' + value_y + ',' + value_z + ']';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['utils_orientation'] = function(block) {
  var value_tx = Blockly.Python.valueToCode(block, 'TX', Blockly.Python.ORDER_ATOMIC);
  var value_ty = Blockly.Python.valueToCode(block, 'TY', Blockly.Python.ORDER_ATOMIC);
  var value_tz = Blockly.Python.valueToCode(block, 'TZ', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = '[' + value_tx + ',' + value_ty + ',' + value_tz + ']';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['utils_arithmetic'] = function(block) {
  var value_a = Blockly.Python.valueToCode(block, 'A', Blockly.Python.ORDER_ATOMIC);
  var dropdown_name = block.getFieldValue('NAME');
  var value_b = Blockly.Python.valueToCode(block, 'B', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = '...';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};