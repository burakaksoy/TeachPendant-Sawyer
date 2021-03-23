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
  "type": "utils_sum_vectors",
  "message0": "T1: %1 + %2 T2: %3",
  "args0": [
    {
      "type": "input_value",
      "name": "position_1",
      "check": "position"
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "input_value",
      "name": "position_2",
      "check": "position"
    }
  ],
  "inputsInline": true,
  "output": "position",
  "colour": 205,
  "tooltip": "Sum for 2 vectors of Position",
  "helpUrl": ""
},
{
  "type": "utils_subtract_vectors",
  "message0": "T1: %1 - %2 T2: %3",
  "args0": [
    {
      "type": "input_value",
      "name": "position_1",
      "check": "position"
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "input_value",
      "name": "position_2",
      "check": "position"
    }
  ],
  "inputsInline": true,
  "output": "position",
  "colour": 205,
  "tooltip": "Subtract for 2 vectors of Position",
  "helpUrl": ""
},
{
  "type": "utils_rotate_frame",
  "message0": "R1: %1 × %2 R2: %3",
  "args0": [
    {
      "type": "input_value",
      "name": "orientation_1",
      "check": "orientation"
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "input_value",
      "name": "orientation_2",
      "check": "orientation"
    }
  ],
  "inputsInline": true,
  "output": "orientation",
  "colour": 195,
  "tooltip": "Combined Rotation for 2 Rotations",
  "helpUrl": ""
},
{
  "type": "utils_rotate_vector",
  "message0": "R: %1 × %2 T: %3",
  "args0": [
    {
      "type": "input_value",
      "name": "orientation",
      "check": "orientation"
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "input_value",
      "name": "position",
      "check": "position"
    }
  ],
  "inputsInline": true,
  "output": "position",
  "colour": 205,
  "tooltip": "Rotate a vector",
  "helpUrl": ""
},
{
  "type": "utils_inverse_frame",
  "message0": "(R: %1 )⁻¹",
  "args0": [
    {
      "type": "input_value",
      "name": "orientation",
      "check": "orientation"
    }
  ],
  "inputsInline": true,
  "output": "orientation",
  "colour": 205,
  "tooltip": "Inverse a Rotation",
  "helpUrl": ""
},
{
  "type": "utils_minus_vector",
  "message0": "- (T: %1 )",
  "args0": [
    {
      "type": "input_value",
      "name": "position",
      "check": "position"
    }
  ],
  "inputsInline": true,
  "output": "position",
  "colour": 205,
  "tooltip": "Minus vector",
  "helpUrl": ""
}]);

Blockly.Python['utils_position_in_pose'] = function(block) {
  var value_pose = Blockly.Python.valueToCode(block, 'POSE', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'self.utils_position_in_pose('+  value_pose + ')';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['utils_orientation_in_pose'] = function(block) {
  var value_pose = Blockly.Python.valueToCode(block, 'POSE', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'self.utils_orientation_in_pose('+  value_pose + ')';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['utils_pose_from_position_n_orientation'] = function(block) {
  var value_position = Blockly.Python.valueToCode(block, 'POSITION', Blockly.Python.ORDER_ATOMIC);
  var value_orientation = Blockly.Python.valueToCode(block, 'ORIENTATION', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'self.utils_pose_from_position_n_orientation(' + value_position + ',' + value_orientation + ')';
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

Blockly.Python['utils_sum_vectors'] = function(block) {
  var value_position_1 = Blockly.Python.valueToCode(block, 'position_1', Blockly.Python.ORDER_ATOMIC);
  var value_position_2 = Blockly.Python.valueToCode(block, 'position_2', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'self.utils_sum_vectors(' + value_position_1 + ',' + value_position_2 + ')';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['utils_subtract_vectors'] = function(block) {
  var value_position_1 = Blockly.Python.valueToCode(block, 'position_1', Blockly.Python.ORDER_ATOMIC);
  var value_position_2 = Blockly.Python.valueToCode(block, 'position_2', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'self.utils_subtract_vectors(' + value_position_1 + ',' + value_position_2 + ')';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['utils_rotate_frame'] = function(block) {
  var value_orientation_1 = Blockly.Python.valueToCode(block, 'orientation_1', Blockly.Python.ORDER_ATOMIC);
  var value_orientation_2 = Blockly.Python.valueToCode(block, 'orientation_2', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'self.utils_rotate_frame(' + value_orientation_1 + ',' + value_orientation_2 + ')';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['utils_rotate_vector'] = function(block) {
  var value_orientation = Blockly.Python.valueToCode(block, 'orientation', Blockly.Python.ORDER_ATOMIC);
  var value_position = Blockly.Python.valueToCode(block, 'position', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'self.utils_rotate_vector(' + value_orientation + ',' + value_position + ')';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['utils_inverse_frame'] = function(block) {
  var value_orientation = Blockly.Python.valueToCode(block, 'orientation', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'self.utils_inverse_frame(' + value_orientation + ')';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['utils_minus_vector'] = function(block) {
  var value_position = Blockly.Python.valueToCode(block, 'position', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'self.utils_minus_vector(' + value_position + ')';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};