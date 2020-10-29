Blockly.defineBlocksWithJsonArray([{
  "type": "robot_jog_cartesian_not_relative",
  "lastDummyAlign0": "RIGHT",
  "message0": "Jog Cartesian to: %1 Position %2 Orientation %3 with  %4 %% speed",
  "args0": [
    {
      "type": "input_dummy",
      "align": "RIGHT"
    },
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
    },
    {
      "type": "field_number",
      "name": "SPEED",
      "value": 100,
      "min": 0,
      "max": 100,
      "precision": 1
    }
  ],
  "inputsInline": true,
  "previousStatement": null,
  "nextStatement": null,
  "colour": 340,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "robot_jog_cartesian_relative",
  "lastDummyAlign0": "RIGHT",
  "message0": "Jog Cartesian Relatively: %1 Position %2 Orientation %3 with  %4 %% speed",
  "args0": [
    {
      "type": "input_dummy",
      "align": "RIGHT"
    },
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
    },
    {
      "type": "field_number",
      "name": "SPEED",
      "value": 100,
      "min": 0,
      "max": 100,
      "precision": 1
    }
  ],
  "inputsInline": true,
  "previousStatement": null,
  "nextStatement": null,
  "colour": 340,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "robot_jog_cartesian_position",
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
  "type": "robot_jog_cartesian_orientation",
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
  "type": "robot_wait",
  "message0": "Wait %1 seconds",
  "args0": [
    {
      "type": "field_number",
      "name": "WAIT_TIME",
      "value": 1,
      "min": 0
    }
  ],
  "previousStatement": null,
  "nextStatement": null,
  "colour": 120,
  "tooltip": "",
  "helpUrl": ""
}]);

Blockly.Python['robot_jog_cartesian_position'] = function(block) {
  var value_x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_ATOMIC);
  var value_y = Blockly.Python.valueToCode(block, 'Y', Blockly.Python.ORDER_ATOMIC);
  var value_z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = '[' + value_x + ',' + value_y + ',' + value_z + ']';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['robot_jog_cartesian_orientation'] = function(block) {
  var value_tx = Blockly.Python.valueToCode(block, 'TX', Blockly.Python.ORDER_ATOMIC);
  var value_ty = Blockly.Python.valueToCode(block, 'TY', Blockly.Python.ORDER_ATOMIC);
  var value_tz = Blockly.Python.valueToCode(block, 'TZ', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = '[' + value_tx + ',' + value_ty + ',' + value_tz + ']';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};


Blockly.Python['robot_jog_cartesian_not_relative'] = function(block) {
  var value_position = Blockly.Python.valueToCode(block, 'POSITION', Blockly.Python.ORDER_ATOMIC);
  var value_orientation = Blockly.Python.valueToCode(block, 'ORIENTATION', Blockly.Python.ORDER_ATOMIC);
  var number_speed = block.getFieldValue('SPEED');
  // TODO: Assemble Python into code variable.
  var code = 'self.jog_cartesian('+ value_position + ',' + value_orientation +','+ number_speed +')\n';
  return code;
};

Blockly.Python['robot_jog_cartesian_relative'] = function(block) {
  var value_position = Blockly.Python.valueToCode(block, 'POSITION', Blockly.Python.ORDER_ATOMIC);
  var value_orientation = Blockly.Python.valueToCode(block, 'ORIENTATION', Blockly.Python.ORDER_ATOMIC);
  var number_speed = block.getFieldValue('SPEED');
  // TODO: Assemble Python into code variable.
  var code = 'self.jog_cartesian_relative('+ value_position + ',' + value_orientation +','+ number_speed +')\n';
  return code;
};

Blockly.Python['robot_wait'] = function(block) {
  var number_wait_time = block.getFieldValue('WAIT_TIME');
  // TODO: Assemble Python into code variable.
  var code = 'time.sleep(' + number_wait_time+ ')\n';
  return code;
};
// =====================

