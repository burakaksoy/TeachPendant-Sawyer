Blockly.defineBlocksWithJsonArray([{
  "type": "robot_jog_joint_not_relative",
  "message0": "Jog Joint %1 to degree %2 %3 with %4 %% speed",
  "args0": [
    {
      "type": "field_dropdown",
      "name": "JOINT_SELECTED",
      "options": [
        [
          "1",
          "1"
        ],
        [
          "2",
          "2"
        ],
        [
          "3",
          "3"
        ],
        [
          "4",
          "4"
        ],
        [
          "5",
          "5"
        ],
        [
          "6",
          "6"
        ],
        [
          "7",
          "7"
        ]
      ]
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "input_value",
      "name": "DEGREE",
      "check": "Number"
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
  "colour": 0,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "robot_jog_joint_relative",
  "message0": "Jog Joint %1 relatively by degree %2 %3 with %4 %% speed",
  "args0": [
    {
      "type": "field_dropdown",
      "name": "JOINT_SELECTED",
      "options": [
        [
          "1",
          "1"
        ],
        [
          "2",
          "2"
        ],
        [
          "3",
          "3"
        ],
        [
          "4",
          "4"
        ],
        [
          "5",
          "4"
        ],
        [
          "6",
          "6"
        ],
        [
          "7",
          "7"
        ]
      ]
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "input_value",
      "name": "DEGREE",
      "check": "Number"
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
  "colour": 0,
  "tooltip": "",
  "helpUrl": ""
}]);

Blockly.Python['robot_jog_joint_not_relative'] = function(block) {
  var dropdown_joint_selected = block.getFieldValue('JOINT_SELECTED');
  var value_degree = Blockly.Python.valueToCode(block, 'DEGREE', Blockly.Python.ORDER_ATOMIC);
  var number_speed = block.getFieldValue('SPEED');
  // TODO: Assemble Python into code variable.
  var code = 'self.jog_joint(' + dropdown_joint_selected + ', ' + value_degree + ', ' + number_speed + ')\n';
  return code;
};

Blockly.Python['robot_jog_joint_relative'] = function(block) {
  var dropdown_joint_selected = block.getFieldValue('JOINT_SELECTED');
  var value_degree = Blockly.Python.valueToCode(block, 'DEGREE', Blockly.Python.ORDER_ATOMIC);
  var number_speed = block.getFieldValue('SPEED');
  // TODO: Assemble Python into code variable.
  var code = 'self.jog_joint_relative(' + dropdown_joint_selected + ', ' + value_degree + ', ' + number_speed + ')\n';
  return code;
};

Blockly.defineBlocksWithJsonArray([{
  "type": "robot_jog_joints_not_relative",
  "lastDummyAlign0": "RIGHT",
  "message0": "Jog Joints to %1 1 %2 2 %3 3 %4 4 %5 5 %6 6 %7 7 %8 degrees with  %9 %% speed",
  "args0": [
    {
      "type": "input_dummy",
      "align": "CENTRE"
    },
    {
      "type": "input_value",
      "name": "DEGREE_1",
      "check": "Number",
      "align": "RIGHT"
    },
    {
      "type": "input_value",
      "name": "DEGREE_2",
      "check": "Number",
      "align": "RIGHT"
    },
    {
      "type": "input_value",
      "name": "DEGREE_3",
      "check": "Number",
      "align": "RIGHT"
    },
    {
      "type": "input_value",
      "name": "DEGREE_4",
      "check": "Number",
      "align": "RIGHT"
    },
    {
      "type": "input_value",
      "name": "DEGREE_5",
      "check": "Number",
      "align": "RIGHT"
    },
    {
      "type": "input_value",
      "name": "DEGREE_6",
      "check": "Number",
      "align": "RIGHT"
    },
    {
      "type": "input_value",
      "name": "DEGREE_7",
      "check": "Number",
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
  "colour": 0,
  "tooltip": "",
  "helpUrl": ""
},
{
  "type": "robot_jog_joints_relative",
  "lastDummyAlign0": "RIGHT",
  "message0": "Jog Joints relatively %1 1 %2 2 %3 3 %4 4 %5 5 %6 6 %7 7 %8 degrees with  %9 %% speed",
  "args0": [
    {
      "type": "input_dummy",
      "align": "CENTRE"
    },
    {
      "type": "input_value",
      "name": "DEGREE_1",
      "check": "Number",
      "align": "RIGHT"
    },
    {
      "type": "input_value",
      "name": "DEGREE_2",
      "check": "Number",
      "align": "RIGHT"
    },
    {
      "type": "input_value",
      "name": "DEGREE_3",
      "check": "Number",
      "align": "RIGHT"
    },
    {
      "type": "input_value",
      "name": "DEGREE_4",
      "check": "Number",
      "align": "RIGHT"
    },
    {
      "type": "input_value",
      "name": "DEGREE_5",
      "check": "Number",
      "align": "RIGHT"
    },
    {
      "type": "input_value",
      "name": "DEGREE_6",
      "check": "Number",
      "align": "RIGHT"
    },
    {
      "type": "input_value",
      "name": "DEGREE_7",
      "check": "Number",
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
  "previousStatement": null,
  "nextStatement": null,
  "colour": 0,
  "tooltip": "",
  "helpUrl": ""
}]);

Blockly.Python['robot_jog_joints_not_relative'] = function(block) {
  var deg_1 = Blockly.Python.valueToCode(block, 'DEGREE_1', Blockly.Python.ORDER_ATOMIC);
  var deg_2 = Blockly.Python.valueToCode(block, 'DEGREE_2', Blockly.Python.ORDER_ATOMIC);
  var deg_3 = Blockly.Python.valueToCode(block, 'DEGREE_3', Blockly.Python.ORDER_ATOMIC);
  var deg_4 = Blockly.Python.valueToCode(block, 'DEGREE_4', Blockly.Python.ORDER_ATOMIC);
  var deg_5 = Blockly.Python.valueToCode(block, 'DEGREE_5', Blockly.Python.ORDER_ATOMIC);
  var deg_6 = Blockly.Python.valueToCode(block, 'DEGREE_6', Blockly.Python.ORDER_ATOMIC);
  var deg_7 = Blockly.Python.valueToCode(block, 'DEGREE_7', Blockly.Python.ORDER_ATOMIC);
  var number_speed = block.getFieldValue('SPEED');
  // TODO: Assemble Python into code variable.
  var code = 'self.jog_joints([' +deg_1+', ' +deg_2+', '+deg_3+', '+deg_4+', '+deg_5+', '+deg_6+', '+deg_7+'], ' + number_speed + ')\n';
  return code;
};

Blockly.Python['robot_jog_joints_relative'] = function(block) {
  var deg_1 = Blockly.Python.valueToCode(block, 'DEGREE_1', Blockly.Python.ORDER_ATOMIC);
  var deg_2 = Blockly.Python.valueToCode(block, 'DEGREE_2', Blockly.Python.ORDER_ATOMIC);
  var deg_3 = Blockly.Python.valueToCode(block, 'DEGREE_3', Blockly.Python.ORDER_ATOMIC);
  var deg_4 = Blockly.Python.valueToCode(block, 'DEGREE_4', Blockly.Python.ORDER_ATOMIC);
  var deg_5 = Blockly.Python.valueToCode(block, 'DEGREE_5', Blockly.Python.ORDER_ATOMIC);
  var deg_6 = Blockly.Python.valueToCode(block, 'DEGREE_6', Blockly.Python.ORDER_ATOMIC);
  var deg_7 = Blockly.Python.valueToCode(block, 'DEGREE_7', Blockly.Python.ORDER_ATOMIC);
  var number_speed = block.getFieldValue('SPEED');
  // TODO: Assemble Python into code variable.
  var code = 'self.jog_joints_relative([' +deg_1+', ' +deg_2+', '+deg_3+', '+deg_4+', '+deg_5+', '+deg_6+', '+deg_7+'], ' + number_speed + ')\n';
  return code;
};