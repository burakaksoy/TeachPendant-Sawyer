Blockly.defineBlocksWithJsonArray([{
  "type": "robot_jog_joint_not_relative",
  "message0": "Jog Joint %1 to degree %2 %3",
  "args0": [
    {
      "type": "field_dropdown",
      "name": "JOINT_SELECTED",
      "options": [
        ["1","1"],
        ["2","2"],
        ["3","3"],
        ["4","4"],
        ["5","5"],
        ["6","6"],
        ["7","7"]
      ]
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "input_value",
      "name": "DEGREE",
      "check": "Number"
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
  "message0": "Jog Joint %1 by degree %2 %3 relatively.",
  "args0": [
    {
      "type": "field_dropdown",
      "name": "JOINT_SELECTED",
      "options": [
        ["1","1"],
        ["2","2"],
        ["3","3"],
        ["4","4"],
        ["5","5"],
        ["6","6"],
        ["7","7"]
      ]
    },
    {
      "type": "input_dummy"
    },
    {
      "type": "input_value",
      "name": "DEGREE",
      "check": "Number"
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
  // TODO: Assemble Python into code variable.
  var code = 'self.jog_joint(' + dropdown_joint_selected + ', ' + value_degree + ')\n';
  return code;
};

Blockly.Python['robot_jog_joint_relative'] = function(block) {
  var dropdown_joint_selected = block.getFieldValue('JOINT_SELECTED');
  var value_degree = Blockly.Python.valueToCode(block, 'DEGREE', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'self.jog_joint_relative(' + dropdown_joint_selected + ', ' + value_degree + ')\n';
  return code;
};