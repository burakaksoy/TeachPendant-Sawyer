Blockly.defineBlocksWithJsonArray([{
  "type": "tool_gripper",
  "message0": "Gripper %1",
  "args0": [
    {
      "type": "field_dropdown",
      "name": "GRIPPER_STATUS",
      "options": [
        [
          "OPEN",
          "1"
        ],
        [
          "CLOSED",
          "0"
        ]
      ]
    }
  ],
  "previousStatement": null,
  "nextStatement": null,
  "colour": 60,
  "tooltip": "Change the Gripper Mode",
  "helpUrl": ""
},{
  "type": "tool_link_attacher",
  "message0": "%1 link between: Robot: %2 and Object: %3",
  "args0": [
    {
      "type": "field_dropdown",
      "name": "STATUS",
      "options": [
        [
          "Attach",
          "1"
        ],
        [
          "Detach",
          "0"
        ]
      ]
    },
    {
      "type": "field_input",
      "name": "ROBOT_NAME",
      "text": "sawyer"
    },
    {
      "type": "field_input",
      "name": "OBJ_NAME",
      "text": "toothpaste"
    }
  ],
  "previousStatement": null,
  "nextStatement": null,
  "colour": 60,
  "tooltip": "Attach/Detach a link between Robot and Object in Gazebo Simulation",
  "helpUrl": ""
}]);

Blockly.Python['tool_gripper'] = function(block) {
  var dropdown_gripper_status = block.getFieldValue('GRIPPER_STATUS');
  // TODO: Assemble Python into code variable.
  var code = 'self.tool_gripper(' + dropdown_gripper_status + ')\n';
  return code;
};
Blockly.Python['tool_link_attacher'] = function(block) {
  var dropdown_status = block.getFieldValue('STATUS');
  var text_robot_name = block.getFieldValue('ROBOT_NAME');
  var text_obj_name = block.getFieldValue('OBJ_NAME');
  // TODO: Assemble Python into code variable.
  var code = 'self.tool_link_attacher("'+ text_robot_name +'","'+ text_obj_name + '",' + dropdown_status + ')\n';
  return code;
};