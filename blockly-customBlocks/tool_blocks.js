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
          "ON"
        ],
        [
          "CLOSED",
          "OFF"
        ]
      ]
    }
  ],
  "previousStatement": null,
  "nextStatement": null,
  "colour": 60,
  "tooltip": "Change the Gripper Mode",
  "helpUrl": ""
}]);

Blockly.Python['tool_gripper'] = function(block) {
  var dropdown_gripper_status = block.getFieldValue('GRIPPER_STATUS');
  // TODO: Assemble Python into code variable.
  var code = '...\n';
  return code;
};