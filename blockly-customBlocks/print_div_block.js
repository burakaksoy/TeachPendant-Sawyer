Blockly.defineBlocksWithJsonArray([{
  "type": "print_div",
  "message0": "print_div %1",
  "args0": [
    {
      "type": "input_value",
      "name": "TEXT"
    }
  ],
  "previousStatement": null,
  "nextStatement": null,
  "colour": 160,
  "tooltip": "Prints Input to Debug Output Div",
  "helpUrl": ""
}]);

Blockly.Python['print_div'] = function(block) {
  var value_text = Blockly.Python.valueToCode(block, 'TEXT', Blockly.Python.ORDER_NONE) || '\'\'';
  // TODO: Assemble Python into code variable.
  var code = 'print_div(' + value_text + ')\n';
  return code;
};