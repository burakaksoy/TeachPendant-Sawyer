Blockly.defineBlocksWithJsonArray([{
  "type": "camera_get_object_pose_z_required",
  "lastDummyAlign0": "RIGHT",
  "message0": "Get Pose of object: %1 %2 wrt. cam: %3 %4 with z = %5 (m)",
  "args0": [
    // {
    //   "type": "field_dropdown",
    //   "name": "TRAINED_OBJECTS",
    //   "options": [
    //     [
    //       {
    //         "src": "https://www.gstatic.com/codesite/ph/images/star_on.gif",
    //         "width": 30,
    //         "height": 30,
    //         "alt": "*"
    //       },
    //       "OBJECT1"
    //     ],
    //     [
    //       {
    //         "src": "https://www.gstatic.com/codesite/ph/images/star_on.gif",
    //         "width": 30,
    //         "height": 30,
    //         "alt": "*"
    //       },
    //       "OBJECT2"
    //     ],
    //     [
    //       {
    //         "src": "https://www.gstatic.com/codesite/ph/images/star_on.gif",
    //         "width": 30,
    //         "height": 30,
    //         "alt": "*"
    //       },
    //       "OBJECT3"
    //     ]
    //   ]
    // },
    {
      "type": "input_dummy",
      "align": "RIGHT",
      "name" : "TRAINED_OBJECTS"
    },
    {
      "type": "input_dummy",
      "align": "RIGHT"
    },
    // {
    //   "type": "field_dropdown",
    //   "name": "CAMS",
    //   "options": [
    //     [
    //       "cam1",
    //       "CAM1"
    //     ],
    //     [
    //       "cam2",
    //       "CAM2"
    //     ],
    //     [
    //       "cam3",
    //       "CAM3"
    //     ]
    //   ]
    // },
    {
      "type": "input_dummy",
      "align": "RIGHT",
      "name" : "CAMS"
    },
    {
      "type": "input_dummy",
      "align": "RIGHT"
    },
    {
      "type": "input_value",
      "name": "Z_DISTANCE",
      "check": "Number",
      "align": "RIGHT"
    }
  ],
  "inputsInline": true,
  "output": "pose",
  "colour": 180,
  "tooltip": "Use this block to get the pose of a trained object with respect to a camera. You need to specify the z direction distance away from the camera to get correct position values",
  "helpUrl": "",
  "extensions": ["cams_dynamic_menu_extension","images_dynamic_menu_extension"] //Added
},
{
  "type": "camera_get_object_pose_z_not_required",
  "lastDummyAlign0": "RIGHT",
  "message0": "Get Pose of object: %1 %2 wrt. 3D capable cam: %3",
  "args0": [
    // {
    //   "type": "field_dropdown",
    //   "name": "TRAINED_OBJECTS",
    //   "options": [
    //     [
    //       {
    //         "src": "https://www.gstatic.com/codesite/ph/images/star_on.gif",
    //         "width": 30,
    //         "height": 30,
    //         "alt": "*"
    //       },
    //       "OBJECT1"
    //     ],
    //     [
    //       {
    //         "src": "https://www.gstatic.com/codesite/ph/images/star_on.gif",
    //         "width": 30,
    //         "height": 30,
    //         "alt": "*"
    //       },
    //       "OBJECT2"
    //     ],
    //     [
    //       {
    //         "src": "https://www.gstatic.com/codesite/ph/images/star_on.gif",
    //         "width": 30,
    //         "height": 30,
    //         "alt": "*"
    //       },
    //       "OBJECT3"
    //     ]
    //   ]
    // },
    {
      "type": "input_dummy",
      "align": "RIGHT",
      "name" : "TRAINED_OBJECTS"
    },
    {
      "type": "input_dummy",
      "align": "RIGHT"
    },
    // {
    //   "type": "field_dropdown",
    //   "name": "CAMS",
    //   "options": [
    //     [
    //       "cam1",
    //       "CAM1"
    //     ],
    //     [
    //       "cam2",
    //       "CAM2"
    //     ],
    //     [
    //       "cam3",
    //       "CAM3"
    //     ]
    //   ]
    // }
    {
      "type": "input_dummy",
      "align": "RIGHT",
      "name" : "CAMS"
    }
  ],
  "inputsInline": true,
  "output": "pose",
  "colour": 180,
  "tooltip": "Use this block to get the pose of a trained object with respect to a 3D capable camera (eg Kinect, Intel Realsense). Hence, you DON'T need to specify the z direction distance away from the camera to get correct position values. However, be careful to choose a camera with 3D capability",
  "helpUrl": "",
  "extensions": ["cams_dynamic_menu_extension","images_dynamic_menu_extension"] //Added
},
{
  "type": "camera_transform_pose_to_robot",
  "lastDummyAlign0": "RIGHT",
  "message0": "Transform pose %1 in cam %2 frame wrt. robot frame",
  "args0": [
    {
      "type": "input_value",
      "name": "POSE_IN_CAM",
      "check": "pose",
      "align": "RIGHT"
    },
    // {
    //   "type": "field_dropdown",
    //   "name": "CAMS",
    //   "options": [
    //     [
    //       "cam1",
    //       "CAM1"
    //     ],
    //     [
    //       "cam2",
    //       "CAM2"
    //     ],
    //     [
    //       "cam3",
    //       "CAM3"
    //     ]
    //   ]
    // },
    // {
    //   "type": "input_dummy",
    //   "align": "RIGHT"
    // },
    {
      "type": "input_dummy",
      "align": "RIGHT",
      "name" : "CAMS"
    }
  ],
  "inputsInline": true,
  "output": "pose",
  "colour": 180,
  "tooltip": "Transforms the given pose in a camera frame into the robot's frame",
  "helpUrl": "",
  "extensions": ["cams_dynamic_menu_extension"] //Added
}]);

Blockly.Python['camera_get_object_pose_z_required'] = function(block) {
  var dropdown_trained_objects = block.getFieldValue('OPTIONS_IMAGES');
  var dropdown_cams = block.getFieldValue('OPTIONS_CAMS');
  var value_z_distance = Blockly.Python.valueToCode(block, 'Z_DISTANCE', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'self.camera_get_object_pose_z_required("'+ dropdown_trained_objects +'","'+ dropdown_cams +'",'+ value_z_distance +')';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['camera_get_object_pose_z_not_required'] = function(block) {
  var dropdown_trained_objects = block.getFieldValue('OPTIONS_IMAGES');
  var dropdown_cams = block.getFieldValue('OPTIONS_CAMS');
  // TODO: Assemble Python into code variable.
  var code = 'self.camera_get_object_pose_z_not_required("'+ dropdown_trained_objects +'","'+ dropdown_cams +'")';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['camera_transform_pose_to_robot'] = function(block) {
  var value_pose_in_cam = Blockly.Python.valueToCode(block, 'POSE_IN_CAM', Blockly.Python.ORDER_ATOMIC);
  var dropdown_cams = block.getFieldValue('OPTIONS_CAMS');
  // TODO: Assemble Python into code variable.
  var code = 'self.camera_transform_pose_to_robot('+  value_pose_in_cam + ',"'+ dropdown_cams +'")';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};

function register_vision_extensions_blockly(){
  console.log("i am in register_vision_extensions in blockly");

  function options_cams(){
    options = pyodide.globals.cli_vision.camera_node_names_lst_for_blockly(); 
    // options = [["com.robotraconteur.imaging.camera","com.robotraconteur.imaging.camera"]] // TODO: REMOVE THIS LINE
    console.log(options);
    return options
  }
  Blockly.Extensions.register('cams_dynamic_menu_extension',
    function() {
      this.getInput('CAMS')
        .appendField(new Blockly.FieldDropdown(options_cams()), 'OPTIONS_CAMS');
    });

  function options_images(){
    options = pyodide.globals.cli_vision.image_files_lst_for_blockly();
    console.log(options);
    return options
  }  
  Blockly.Extensions.register('images_dynamic_menu_extension',
    function() {
      this.getInput('TRAINED_OBJECTS')
        .appendField(new Blockly.FieldDropdown(options_images()), 'OPTIONS_IMAGES');
    });
}