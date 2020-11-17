Blockly.defineBlocksWithJsonArray([{
  "type": "camera_get_object_pose_z_required",
  "lastDummyAlign0": "RIGHT",
  "message0": "Get Pose of object: %1 %2 wrt. cam: %3 %4 with z = %5 (m)",
  "args0": [
    {
      "type": "field_dropdown",
      "name": "TRAINED_OBJECTS",
      "options": [
        [
          {
            "src": "https://www.gstatic.com/codesite/ph/images/star_on.gif",
            "width": 30,
            "height": 30,
            "alt": "*"
          },
          "OBJECT1"
        ],
        [
          {
            "src": "https://www.gstatic.com/codesite/ph/images/star_on.gif",
            "width": 30,
            "height": 30,
            "alt": "*"
          },
          "OBJECT2"
        ],
        [
          {
            "src": "https://www.gstatic.com/codesite/ph/images/star_on.gif",
            "width": 30,
            "height": 30,
            "alt": "*"
          },
          "OBJECT3"
        ]
      ]
    },
    {
      "type": "input_dummy",
      "align": "RIGHT"
    },
    {
      "type": "field_dropdown",
      "name": "CAMS",
      "options": [
        [
          "cam1",
          "CAM1"
        ],
        [
          "cam2",
          "CAM2"
        ],
        [
          "cam3",
          "CAM3"
        ]
      ]
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
  "helpUrl": ""
},
{
  "type": "camera_get_object_pose_z_not_required",
  "lastDummyAlign0": "RIGHT",
  "message0": "Get Pose of object: %1 %2 wrt. 3D capable cam: %3",
  "args0": [
    {
      "type": "field_dropdown",
      "name": "TRAINED_OBJECTS",
      "options": [
        [
          {
            "src": "https://www.gstatic.com/codesite/ph/images/star_on.gif",
            "width": 30,
            "height": 30,
            "alt": "*"
          },
          "OBJECT1"
        ],
        [
          {
            "src": "https://www.gstatic.com/codesite/ph/images/star_on.gif",
            "width": 30,
            "height": 30,
            "alt": "*"
          },
          "OBJECT2"
        ],
        [
          {
            "src": "https://www.gstatic.com/codesite/ph/images/star_on.gif",
            "width": 30,
            "height": 30,
            "alt": "*"
          },
          "OBJECT3"
        ]
      ]
    },
    {
      "type": "input_dummy",
      "align": "RIGHT"
    },
    {
      "type": "field_dropdown",
      "name": "CAMS",
      "options": [
        [
          "cam1",
          "CAM1"
        ],
        [
          "cam2",
          "CAM2"
        ],
        [
          "cam3",
          "CAM3"
        ]
      ]
    }
  ],
  "inputsInline": true,
  "output": "pose",
  "colour": 180,
  "tooltip": "Use this block to get the pose of a trained object with respect to a 3D capable camera (eg Kinect, Intel Realsense). Hence, you DON'T need to specify the z direction distance away from the camera to get correct position values. However, be careful to choose a camera with 3D capability",
  "helpUrl": ""
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
    {
      "type": "field_dropdown",
      "name": "CAMS",
      "options": [
        [
          "cam1",
          "CAM1"
        ],
        [
          "cam2",
          "CAM2"
        ],
        [
          "cam3",
          "CAM3"
        ]
      ]
    }
  ],
  "inputsInline": true,
  "output": "pose",
  "colour": 180,
  "tooltip": "Transforms the given pose in a camera frame into the robot's frame",
  "helpUrl": ""
}]);

Blockly.Python['camera_get_object_pose_z_required'] = function(block) {
  var dropdown_trained_objects = block.getFieldValue('TRAINED_OBJECTS');
  var dropdown_cams = block.getFieldValue('CAMS');
  var value_z_distance = Blockly.Python.valueToCode(block, 'Z_DISTANCE', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = '...';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['camera_get_object_pose_z_not_required'] = function(block) {
  var dropdown_trained_objects = block.getFieldValue('TRAINED_OBJECTS');
  var dropdown_cams = block.getFieldValue('CAMS');
  // TODO: Assemble Python into code variable.
  var code = '...';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Python['camera_transform_pose_to_robot'] = function(block) {
  var value_pose_in_cam = Blockly.Python.valueToCode(block, 'POSE_IN_CAM', Blockly.Python.ORDER_ATOMIC);
  var dropdown_cams = block.getFieldValue('CAMS');
  // TODO: Assemble Python into code variable.
  var code = '...';
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};