/*var newJointSpaceControlConfig = {
    width: 45,
    type:'component',
    componentName: 'Joint Space Control',
    componentState: {comp_name: 'joint_space_control'}
};

var newTaskSpaceControlConfig = {
    width: 15,
    type:'component',
    componentName: 'Task Space Control',
    componentState: {comp_name: 'task_space_control'}
};

var newSavePlaybackPosesConfig = {
    width: 40,
    type:'component',
    componentName: 'Save Playback Poses',
    componentState: {comp_name: 'save_playback_poses' }
};

var newRobotStatusConfig = {
    type:'component',
    componentName: 'Robot Status',
    componentState: {comp_name: 'robot_status'}
};

var newDebugOutputDivConfig = {                                    
    isClosable: false,
    type:'component',
    componentName: 'Debug Output Div',
    componentState: {message: 'Debug Output will be shown here..'}
};*/

var config = 
{
    settings:{
        hasHeaders: true,
        constrainDragToContainer: true,
        reorderEnabled: true,
        selectionEnabled: false,
        popoutWholeStack: false,
        blockedPopoutsThrowError: true,
        closePopoutsOnUnload: true,
        showPopoutIcon: false, 
        showMaximiseIcon: true,
        showCloseIcon: false,
    },
    dimensions: 
    {
        borderWidth: 15,
        minItemHeight: 10,
        minItemWidth: 10,
        headerHeight: 30,
        dragProxyWidth: 300,
        dragProxyHeight: 200
    },
    labels: 
    {
        close: 'close',
        maximise: 'maximise',
        minimise: 'minimise',
        popout: 'open in new window'
    },
    content:
    [
        {
            type: 'column',
            content: 
            [
                {
                    height: 65,
                    type:'row',
                    content: 
                    [
                        {
                            width: 25,
                            isClosable: false,
                            type:'component',
                            componentName: 'Joint Space Control',
                            componentState: {comp_name: 'joint_space_control'}
                        },
                        {
                            width: 25,
                            isClosable: false,
                            type:'component',
                            componentName: 'Task Space Control',
                            componentState: {comp_name: 'task_space_control'}
                        },
                        {
                            width: 25,
                            isClosable: false,
                            type:'component',
                            componentName: 'Save Playback Poses',
                            componentState: {comp_name: 'save_playback_poses' }
                        },
                        {
                            width: 25,
                            isClosable: false,
                            type:'component',
                            componentName: 'Robot Preview',
                            componentState: {comp_name: 'robot_preview' }
                        }
                    ]
                },
                {
                    type:'stack',
                    content: 
                    [
                        {
                            isClosable: false,
                            type:'component',
                            componentName: 'Robot Status',
                            componentState: {comp_name: 'robot_status'}
                        },
                        {                                    
                            isClosable: false,
                            type:'component',
                            componentName: 'Debug Output Div',
                            componentState: {message: 'Debug Output will be shown here..'}
                        },
                        {   
                            // reorderEnabled: false,                                 
                            isClosable: false,
                            title: 'VISION',
                            type:'row',
                            content:
                            [
                                {
                                    width: 25,
                                    // reorderEnabled: false,
                                    isClosable: false,
                                    type:'component',
                                    componentName: 'Camera Feedback',
                                    componentState: {comp_name: 'camera_feedback'}
                                },
                                {
                                    width: 25,
                                    // reorderEnabled: false,
                                    isClosable: false,
                                    type:'component',
                                    componentName: 'Train Vision',
                                    componentState: {comp_name: 'train_vision'}
                                },
                                {
                                    width: 25,
                                    // reorderEnabled: false,
                                    isClosable: false,
                                    type:'component',
                                    componentName: 'Camera Calibration',
                                    componentState: {comp_name: 'camera_calibration'}
                                },
                                {
                                    width: 25,
                                    // reorderEnabled: false,
                                    isClosable: false,
                                    type:'component',
                                    componentName: 'Object Detection Test',
                                    componentState: {comp_name: 'camera_tracking'}
                                }
                            ]
                        },
                        {
                            // reorderEnabled: false,
                            isClosable: false,
                            title:'BLOCKLY',
                            type:'row',
                            content: 
                            [
                                {
                                    width: 60,
                                    // reorderEnabled: false,
                                    isClosable: false,
                                    type:'component',
                                    componentName: 'Blockly Workspace',
                                    componentState: {comp_name: 'blockly_workspace'}
                                },
                                {
                                    width: 20,
                                    // reorderEnabled: false,
                                    isClosable: false,
                                    type:'component',
                                    componentName: 'Blockly Code',
                                    componentState: {comp_name: 'blockly_code'}
                                },
                                {
                                    width: 20,
                                    // reorderEnabled: false,
                                    isClosable: false,
                                    type:'component',
                                    componentName: 'Blockly Browser',
                                    componentState: {comp_name: 'blockly_browser'}
                                }
                            ]
                        },
                    ]
                },
            ]
        }
    ]
};

function initializeLayout(){
    var myLayout;
    var savedState = localStorage.getItem( 'savedState' );

    if( savedState !== null ) {
        myLayout = new window.GoldenLayout( JSON.parse( savedState ), $('#layoutContainer')  );
    } else {
        myLayout = new window.GoldenLayout( config, $('#layoutContainer') );
    }

    myLayout.registerComponent( 'Joint Space Control', function( container, state ){
        // var table_JointSpaceControl = $('#table_JointSpaceControl'); 
        // container.getElement().append(table_JointSpaceControl);

        var temp = document.getElementById('template_JointSpaceControl');
        var clon = temp.content.cloneNode(true);
        container.getElement().append(clon);
    });

    myLayout.registerComponent( 'Task Space Control', function( container, state ){
        // var table_TaskSpaceControl = $('#table_TaskSpaceControl'); 
        // container.getElement().append(table_TaskSpaceControl);

        var temp = document.getElementById('template_TaskSpaceControl');
        var clon = temp.content.cloneNode(true);
        container.getElement().append(clon);
    });

    myLayout.registerComponent( 'Save Playback Poses', function( container, state ){
        // var table_SavePlaybackPoses = $('#table_SavePlaybackPoses'); 
        // container.getElement().append(table_SavePlaybackPoses);

        var temp = document.getElementById('template_SavePlaybackPoses');
        var clon = temp.content.cloneNode(true);
        container.getElement().append(clon);
    });

    myLayout.registerComponent( 'Robot Preview', function( container, state ){
        // var table_RobotPreview = $('#table_RobotPreview'); 
        // container.getElement().append(table_RobotPreview);

        var temp = document.getElementById('template_RobotPreview');
        var clon = temp.content.cloneNode(true);
        container.getElement().append(clon);
    });

    myLayout.registerComponent( 'Robot Status', function( container, state ){
        // var table_RobotStatus = $('#table_RobotStatus'); 
        // container.getElement().append(table_RobotStatus);

        var temp = document.getElementById('template_RobotStatus');
        var clon = temp.content.cloneNode(true);
        container.getElement().append(clon);
    });

    myLayout.registerComponent( 'Debug Output Div', function( container, state ){
        // var div_print = $('#print_div');
        // container.getElement().append(div_print);

        var temp = document.getElementById('template_print_div');
        var clon = temp.content.cloneNode(true);
        container.getElement().append(clon);
    });

    myLayout.registerComponent( 'Camera Feedback', function( container, state ){
        var temp = document.getElementById('template_CameraFeedback');
        var clon = temp.content.cloneNode(true);
        container.getElement().append(clon);
    });

    myLayout.registerComponent( 'Train Vision', function( container, state ){
        var temp = document.getElementById('template_TrainVision');
        var clon = temp.content.cloneNode(true);
        container.getElement().append(clon);
    });

    myLayout.registerComponent( 'Camera Calibration', function( container, state ){
        var temp = document.getElementById('template_CameraCalibration');
        var clon = temp.content.cloneNode(true);
        container.getElement().append(clon);
    });

    myLayout.registerComponent( 'Object Detection Test', function( container, state ){
        var temp = document.getElementById('template_CameraTracking');
        var clon = temp.content.cloneNode(true);
        container.getElement().append(clon);
    });

    myLayout.registerComponent( 'Blockly Workspace', function( container, state ){
        var temp = document.getElementById('template_BlocklyWorkspace');
        var clon = temp.content.cloneNode(true);
        container.getElement().append(clon);
    });

    myLayout.registerComponent( 'Blockly Code', function( container, state ){
        var temp = document.getElementById('template_BlocklyCode');
        var clon = temp.content.cloneNode(true);
        container.getElement().append(clon);
    });

    myLayout.registerComponent( 'Blockly Browser', function( container, state ){
        var temp = document.getElementById('template_BlocklyBrowser');
        var clon = temp.content.cloneNode(true);
        container.getElement().append(clon);
    });

    myLayout.init();
    return myLayout;
}

$(document).ready(function() {
    var myLayout = initializeLayout();

    // $("#print_div_ik_info").html("message")

    var element_name = $('#robotOptsContainer');
    $( '#menuContainer' ).append( element_name );
    var seperator = $('<hr>');
    $( '#menuContainer' ).append( seperator );

    var addMenuItem = function(component_name) {
        var element_name = $( '<li>' + component_name + '</li>' );
        
        if (component_name == 'BLOCKLY' || component_name == 'VISION' ){
            var seperator = $('<hr>');
            $( '#menuContainer' ).append( seperator );
        }

        $( '#menuContainer' ).append( element_name );
        
/*        var newItemConfig;
        var element;
        switch(component_name) {
            case 'Joint Space Control':
                newItemConfig = newJointSpaceControlConfig;
                element = $('#table_RobotStatus'); 
                break;
            case 'Task Space Control':
                newItemConfig = newTaskSpaceControlConfig;
                element = $('#table_RobotStatus'); 
                break;
            case 'Save Playback Poses':
                newItemConfig = newSavePlaybackPosesConfig;
                element = $('#table_RobotStatus'); 
                break;
            case 'Robot Status':
                newItemConfig = newRobotStatusConfig;
                element = $('#table_RobotStatus'); 
                break;
            case 'Debug Output Div':
                newItemConfig = newDebugOutputDivConfig;
                element = $('#print_div'); 
                break;
            default:
                // code block
        }  
        myLayout.createDragSource( element_name, newItemConfig );*/

        element_name.click(function(){
            // console.log( myLayout.root.contentItems[0].getItemsByType('stack'));

            for (var i = 0; i < myLayout._getAllContentItems().length; i++) {
                // console.log(myLayout._getAllContentItems()[i].componentName);
                // console.log(myLayout._getAllContentItems()[i]);
                // console.log(myLayout._getAllContentItems()[i].config.title);
                // console.log("------------------------------------------------");
                // if (myLayout._getAllContentItems()[i].componentName == component_name) {
                if (myLayout._getAllContentItems()[i].config.title == component_name) {
                    var contentItem = myLayout._getAllContentItems()[i];
                    // contentItem.tab.header.parent.setActiveContentItem(contentItem);
                    contentItem.parent.setActiveContentItem(contentItem);
                    // contentItem.parent.header.parent.setActiveContentItem(contentItem);
                }
            }
        });
    };

    addMenuItem( 'Joint Space Control');
    addMenuItem( 'Task Space Control');
    addMenuItem( 'Save Playback Poses');
    addMenuItem( 'Robot Preview');
    addMenuItem( 'Robot Status');
    addMenuItem( 'Debug Output Div');
    addMenuItem( 'VISION');
    addMenuItem( 'Camera Feedback');
    addMenuItem( 'Train Vision');
    addMenuItem( 'Camera Calibration');
    addMenuItem( 'Object Detection Test');
    addMenuItem( 'BLOCKLY');
    addMenuItem( 'Blockly Workspace');
    addMenuItem( 'Blockly Code');
    addMenuItem( 'Blockly Browser');

    $(window).resize(function(){
        myLayout.updateSize();
        // console.log("resized22..");
    });

    // Save the current layout configuration state.
    // var save_layout_btn = document.getElementById('saveLayoutState_btn');
    var save_layout_btn = $('#saveLayoutState_btn');
    save_layout_btn.click(function(){
        console.log("Layout is Saved!");
        var state = JSON.stringify( myLayout.toConfig() );
        localStorage.setItem( 'savedState', state );
    });

    // Load the saved layout configuration state
    var load_layout_btn = $('#loadSavedLayoutState_btn');
    load_layout_btn.click(function(){
        console.log("Saved Layout is loading!");
        
        var savedState = localStorage.getItem( 'savedState' );

        if( savedState !== null ) {
            // myLayout.destroy();
            // myLayout = initializeLayout();
            window.location.reload(false); 
        }
    });

    // Load the default layout configuration state
    var def_layout_btn = $('#defaultLayoutState_btn');
    def_layout_btn.click(function(){
        console.log("Default Layout is loading!");
        
        localStorage.clear();

        // myLayout.destroy();
        // myLayout = initializeLayout();
        window.location.reload(false); 
    });
   
});

