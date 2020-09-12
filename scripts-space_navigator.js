// Function to execute pyodide code
async function run_gamepad(code_text){
        pyodide.runPython(code_text);
}

// ----- Later added codes 
function applyDeadzone(controller){
    // Default axis values for gamepad controller.
    var default_axis_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    // Current axis values
    var current_axis_values = controller.axes;
    var deadzoned_current_axis_values = [];

    var tolerance = 0.25;

    for (i = 0; i < current_axis_values.length; i++) {
        difference = Math.abs(current_axis_values[i] - default_axis_values[i]);
        
        if(difference < tolerance){
            deadzoned_current_axis_values.push(default_axis_values[i]); 
        }
        else {
            deadzoned_current_axis_values.push(current_axis_values[i]);
        }
    }
    return deadzoned_current_axis_values;
}

// -----

// -----------------------------------------------------------
// Functions for gamepad.
var haveEvents = 'ongamepadconnected' in window;
var controllers = {};

function connecthandler(e) {
    addgamepad(e.gamepad);

    /*
    // Example to run pyodide code from here.
    var code_text = "print_div('Gamepad is ready.<br>')";
    run_gamepad(code_text);
    */

}

function addgamepad(gamepad) {
    controllers[gamepad.index] = gamepad;

    print_div("gamepad: " + gamepad.id + " is connected.<br>");
    print_div("There are " + gamepad.buttons.length + " buttons, and " +  gamepad.axes.length  + " axes.<br>");
    print_div("Press Center completely and pull completely back to start using the gamepad.<br>");

    requestAnimationFrame(updateStatus);
}

function disconnecthandler(e) {
    removegamepad(e.gamepad);
}

function removegamepad(gamepad) {
    print_div("gamepad: " + gamepad.id + " is disconnected.<br>");
    delete controllers[gamepad.index];
}

var is_gamepadbuttondown = false;
function buttonUpDownListener(controller) {
    var is_any_pressed = (element) => element.pressed;

    if (controller.buttons.some(is_any_pressed) && !is_gamepadbuttondown) {
        is_gamepadbuttondown = true;

        var code_text = "gamepadbuttondown()";
        run_gamepad(code_text);

    } else if (!controller.buttons.some(is_any_pressed) && is_gamepadbuttondown) {
        is_gamepadbuttondown = false;

        var code_text = "gamepadbuttonup()";
        run_gamepad(code_text);
    }
}


function is_any_axis_active(controller){
    // Default axis values for gamepad controller.
    var default_axis_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    
    // Current axis values
    var current_axis_values = controller.axes;

    var tolerance = 0.25;
    for (i = 0; i < current_axis_values.length; i++) {
        // difference = deadzoned_current_axis_values[i] - default_axis_values[i];
        difference = Math.abs(current_axis_values[i] - default_axis_values[i]);
        
        // console.log("3");
        // console.log(difference);
        
        if(difference >= tolerance){
            return true;
        } 
    }
    return false
}
    

var is_gamepadaxisactive = false;
function axisInactiveActiveListener(controller) {
    var any_axis_active = is_any_axis_active(controller);

    if (any_axis_active && !is_gamepadaxisactive) {
        is_gamepadaxisactive = true;

        var code_text = "gamepadaxisactive()";
        run_gamepad(code_text);

    } else if (!any_axis_active && is_gamepadaxisactive) {
        is_gamepadaxisactive = false;

        var code_text = "gamepadaxisinactive()";
        run_gamepad(code_text);
    }
}


function gamepadModeSwitcher(controller){
    var gamepad_modes = ["joint_jog", "cartesian_jog", "locked"];

    return gamepad_modes[1];
}

function gamepadMode(mode, controller){
    var axes = applyDeadzone(controller);

    switch(mode) {
        case "joint_jog":
            var joint_speed_constants = [axes[0],axes[1],axes[3],axes[4],axes[6],axes[7],(-controller.buttons[4].value+controller.buttons[5].value)];

            // if (is_gamepadbuttondown){
            //    console.log("jog_joints_gamepad([" + joint_speed_constants + "])"); 
            // }
            var code_text = "jog_joints_gamepad([" + joint_speed_constants + "])";
            run_gamepad(code_text);

            if(controller.buttons[2].pressed){
                var code_text = "home_func_gamepad()";
                run_gamepad(code_text);
            }

            break;
        
        case "cartesian_jog":
            // print_div("cartesian_jog mode")
            
            // var P_axis = [axes[0],-axes[1],-axes[2]]; 
            // var R_axis = [-axes[4],-axes[3],-axes[5]];

            var P_axis = [axes[1],axes[0],-axes[2]]; 
            var R_axis = [axes[4],axes[3],-axes[5]];

            // if (is_gamepadbuttondown){
            //    console.log("jog_cartesian_gamepad([" + P_axis + "]," + "[" + R_axis + "]  )"); 
            // }
            var code_text = "jog_cartesian_gamepad([" + P_axis + "]," + "[" + R_axis + "]  )";
            run_gamepad(code_text);  
            break;
    }
}

var is_unlocked = false;
var is_initial_pressed = false;

function updateStatus() {
    if (!haveEvents) {
        scangamepads();
    }

    var i = 0;
    var j;

    for (j in controllers) {
        var controller = controllers[j];

        if (is_unlocked){
            buttonUpDownListener(controller);
            axisInactiveActiveListener(controller);

            gamepad_mode = gamepadModeSwitcher(controller);

            gamepadMode(gamepad_mode,controller);
        } 
        else if (is_initial_pressed == false && controller.axes[2] == 1.0){
            is_initial_pressed = true;
            print_div("Now pull the center to unlock.<br>");
            
        } else if (is_initial_pressed == true && controller.axes[2] == -1.0){
            is_unlocked = true;
            print_div("Gamepad is Unlocked!<br>");
        }
    }
    requestAnimationFrame(updateStatus);
}

function scangamepads() {
    var gamepads = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads() : []);
    for (var i = 0; i < gamepads.length; i++) {
        if (gamepads[i]) {
            if (gamepads[i].index in controllers) {
                controllers[gamepads[i].index] = gamepads[i];
            } else {
                addgamepad(gamepads[i]);
            }
        }
    }
}

window.addEventListener("gamepadconnected", connecthandler);
window.addEventListener("gamepaddisconnected", disconnecthandler);


if (!haveEvents) {
    setInterval(scangamepads, 500);
}

