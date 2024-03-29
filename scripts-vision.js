function print_div(message)
{
    console.log(message) // For debug purposes 
    //$("#print_div").append("<br/>" + message +"\n");
    $("#print_div").append( message +" ");

    // Scroll to the bottom in Golden Laytout as more data added.
    var $textarea = $("#print_div").parent();
    $textarea.scrollTop($textarea[0].scrollHeight);

}

// FOR OBJECT DETECTION TEST
function print_div_test_selected_camera(message)
{
    $("#print_div_test_selected_camera").html(message)
}

function print_div_test_selected_visual(message)
{
    $("#print_div_test_selected_visual").html(message)
}

function print_div_test_selected_camera_img_size(message)
{
    $("#print_div_test_selected_camera_img_size").html(message)
}   

function print_div_test_selected_visual_img_size(message)
{
    $("#print_div_test_selected_visual_img_size").html(message)
}   

function print_div_test_detected_obj_size(message)
{
    $("#print_div_test_detected_obj_size").html(message)
}
function print_div_test_detected_obj_coordinates(message)
{
    $("#print_div_test_detected_obj_coordinates").html(message)
}   
function print_div_test_detected_obj_angle(message)
{
    $("#print_div_test_detected_obj_angle").html(message)
}         

// FOR CAMERA CALIBRATION
function print_div_selected_camera_matrix(message)
{
    $("#print_div_selected_camera_matrix").html(message)
} 
function print_div_selected_camera_distortion_coefficients(message)
{
    $("#print_div_selected_camera_distortion_coefficients").html(message)
} 
function print_div_selected_camera_RnT(message)
{
    $("#print_div_selected_camera_RnT").html(message)
} 
function print_div_selected_camera_calibration_error(message)
{
    $("#print_div_selected_camera_calibration_error").html(message)
} 

function print_div_num_captured_calibration_imgs(message)
{
    $("#print_div_num_captured_calibration_imgs").html("Currently "+ message + " images are captured.")
}

function clear_div_modal_body_CameraCalibration(){
    print_div("clearing modal<br>")
    $("#modal_body_CameraCalibration div").empty()
}

// FOR ROBOT-CAMERA CALIBRATION
function print_div_selected_robotcamerapair_rotation_matrix(message)
{
    $("#print_div_selected_robotcamerapair_rotation_matrix").html(message)
} 
function print_div_selected_robotcamerapair_translation_vector(message)
{
    $("#print_div_selected_robotcamerapair_translation_vector").html(message)
}

function print_div_robotcamera_calibration_selected_robot(message)
{
    $("#print_div_robotcamera_calibration_selected_robot").html(message)
} 
function print_div_robotcamera_calibration_selected_camera(message)
{
    $("#print_div_robotcamera_calibration_selected_camera").html(message)
} 
function print_div_num_captured_robotcamera_calibration_imgs(message)
{
    $("#print_div_num_captured_robotcamera_calibration_imgs").html("Currently "+ message + " images are captured.")
} 

function clear_div_modal_body_RobotCameraCalibration(){
    print_div("clearing modal<br>")
    $("#modal_body_RobotCameraCalibration div").empty()
}

function copy_saved_poses(){
    print_div("Copying saved poses list 'select' into Robot Camera Calibration Modal<br>");
    var $clone = $("#saved_poses_list").clone(true).prop('id', 'saved_poses_list_clone' );
    $clone.appendTo("#table_cell_for_saved_poses");
}
function remove_copied_saved_poses(){
    // print_div("Copying saved poses list 'select' into Robot Camera Calibration Modal<br>")
    $("#table_cell_for_saved_poses").find("#saved_poses_list_clone").remove()
}


function run_vision(){
    run_test_vision();
}

async function run_test_vision(){
    await languagePluginLoader;
    await pyodide.loadPackage(["numpy"]);

    // const response_vision = await fetch("./RR-web_browser-Webcam/client_webcam.py", {cache: "no-store"});
    // const response_vision = await fetch("./RR-Client-WebBrowser-Vision.py", {cache: "no-store"});
    const response_vision = await fetch("./RR-Client-WebBrowser-Vision2.py", {cache: "no-store"});
    const client_vision_py = await response_vision.text();
    pyodide.runPython(client_vision_py)
}

run_vision();


