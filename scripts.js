function print_div(message)
{
    //$("#print_div").append("<br/>" + message +"\n");
    $("#print_div").append( message +" ");

    // Scroll to the bottom in Golden Laytout as more data added.
    var $textarea = $("#print_div").parent();
    $textarea.scrollTop($textarea[0].scrollHeight);

}         
function print_div_j_info(message)
{   
    var id="" 
    var angles_array = message.split("<br>");
    for (let i = 0; i < angles_array.length - 1 ; i++) {
        id = "j" + (i+1).toString() + "_angle_out" 
        document.getElementById(id).innerHTML = angles_array[i];
    }
}
function print_div_j_limit_info(message_lower, message_upper)
{   
    var id="j1_neg_limit" 
    var lower_angles_array = message_lower.split(" ");
    for (let i = 0; i < lower_angles_array.length - 1; i++) {
        id = "j" + (i+1).toString() + "_neg_limit" 
        document.getElementById(id).innerHTML = lower_angles_array[i];
    }
    
    var id="j1_pos_limit" 
    var upper_angles_array = message_upper.split(" ");
    for (let i = 0; i < upper_angles_array.length -1; i++) {
        id = "j" + (i+1).toString() + "_pos_limit" 
        document.getElementById(id).innerHTML = upper_angles_array[i];
    }
}

function print_div_flag_info(message)
{
    $("#print_div_flag_info").html(message)
}
function print_div_kin_info(message)
{
    $("#print_div_kin_info").html(message)
}
function print_div_num_info(message)
{
    $("#print_div_num_info").html(message)
}
function print_div_end_info(message)
{
    $("#print_div_end_info").html(message)
}
function print_div_ik_info(message)
{
    $("#print_div_ik_info").html(message)
}

function print_div_cur_pose(msg_orientation, msg_position)
{
    $("#cur_ZYX_angles").html(msg_orientation)
    $("#cur_position").html(msg_position)
}

function upSelPose()
{
    var x = document.getElementById("saved_poses_list");
    var index = x.selectedIndex
    if (index > 0) {
      var option = x.options[index];
      x.remove(index);
      x.add(option,index-1)
    }
}

function downSelPose()
{
    var x = document.getElementById("saved_poses_list");
    var index = x.selectedIndex
    if (index < x.length-1) {
      var option = x.options[index];
      x.remove(index);
      x.add(option,index+1)
    }
}

function delSelPose()
{
    var x = document.getElementById("saved_poses_list");
    x.remove(x.selectedIndex);
}


function showJointVelValue()
{
    var slider = document.getElementById("joint_vel_range");
    var percent = document.getElementById("joint_vel_percentage");
    percent.innerHTML = slider.value;
}

async function run_test(){
    await languagePluginLoader;
    await pyodide.loadPackage(["numpy"]);      

    const response_zip = await fetch("./my_source.zip", {cache: "no-store"});

    const response = await fetch("./RR-Client-WebBrowser2.py", {cache: "no-store"});
    // const response = await fetch("./RR-Client-WebBrowser.py", {cache: "no-store"});
    const client_zip = await response_zip.arrayBuffer();
    let FS = pyodide._module.FS; 
    FS.writeFile("./my_source.zip", new Uint8Array(client_zip))

    const client_py = await response.text();                
    pyodide.runPython(client_py)
}

run_test();