function print_div(message)
{
    console.log(message) // For debug purposes 
    //$("#print_div").append("<br/>" + message +"\n");
    $("#print_div").append( message +" ");

    // Scroll to the bottom in Golden Laytout as more data added.
    var $textarea = $("#print_div").parent();
    $textarea.scrollTop($textarea[0].scrollHeight);

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


