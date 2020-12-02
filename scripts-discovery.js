function print_div(message)
{
    console.log(message) // For debug purposes 
    //$("#print_div").append("<br/>" + message +"\n");
    $("#print_div").append( message +" ");

    // Scroll to the bottom in Golden Laytout as more data added.
    var $textarea = $("#print_div").parent();
    $textarea.scrollTop($textarea[0].scrollHeight);

}   
async function run_discovery(){
    // Start the python code of auto discovery of the available robots
    await languagePluginLoader;
    await pyodide.loadPackage(["numpy"]);

    const response_discovery = await fetch("./RR-Client-WebBrowser-Discovery.py", {cache: "no-store"});
    const client_discovery_py = await response_discovery.text();
    pyodide.runPython(client_discovery_py)
}

run_discovery();