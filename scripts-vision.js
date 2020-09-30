async function run_vision(){
    await languagePluginLoader;
    await pyodide.loadPackage(["numpy"]);

    // const response_vision = await fetch("./RR-web_browser-Webcam/client_webcam.py", {cache: "no-store"});
    const response_vision = await fetch("./RR-Client-WebBrowser-Vision.py", {cache: "no-store"});
    const client_vision_py = await response_vision.text();
    pyodide.runPython(client_vision_py)
}

run_vision();