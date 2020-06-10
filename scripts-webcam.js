async function run_webcam(){
    await languagePluginLoader;
    await pyodide.loadPackage(["numpy"]);

    const response_webcam = await fetch("./RR-web_browser-Webcam/client_webcam.py", {cache: "no-store"});
    const client_webcam_py = await response_webcam.text();
    pyodide.runPython(client_webcam_py)
}

run_webcam();