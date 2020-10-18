async function run_discovery(){
    // Start the python code of auto discovery of the available robots
    await languagePluginLoader;
    await pyodide.loadPackage(["numpy"]);

    const response_discovery = await fetch("./RR-Client-WebBrowser-Discovery.py", {cache: "no-store"});
    const client_discovery_py = await response_discovery.text();
    pyodide.runPython(client_discovery_py)
}

run_discovery();