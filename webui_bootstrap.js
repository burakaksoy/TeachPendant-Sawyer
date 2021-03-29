
async function webui_bootstrap(){
    await languagePluginLoader;
    await pyodide.loadPackage(["numpy","micropip"]);
    const response = await fetch("webui_bootstrap.py", {cache: "no-store"});
    const webui_bootstrap_py = await response.text();
    pyodide.runPython(webui_bootstrap_py)
}

function golden_layout_new(config, target)
{
    return new window.GoldenLayout(config,target)
}

function golden_layout_register_component(layout, name, component_py)
{
    layout.registerComponent(name, function(container, state)
        {
            this.py_this = component_py(container,state)
        }    
    )
}

function golden_layout_append_menu_item(component_name)
{
    var element_name = $( '<li>' + component_name + '</li>' );

    $( '#menuContainer' ).append( element_name );
    
    return element_name

}

function vuex_store_new(args)
{
    return new Vuex.Store(args)
}

function python_to_js(obj)
{
    //console.log(obj)
    let js_obj = obj.toJs()
    obj.destroy()
    return js_obj
}

function pyodide_set_timeout(handler, delay)
{
  setTimeout(function() {
    try
    {
        handler()
    }
    catch (e)
    {
      handler.destroy()
      throw e
    }
  }, delay)
}


webui_bootstrap();
