// $(document).ready(function() {
//     var workspace = Blockly.inject('blocklyDiv',
//         {media: './blockly/media/',
//          toolbox: document.getElementById('toolbox')});
// });

$(document).ready(function() {
    // Inject Blockly workspace
    var blocklyArea = document.getElementById('blocklyArea');
    var blocklyDiv = document.getElementById('blocklyDiv');
    var workspace = Blockly.inject(blocklyDiv,
        {
            toolbox: document.getElementById('toolbox'),
            zoom:
                {
                    controls: true,
                    wheel: false,
                    startScale: 1.0,
                    maxScale: 3,
                    minScale: 0.3,
                    scaleSpeed: 1.2
                },
            trashcan: true,  
            maxTrashcanContents: 32, // Default 32
            css:true,
            horizontalLayout: false, // Defaul: false
            move:
                {
                    scrollbars: true,
                    drag: true,
                    wheel: false
                },
        });

    // Blockly.Xml.domToWorkspace(document.getElementById('startBlocks'), workspace);
    
    // For resizable workspace
    function onresize_cb(){
        blocklyDiv.style.width = parent_div.offsetWidth-5 + 'px';
        blocklyDiv.style.height = parent_div.offsetHeight-5 + 'px';
        Blockly.svgResize(workspace);
    }
    var table_blockly = document.getElementById('table_BlocklyWorkspace');
    var parent_div = table_blockly.parentNode;
    // console.log(parent_div);
    new ResizeObserver(onresize_cb).observe(parent_div);
    onresize_cb();
    Blockly.svgResize(workspace);

    // Realtime Code Generation
    function myUpdateFunction(event) {
        // var code_js = Blockly.JavaScript.workspaceToCode(workspace);
        // document.getElementById('textareaBlocklyJS').value = code_js;

        var code_py = Blockly.Python.workspaceToCode(workspace);
        document.getElementById('textareaBlocklyPy').value = code_py;
    }
    workspace.addChangeListener(myUpdateFunction);


    // var executeBlockly_btn = $('#execute_blockly_btn');
    // executeBlockly_btn.click(function(){
    //     // console.log("Blockly Execute button is clicked!");
    //     // document.getElementById('textareaBlocklyPy').value += "\nBlockly Execute button is clicked!\n";
    //     print_div("<br>Blockly Execute button is clicked!<br><br>");

    //     async function run_blockly(code_text){
    //         // await languagePluginLoader;
    //         // await pyodide.loadPackage(["numpy"]);

    //         // const response_webcam = await fetch("./RR-web_browser-Webcam/client_webcam.py", {cache: "no-store"});
    //         // const client_webcam_py = await response_webcam.text();
    //         // pyodide.runPython(client_webcam_py);
    //         pyodide.runPython(code_text);
    //     }

    //     // code_text = "from js import print_div\nprint_div('HELLO WORLD')";
    //     // code_text = "print_div('HELLO WORLD')\njog_joints(1,+1)";
    //     var code_text = Blockly.Python.workspaceToCode(workspace);
        
    //     run_blockly(code_text);
    // });
});

