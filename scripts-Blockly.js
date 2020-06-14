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
        var code_js = Blockly.JavaScript.workspaceToCode(workspace);
        document.getElementById('textareaBlocklyJS').value = code_js;

        var code_py = Blockly.Python.workspaceToCode(workspace);
        document.getElementById('textareaBlocklyPy').value = code_py;
    }
    workspace.addChangeListener(myUpdateFunction);
});

