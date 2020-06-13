// $(document).ready(function() {
//     var demoWorkspace = Blockly.inject('blocklyDiv',
//         {media: './blockly/media/',
//          toolbox: document.getElementById('toolbox')});
// });

$(document).ready(function() {
    var blocklyArea = document.getElementById('blocklyArea');
    var blocklyDiv = document.getElementById('blocklyDiv');
    var demoWorkspace = Blockly.inject(blocklyDiv,
        {media: '../../media/',
         toolbox: document.getElementById('toolbox')});
    
    function onresize_cb(){
        // Compute the absolute coordinates and dimensions of blocklyArea.
        // console.log("resized..");
        var element = blocklyArea;
        var x = 0;
        var y = 0;
        do {
        x += element.offsetLeft;
        y += element.offsetTop;
        element = element.offsetParent;
        } while (element);

        // Position blocklyDiv over blocklyArea.
        blocklyDiv.style.left = x + 'px';
        blocklyDiv.style.top = y + 'px';
        blocklyDiv.style.width = blocklyArea.offsetWidth + 'px';
        blocklyDiv.style.height = blocklyArea.offsetHeight + 'px';
        Blockly.svgResize(demoWorkspace);
    }

    var table_blockly = document.getElementById('table_Blockly');
    var parent_div = table_blockly.parentNode;
    console.log(parent_div);

    new ResizeObserver(onresize_cb).observe(parent_div);

    onresize_cb();
    Blockly.svgResize(demoWorkspace);
});

