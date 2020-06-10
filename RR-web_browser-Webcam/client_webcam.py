
from js import print_div
from js import document
from js import ImageData
from RobotRaconteur.Client import *
import numpy as np

def new_frame(pipe_ep):
    global canvas, ctx
    
    #Loop to get the newest frame
    while (pipe_ep.Available > 0):
        #Receive the packet
        image=pipe_ep.ReceivePacket()
        #Convert the packet to an image and set the global variable
        
        if (canvas == None):
            canvas = document.getElementById("image")
            ctx = canvas.getContext("2d")
        
        imageBytes=np.zeros(4*image.width*image.height, dtype=np.uint8)        #dtype essential here, IndexSizeError
        imageBytes[3::4] = 255
        imageBytes[0::4] = image.data[2::3]
        imageBytes[1::4] = image.data[1::3]
        imageBytes[2::4] = image.data[0::3]

        image_data=ImageData.new(bytes(imageBytes),image.width,image.height)
        ctx.putImageData(image_data, 0, 0,0,0,320,240)
       
async def client_webcam():
    # rr+ws : WebSocket connection without encryption
    url ='rr+ws://localhost:2355?service=Webcam'
    # url ='rr+ws://192.168.1.128:2355?service=Webcam'
    # url ='rr+ws://192.168.43.241:2355?service=Webcam'
    
    try:
        c_host = await RRN.AsyncConnectService(url,None,None,None,None)
        c = await c_host.async_get_Webcams(0,None)

        p = await c.FrameStream.AsyncConnect(-1,None)
        
        global canvas, ctx
        canvas = document.getElementById("image")
        ctx = canvas.getContext("2d")
        print_div("Running!")

        finish_time = 0
        # while True:            
        p.PacketReceivedEvent+=new_frame
            
        c.async_StartStreaming(None)
                   
        await RRN.AsyncSleep(0.01,None)
    except:
        import traceback
        print_div(traceback.format_exc())
        raise

RR.WebLoop.run(client_webcam())
