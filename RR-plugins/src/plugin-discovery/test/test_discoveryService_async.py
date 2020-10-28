from RobotRaconteur.Client import *
import asyncio

import numpy as np


async def clientt():
    ip_plugins = '192.168.50.152' # plugins ip
    url_plugin_discovery = 'rr+ws://' + ip_plugins + ':8896?service=Discovery'
    
    global plugin_discovery
    plugin_discovery = await RRN.AsyncConnectService(url_plugin_discovery,None,None,None,None)
    print('discovery plugin is connected..<br>')

    RobotConnectionURLs = await plugin_discovery.async_available_robot_ConnectionURLs(None)
    print(str(RobotConnectionURLs))
    RobotNames = await plugin_discovery.async_available_robot_Names(None)
    print(str(RobotNames))
    RobotNodeNames = await plugin_discovery.async_available_robot_NodeNames(None)
    print(str(RobotNodeNames))

    '''
        Expected Output Example:
        ['rr+local:///?nodeid=88ada0ca-6275-4e64-ac17-e299671ead07&service=robot', 'rr+tcp://[fe80::7c64:bf9f:7c1d:5a9e]:58653/?nodeid=eb42bd99-6352-4784-9769-6a6ea260f558&service=robot']
        ['robot', 'robot']
        ['rp260_robot', 'sawyer_robot']
    '''

    # Set the url of the robot
    url = RobotConnectionURLs[1]
    print(str(url) + "<br>")


    #Connect to the service
    global d # d is the robot object from RR
    d = await RRN.AsyncConnectService(url,None,None,None,None)

    d.async_reset_errors(None)
    d.async_enable(None)

    joint_diff = np.array([50,0,0,0,0,0,0])*np.deg2rad(1)
    joint_vel = np.ones((7,))


    while (True):
        await d.async_jog_freespace(joint_diff, joint_vel, False, None)
        print(hex(d.robot_state.PeekInValue()[0].robot_state_flags))
        # await RRN.AsyncSleep(1, None)


loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)
loop.run_until_complete(clientt())