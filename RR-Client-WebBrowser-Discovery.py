# Client for auto discovery of the available robots 
from js import print_div
from js import document

from RobotRaconteur.Client import *
import numpy as np

class ClientDiscovery(object):
    """Client Class to access client data in a more convenient way"""
    def __init__(self, ip_plugins):
        # Service IPs
        self.ip_plugins = ip_plugins
        
        # Port numbers
        self.port_pluginDiscovery_service = '8896'
        
        # Create Service and Plugin URLs 
        # rr+ws : WebSocket connection without encryption
        self.url_plugin_discovery='rr+ws://'+ self.ip_plugins + ':' + self.port_pluginDiscovery_service+'?service=Discovery'

        # Define Element references
        self.define_element_references()
        # Define Event Listeners
        self.define_event_listeners()

        # Call the async main for async opetations on browser
        self.loop_client = RR.WebLoop()
        self.loop_client.call_soon(self.async_client_main())


    def define_element_references(self):
        # print_div("HTML Element references are being created..<br>")
        self.available_robots_list = document.getElementById("available_robots")
        # self.button_start_robot = document.getElementById("start_robot_btn")


    def define_event_listeners(self):
        # print_div("Event Listeners are being created.. <br>")
        # EXAMPLE self.available_robots_list.addEventListener("change", self.select_available_cam_func)
        pass


    async def async_client_main(self):
        # Connect to plugins
        await self.async_connect_to_plugins()

        # while True:
        #     # Get available robot names and add them as options to available_cams in html
        #     await self.async_create_available_robots_list()
        #     # Sleep for 5 seconds
        #     await RRN.AsyncSleep(5,None)

        # Get available robot names and add them as options to available_cams in html
        await self.async_create_available_robots_list()


    async def async_connect_to_plugins(self):
        ## Discovery plugin
        print_div('Discovery plugin is connecting. ..<br>')
        try:
            self.plugin_discovery = await RRN.AsyncConnectService(self.url_plugin_discovery,None,None,None,None)
            print_div('Discovery plugin is connected!<br>')
        except:
            import traceback
            print_div(traceback.format_exc())


    async def async_create_available_robots_list(self):
        print_div("Auto discovering..")
        await self.plugin_discovery.async_autodiscover(None)

        try:
            # print_div("Clearing the previous available robot options..")
            length = self.available_robots_list.options.length
            i = length-1
            while i >= 0:
                self.available_robots_list.options[i] = None
                i -= 1

            print_div('Creating available robots options..<br>')
            self.robot_nodeNames = await self.plugin_discovery.async_available_robot_NodeNames(None) 
            print_div(str(self.robot_nodeNames) + "<br>") 
            i = 0
            for nodeName in self.robot_nodeNames:
                # print_div(str(self.robot_nodeNames[key]) + "<br>") # --> Right
                # Add the available robot nodeName to the available_robots list
                option = document.createElement("option")
                option.text = str(i) + ": " + str(nodeName)
                self.available_robots_list.add(option)
                i += 1 
        except:
            import traceback
            print_div(traceback.format_exc())


async def client_discovery():
    # ip_plugins = 'localhost'
    ip_plugins = '192.168.50.152'
    
    try:
        # Run the client as a class to access client data in a more convenient way
        cli_discovery = ClientDiscovery(ip_plugins) 
        
    except:
        import traceback
        print_div(traceback.format_exc())
        raise

loop = RR.WebLoop()
loop.call_soon(client_discovery())