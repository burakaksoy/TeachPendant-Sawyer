from RobotRaconteur.Client import *

ip_plugins = '192.168.50.152' # plugins ip

url_plugin_discovery = 'rr+ws://' + ip_plugins + ':8896?service=Discovery'


plugin_discovery = RRN.ConnectService(url_plugin_discovery)


RobotConnectionURLs = plugin_discovery.available_robot_ConnectionURLs()
print(str(RobotConnectionURLs))

RobotNames =  plugin_discovery.available_robot_Names()
print(str(RobotNames))

RobotNodeNames =  plugin_discovery.available_robot_NodeNames()
print(str(RobotNodeNames))

'''
    Expected Output Example:
    ['rr+local:///?nodeid=88ada0ca-6275-4e64-ac17-e299671ead07&service=robot', 'rr+tcp://[fe80::7c64:bf9f:7c1d:5a9e]:58653/?nodeid=eb42bd99-6352-4784-9769-6a6ea260f558&service=robot']
    ['robot', 'robot']
    ['rp260_robot', 'sawyer_robot']
'''