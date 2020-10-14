from RobotRaconteur.Client import *
import time



def autodiscover(service_type):
    time.sleep(2)

    transportschemes = ["rr+local","rr+tcp","rrs+tcp"] 
    res=RRN.FindServiceByType(service_type,transportschemes)
    for serviceinfo2 in res:
        print(serviceinfo2.NodeID) 
        print(serviceinfo2.NodeName)
        print(serviceinfo2.Name)
        print(serviceinfo2.RootObjectType) 
        print(serviceinfo2.RootObjectImplements)
        print(serviceinfo2.ConnectionURL)
        print(serviceinfo2.ConnectionURL[0])
            
    return

# Nodes can also be searched for by “NodeID” and “NodeName” separate from services. 
# Use FindNodeByID and FindNodeByName in RobotRaconteurNode. 
# These will return the “NodeID”, “NodeName”, 
# and the possible “ConnectionURLs” without the query portion.


def autodiscover2(name):
    time.sleep(2)

    transportschemes = ["rr+local","rr+tcp","rrs+tcp"]

    res=RRN.FindNodeByName(name, transportschemes)
    for NodeInfo2 in res:
        print(NodeInfo2.NodeID) 
        print(NodeInfo2.NodeName)
        # print(NodeInfo2.Name)
        # print(NodeInfo2.RootObjectType) 
        # print(NodeInfo2.RootObjectImplements)
        print(NodeInfo2.ConnectionURL)
        print(NodeInfo2.ConnectionURL[0])
            
    return

service_type = "com.robotraconteur.robotics.robot.Robot"
autodiscover(service_type)


print("--------------")

name = "sawyer_robot"
autodiscover2(name)