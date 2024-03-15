#=======================================================================||====#
import os 
import sys
import pathlib
path_root = pathlib.Path(__file__)
sys.path.append(str(path_root.parent))

from pxr import Gf, Usd, UsdGeom, Sdf 
import omni 
import numpy as np 

## SEE : D:\z2024_1\GitHub\VarjoVehicles_Code\Extension\Vehicles\Fh16\Fh16.py
import _VehicleGuideTools 
import _OmniverseTools as OmniverseTools


#=======================================================================||====#
#=======================================================================||====#
def RigidBodyCreate(rootName, vehicleName, sceneName, parameters, show) : 
    ## X.1. 
    dims = parameters["dims"]
    offset = parameters["offset"]
    center = parameters["center"]

    ## X.1. 
    dims = Gf.Vec3f(dims[0], dims[1], dims[2]) 
    offset = Gf.Vec3f(offset[0], offset[1], offset[2]) 
    center = Gf.Vec3f(center[0], center[1], center[2]) 

    stage = omni.usd.get_context().get_stage()  
    rootPath = "/%s" % rootName 
    driveBasic = _VehicleGuideTools.PhysxVehicleDriveBasicAPICreate(stage, rootPath, center)  
    driveBasic.GroundSurface(sceneName, verticalAxis="Z", longitudinalAxis="Y") 
    driveBasic.SceneContext(sceneName, vehicleName, verticalAxis="Z", longitudinalAxis="Y")  
    driveBasic.ChassisCreate(vehicleName, dims=dims, offset=offset, show=show)   

    ## X.1. Wheels  
    wheelWidth = parameters["width"] 
    wheelRadius = parameters["radius"] 
    wheelOffset = -wheelRadius 

    x0 = parameters["x0"]
    x1 = parameters["x1"]
    y0 = parameters["y0"]
    y1 = parameters["y1"]

    wheelID = 0 
    wheelPos = Gf.Vec3f( x0,  y0, wheelOffset) 
    wheelName = rootName + "FL"
    driveBasic.WheelAdd(wheelName, wheelID, wheelRadius, wheelWidth, wheelPos)

    wheelID = 1 
    wheelPos = Gf.Vec3f(-x0,  y0, wheelOffset)  
    wheelName = rootName + "FR"
    driveBasic.WheelAdd(wheelName, wheelID, wheelRadius, wheelWidth, wheelPos)

    wheelID = 2 
    wheelPos = Gf.Vec3f( x1, y1, wheelOffset)  
    wheelName = rootName + "RL"
    driveBasic.WheelAdd(wheelName, wheelID, wheelRadius, wheelWidth, wheelPos)

    wheelID = 3 
    wheelPos = Gf.Vec3f(-x1, y1, wheelOffset)  
    wheelName = rootName + "RR"
    driveBasic.WheelAdd(wheelName, wheelID, wheelRadius, wheelWidth, wheelPos)

    driveBasic.WheelsCreate(rootName+"Wheel", normalAxis="X", show=show) 
    return driveBasic 


#=======================================================================||====#
#=======================================================================||====#
def ParametersGet( modelPath ) : 
    #size = OmniverseTools.GetSize( modelPath ) 
    #midpoint = OmniverseTools.GetMidpoint( modelPath ) 

    size,midpoint = OmniverseTools.compute_path_bbox( modelPath ) 
    print( size, midpoint )

    return 



def ModelParameters() : 
    parameters = {} 

    ## X.1. 
    bodyPath = "/World/Car/Model/Body" 
    dims,center = OmniverseTools.compute_path_bbox( bodyPath ) 
    print("Body:", dims, center )

    parameters["dims"] = dims * np.array([0.8,0.9,0.9])
    parameters["offset"] = np.zeros(3)

    ## X.1. 
    wheelPaths = {}
    wheelPaths["WRL"] = "/World/Car/Model/W_RearLeft" 
    wheelPaths["WRR"] = "/World/Car/Model/W_RearRight" 
    wheelPaths["WFL"] = "/World/Car/Model/W_FrontLeft" 
    wheelPaths["WFR"] = "/World/Car/Model/W_FrontRight" 

    size = np.average([OmniverseTools.GetSize(p) for n,p in wheelPaths.items()],axis=0)
    print("WheelDims:", size)
    parameters["width"] = size[0] 
    parameters["radius"] = size[1] * 0.5  

    midpoints = [OmniverseTools.GetMidpoint(p) for n,p in wheelPaths.items()] 
    average = np.average(midpoints,axis=0) 
    parameters["center"] = average 

    #parameters["offset"][2] = (parameters["dims"][2] + parameters["radius"] * 0.5) * 0.125 

    midpoints = [m - average for m in midpoints] 
    x0,y0,_ = np.amin(midpoints, axis=0)
    x1,y1,_ = np.amax(midpoints, axis=0) 
    parameters["x0"] = x0 
    parameters["x1"] = x1 
    parameters["y0"] = y0 
    parameters["y1"] = y1 
    return parameters



#=======================================================================||====#
#=======================================================================||====#
def Creating() : 
    parameters = ModelParameters() 

    RENDER_NAME = "G63"
    VEHICLE_NAME = RENDER_NAME + "Chassis"
    model = RigidBodyCreate(RENDER_NAME, VEHICLE_NAME, "SceneSetUp", parameters, show=True)  
    return 



#=======================================================================||====#





#=======================================================================||====#
def Create() : 
    ##USD_PATH = r"D:\GitHub\ModelsTruckFH16"
    #loaded = Vehicle.Loading(USD_PATH, run=True) 
    created = Creating() 
    #keyBoard = Vehicle.Controlling(created=True, loaded=True) 
    #return keyBoard
    return 





print("\n\nRunning:'%s'... " % path_root.name) 
#=======================================================================||====#
#=======================================================================||====#