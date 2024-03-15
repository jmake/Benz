import numpy as np 
import typing
import sys


from pxr import Gf, Usd, UsdGeom, Sdf 
from pxr import UsdPhysics, PhysxSchema
from pxr import UsdLux

import omni 
#import omni.kit.viewport.utility as vp_utils


#================================================================================================================||====#
def CheckMembers(module) : 
    from inspect import getmembers
    for m in getmembers(module) : print(m)
    return 


#================================================================================================================||====#
def FileInfoGet(fileName, show=True) : 
    import sys
    import pathlib
    path_root = pathlib.Path(fileName)
    if show : print("'%s' (%s) ... " % (path_root.name, path_root.parent)) 
    return path_root 


def PrimRemoveAtPath(path) : 
    stage = omni.usd.get_context().get_stage() 
    prim = stage.GetPrimAtPath(path)
    if prim is None : 
        pass 
    else : 
        stage.RemovePrim(path) 
    return 


def RemoveAll() : 
    stage = omni.usd.get_context().get_stage() 
    for prim in stage.Traverse() : 
        print( prim )

    assert False 
    return 

#================================================================================================================||====#
#================================================================================================================||====#
def PrimFindPaths() : 
    import os
    stage = omni.usd.get_context().get_stage()

    dic = {} 
    for prim in stage.Traverse() :
        _,tail = os.path.split( str(prim.GetPath()) ) 
        dic[tail] = str(prim.GetPath()) 

    return dic 


def PrimGetPath(prinName) : 
    dic = PrimFindPaths() 
    if dic.get(prinName) : return dic.get(prinName)
    return  


#================================================================================================================||====#
#================================================================================================================||====#
def AutoFocus(pathPrim, zoom, camPath="/OmniverseKit_Persp") : 
    import omni.kit.viewport.utility as vp_utils

    resolution = vp_utils.get_active_viewport().resolution
    omni.kit.commands.execute(
        'FramePrimsCommand',
        prim_to_move=camPath,
        prims_to_frame=pathPrim, 
        time_code=Usd.TimeCode.Default(),
        aspect_ratio=resolution[0]/resolution[1],
        zoom=zoom
    )    


def CameraSet(cameraName="/OmniverseKit_Persp") : 
    ## X.1. 
    cameraPath = PrimGetPath(cameraName)

    ## X.1. "/OmniverseKit_Persp" 
    viewport = omni.kit.viewport.utility.get_active_viewport()
    print("[CameraSet] '%s' -> '%s' " % (viewport.camera_path, cameraPath)) 

    viewport.camera_path = cameraPath
    return 


def CameraCreate(parentPath, cameraName, props) : 
    ## X.1. 
    stage = omni.usd.get_context().get_stage()

    prim = stage.GetPrimAtPath(parentPath)
    cam_path = prim.GetPath().AppendPath(cameraName)
    stage.RemovePrim(cam_path) 

    ## X.1. 
    usd_camera = UsdGeom.Camera.Define(stage, cam_path)
    usd_camera.CreateProjectionAttr().Set(UsdGeom.Tokens.perspective) 

    usd_camera.GetFStopAttr().Set( props.get("fstop",0.0) ) 
    usd_camera.GetFocalLengthAttr().Set( props.get("focalLength",6.0) ) 
    usd_camera.GetFocusDistanceAttr().Set( props.get("focusDistance",0.0) ) 
    usd_camera.GetClippingRangeAttr().Set( props.get("clippingRange", (0.1,100.0))) 

    ## X.1. 
    viewport_window = omni.kit.viewport.utility.get_active_viewport_window()
    viewport = omni.kit.viewport.utility.get_active_viewport()
    print("[CameraCreate]", prim, cam_path, viewport.camera_path , viewport_window)
    viewport.camera_path = cam_path

    ## X.1. 
    usdGeomXform = UsdGeom.Xform.Get(stage, cam_path)
    usdGeomXform.AddTranslateOp().Set(Gf.Vec3f(0.0))    
    usdGeomXform.AddRotateXYZOp().Set(Gf.Vec3f(0.0)) 
    usdGeomXform.AddScaleOp().Set(Gf.Vec3f(1.0))  

    parent = stage.GetPrimAtPath(cam_path)
    #parent.GetAttribute('xformOp:translate').Set( Gf.Vec3f(0.0,0.0,60.0) ) 

    ##stage.SetDefaultPrim(prim) ??
    return cam_path


def CameraGet(cameraName) :
    stage = omni.usd.get_context().get_stage()
    cameraPath = OmniverseTools.PrimGetPath(cameraName) 
    return UsdGeom.Camera.Get(stage, cameraPath)


#================================================================================================================||====#
def LidarCreate(parentPath, lidarName) :
    ## X.1. 
    stage = omni.usd.get_context().get_stage()

    prim = stage.GetPrimAtPath(parentPath)
    #lidarPath = prim.GetPath().AppendPath(lidarName)
    #stage.RemovePrim(prim.GetPath()) 

    ## X.1.
    result, prim = omni.kit.commands.execute(
                "RangeSensorCreateLidar",
                path=lidarName,
                parent=parentPath, 
                min_range=0.4,
                max_range=100.0,
                draw_points=False,
                draw_lines=True,
                horizontal_fov=360.0,
                vertical_fov=30.0,
                horizontal_resolution=0.4,
                vertical_resolution=4.0,
                rotation_rate=0.0,
                high_lod=False,
                yaw_offset=0.0,
                enable_semantics=False
            )

    return prim.GetPath()


#================================================================================================================||====#
def Transforms(path, scale=Gf.Vec3f(1.0), translate=Gf.Vec3f(0.0,0.0,0.0), rotateXYZ=Gf.Vec3f(0.0,0.0,0.0)) : 
    stage = omni.usd.get_context().get_stage()

    parent = stage.GetPrimAtPath(path)
    try : parent.GetAttribute('xformOp:scale').Set(scale)         #  Gf.Vec3f(0.02) )   
    except : pass

    try : parent.GetAttribute('xformOp:translate').Set(translate) #  Gf.Vec3f(0.0,0.0,0.3) ) 
    except : pass 

    try : parent.GetAttribute('xformOp:rotateXYZ').Set(rotateXYZ) #  Gf.Vec3f(0.0,0.0,0.0) )    
    except : pass 

    return 


def XformPropertiesGet(path, prop) : 
    stage = omni.usd.get_context().get_stage()
    usdGeomXform = stage.GetPrimAtPath(path)
    return usdGeomXform.GetAttribute('xformOp:%s' % prop).Get() 


def XformPropertiesCreate(path, prop) : 
    stage = omni.usd.get_context().get_stage()
    usdGeomXform = stage.GetPrimAtPath(path)
    usdGeomXform.CreateAttribute('xformOp:%s' % prop, Sdf.ValueTypeNames.Float3)
    return 


def XformPropertiesSet(path, prop, value, end=True) : 
    stage = omni.usd.get_context().get_stage()
    usdGeomXform = stage.GetPrimAtPath(path) 

    attribute = 'xformOp:%s' % prop
    if not usdGeomXform.HasAttribute(attribute) : 
        print("[XformPropertiesSet] '%s' no found!" % attribute)     
        print("[XformPropertiesSet] '%s'" % path)     
        if end : exit() 
        return 

    usdGeomXform.GetAttribute(attribute).Set(value)
    return 



#================================================================================================================||====#
def compute_path_bbox(path) :
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(path)

    imageable = UsdGeom.Imageable(prim)
    time = Usd.TimeCode.Default() # The time at which we compute the bounding box
    bound = imageable.ComputeWorldBound(time, UsdGeom.Tokens.default_)
    bound_range = bound.ComputeAlignedBox()

    size = bound_range.GetSize() 
    mid = bound_range.GetMidpoint()

    return np.array(size), np.array(mid)  


def GetSize(path) : 
    size, midpoint = compute_path_bbox(path)
    return np.array(size) 


def GetMidpoint(path) :
    size, midpoint = compute_path_bbox(path)
    return np.array(midpoint)  


#================================================================================================================||====#
def PhysicsSetUp(path, approximation) :
    stage = omni.usd.get_context().get_stage()
    parent = stage.GetPrimAtPath(path)
    rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(parent)

    nChildren = len(parent.GetChildren())
    print("PhysicsSetUp: %d " % nChildren) 

    if nChildren > 1 : 
        children = [ip for ip in parent.GetChildren()]; 
        children = [c for c in children if "Mesh" in  c.GetTypeName()]; 

        MassAPIs = [UsdPhysics.MassAPI.Apply(c) for c in children]
        collisionAPIs = [UsdPhysics.CollisionAPI.Apply(c) for c in children]

        for c in children : 
            mehCollisionAPI = UsdPhysics.MeshCollisionAPI.Get(stage,c.GetPrimPath())  ## [4] UsdPhysicsMeshCollisionAPI
            if mehCollisionAPI.GetApproximationAttr().Get() is None : 
                mehCollisionAPI.Apply(c)  
                mehCollisionAPI.CreateApproximationAttr(approximation) # ("meshSimplification")  
            mehCollisionAPI.GetApproximationAttr().Set(approximation) # ("sdf")
            PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(c)                             ## [5] PhysxSchemaPhysxSDFMeshCollisionAPI 
    else : 
        UsdPhysics.MassAPI.Apply(parent)

        UsdPhysics.CollisionAPI.Apply(parent)
        mehCollisionAPI = UsdPhysics.MeshCollisionAPI.Get(stage,parent.GetPrimPath())  ## [4] UsdPhysicsMeshCollisionAPI
        if mehCollisionAPI.GetApproximationAttr().Get() is None : 
            mehCollisionAPI.Apply(parent)  
            mehCollisionAPI.CreateApproximationAttr(approximation) # ("meshSimplification")  
        else : mehCollisionAPI.GetApproximationAttr().Set(approximation) # ("meshSimplification")
        print("[PhysicsAddSDF] '%s' " % mehCollisionAPI.GetApproximationAttr().Get() ) 
    return  



def UsdGeomXformCreate(path, stage, usdGeomXform) :
    try : usdGeomXform.AddTranslateOp().Set(Gf.Vec3f(0.0)) 
    except : pass 
    
    try : usdGeomXform.AddRotateXYZOp().Set(Gf.Vec3f(1.0))
    except : pass 
    
    try : usdGeomXform.AddScaleOp().Set(Gf.Vec3f(1.0)) 
    except : pass 

    #print(  usdGeomXform.GetSchemaAttributeNames() )
    return stage.GetPrimAtPath(path)


def CubeCreate(path) :
    stage = omni.usd.get_context().get_stage()
    stage.RemovePrim(path) 

    usdGeomXform = UsdGeom.Cube.Define(stage, path)

    extent = UsdGeom.Cube.ComputeExtentFromPlugins(usdGeomXform, 0)
    usdGeomXform.CreateExtentAttr(extent) 

    return UsdGeomXformCreate(path, stage, usdGeomXform)


def CubeSize(path, size) : 
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(path)

    usdGeomXform = UsdGeom.Cube.Get(stage, path)
    usdGeomXform.GetSizeAttr().Set(size) 
    usdGeomXform.CreateDisplayColorAttr([Gf.Vec3f(0.1)])

    extent = UsdGeom.Cube.ComputeExtentFromPlugins(usdGeomXform, 0)
    usdGeomXform.CreateExtentAttr(extent) 

    extent = usdGeomXform.GetExtentAttr() 
    print("[CubeSize] ", extent.Get()) 
    return 


def CylinderCreate(path, axis="Z") :
    stage = omni.usd.get_context().get_stage()
    stage.RemovePrim(path) 

    usdGeomXform = UsdGeom.Cylinder.Define(stage, path)
    usdGeomXform.GetAxisAttr().Set(axis)

    extent = UsdGeom.Cylinder.ComputeExtentFromPlugins(usdGeomXform, 0)
    usdGeomXform.CreateExtentAttr(extent) 

    return UsdGeomXformCreate(path, stage, usdGeomXform)


def XformCreate(path, translate) : 
    stage = omni.usd.get_context().get_stage()
    stage.RemovePrim(path) 

    usdGeomXform = UsdGeom.Xform.Define(stage, path)
    usdGeomXform.AddTranslateOp().Set( translate ) 
    usdGeomXform.AddRotateXYZOp().Set( Gf.Vec3f(0.0) ) 
    usdGeomXform.AddScaleOp().Set( Gf.Vec3f(1.0) )  

    ##prim = stage.GetPrimAtPath(path)
    return usdGeomXform


#================================================================================================================||====#
def SceneGetGravity(path) :  
    stage = omni.usd.get_context().get_stage()
    scene = UsdPhysics.Scene.Define(stage, path) 
    print("[SceneGetGravity] g='%s' '%s' " % (scene.GetGravityMagnitudeAttr().Get(), scene.GetGravityDirectionAttr().Get()) ) 
    return scene.GetGravityMagnitudeAttr().Get() 


def SceneGetGravityDirectionAttr(path) :  
    stage = omni.usd.get_context().get_stage()
    scene = UsdPhysics.Scene.Define(stage, path) 
    return scene.GetGravityDirectionAttr().Get() 


def CylinderSize(path, height, radius, axis) : 
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(path)

    usdGeomXform = UsdGeom.Cylinder.Get(stage, path)
    usdGeomXform.GetRadiusAttr().Set(radius) 
    usdGeomXform.GetHeightAttr().Set(height) 
    usdGeomXform.GetAxisAttr().Set(axis)
    usdGeomXform.CreateDisplayColorAttr([Gf.Vec3f(0.3, 0.65, 1)]) 

    cylExtent = UsdGeom.Cylinder.ComputeExtentFromPlugins(usdGeomXform, 0)
    usdGeomXform.CreateExtentAttr(cylExtent) 

    extent = usdGeomXform.GetExtentAttr() 
    print("[CylinderSize] ", extent.Get())
    return 


def CylinderPropertiesGet(path) : 
    stage = omni.usd.get_context().get_stage()

    usdGeomXform = UsdGeom.Cylinder.Get(stage, path)
    r = usdGeomXform.GetRadiusAttr().Get()  
    h = usdGeomXform.GetHeightAttr().Get()  
    a = usdGeomXform.GetAxisAttr().Get() 
    return r,h,a 


def CylinderColor(path, color) : 
    stage = omni.usd.get_context().get_stage()
    usdGeomXform = UsdGeom.Cylinder.Get(stage, path) 
    usdGeomXform.GetDisplayColorAttr().Set([Gf.Vec3f(color[0],color[1],color[2])]) 
    return 


#================================================================================================================||====#
def get_world_transform_xform(path) -> typing.Tuple[Gf.Vec3d, Gf.Rotation, Gf.Vec3d]:
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(path)

    xform = UsdGeom.Xformable(prim)
    time = Usd.TimeCode.Default() # The time at which we compute the bounding box
    world_transform: Gf.Matrix4d = xform.ComputeLocalToWorldTransform(time)
    translation: Gf.Vec3d = world_transform.ExtractTranslation()
    rotation: Gf.Rotation = world_transform.ExtractRotation()
    scale: Gf.Vec3d = Gf.Vec3d(*(v.GetLength() for v in world_transform.ExtractRotationMatrix()))
    return translation, rotation, scale


#================================================================================================================||====#
def WheelCreate(path, translate, scale) : 
    CylinderCreate(path,"X") 
    PhysicsSetUp(path)
    Transforms(path, translate=translate, scale=scale)

    stage = omni.usd.get_context().get_stage()
    return stage.GetPrimAtPath(path) 


def DifferentialCreate(path, translate, scale) : 
    CubeCreate(path)
    PhysicsSetUp(path)
    Transforms(path, translate=translate, scale=scale)

    stage = omni.usd.get_context().get_stage()
    return stage.GetPrimAtPath(path) 


#================================================================================================================||====#
def DriverAngularCreate(path, velocity) : 
    stage = omni.usd.get_context().get_stage()

    angularDriveAPI = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(path), "angular")
    angularDriveAPI.CreateTypeAttr("force")
    angularDriveAPI.CreateMaxForceAttr(  1000000.0)
    angularDriveAPI.CreateDampingAttr(   1000000.0)
    angularDriveAPI.CreateStiffnessAttr( 1000000.0)
    angularDriveAPI.CreateTargetVelocityAttr(velocity)


def DriverModifier(drivePath, driveType, velocity=0.0, position=0.0) : 
    stage = omni.usd.get_context().get_stage()

    prim = stage.GetPrimAtPath(drivePath)
    drive = pxr.UsdPhysics.DriveAPI.Get(prim,driveType)
    drive.GetTargetVelocityAttr().Set(velocity) 
    drive.GetTargetPositionAttr().Set(position)
    return 


#================================================================================================================||====#
def JointCreate(fixedJoint, bodyPath0=None, bodyPath1=None, Enabled=True) : 
    C01 = Gf.Vec3f(0.0) 

    if bodyPath0 is not None: 
        fixedJoint.CreateBody0Rel().SetTargets([bodyPath0])

        bodyPos0 = GetMidpoint(bodyPath0)
        t0,r0,s0 = get_world_transform_xform(bodyPath0)
        print("[JointCreate00] '%s' "% bodyPath0, bodyPos0, "/", s0, "=", bodyPos0/s0) 

    if bodyPath1 is not None: 
        fixedJoint.CreateBody1Rel().SetTargets([bodyPath1])

        bodyPos1 = GetMidpoint(bodyPath1)
        t1,r1,s1 = get_world_transform_xform(bodyPath1)
        print("[JointCreate11] '%s' "% bodyPath1, bodyPos1, "/", s1, "=", bodyPos1/s1) 

        #C01 = bodyPos1 - bodyPos0; C01 /= s0 
        #C01 = bodyPos1/s1 - bodyPos0/s0; 
        #C01 = bodyPos1 - bodyPos0/s0*s1; 
        C01 = bodyPos1 - bodyPos0; # ??
        C01 = Gf.Vec3f(C01[0], C01[1], C01[2])

    print("[JointCreate01] C01:", C01) 

    fixedJoint.GetLocalPos0Attr().Set(C01)
    fixedJoint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0))

    #fixedJoint.CreateLocalPos0Attr().Set(C01)
    #fixedJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0)) 

    #fixedJoint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0)) 
    fixedJoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0)) 

    fixedJoint.GetJointEnabledAttr().Set(Enabled)  
    return fixedJoint


def JointFixedCreate(path, bodyPath0=None, bodyPath1=None, Enabled=True) : 
    stage = omni.usd.get_context().get_stage()
    stage.RemovePrim(path)

    fixedJoint = UsdPhysics.FixedJoint.Define(stage, path)
    JointCreate(fixedJoint, bodyPath0, bodyPath1, Enabled) 
    return 


def JoinRevoluteCreate(path, bodyPath0, bodyPath1, axis="Z", Enabled=True) : 
    stage = omni.usd.get_context().get_stage()
    stage.RemovePrim(path)

    revoluteJoint = UsdPhysics.RevoluteJoint.Define(stage, path)
    ##revoluteJoint.CreateLowerLimitAttr(-90.0)
    ##revoluteJoint.CreateUpperLimitAttr(90.0)
    revoluteJoint.CreateAxisAttr(axis)
    revoluteJoint.CreateBreakForceAttr().Set( sys.float_info.max) 
    revoluteJoint.CreateBreakTorqueAttr().Set(sys.float_info.max) 

    JointCreate(revoluteJoint, bodyPath0, bodyPath1, Enabled) 
    return 


#================================================================================================================||====#
#================================================================================================================||====#
"""
camera2 = CameraCreate("/Bc", "Camera2")
Transforms(camera2, translate=Gf.Vec3f(-10.0,0.0,25.0), rotateXYZ=Gf.Vec3f(80.0,0.0,-90.0))

lidar1 = LidarCreate("/Bc", "/Lidar") 
Transforms(lidar1, translate=Gf.Vec3f(0.0,0.0,1.5))

"""

#================================================================================================================||====#
#================================================================================================================||====#
import pxr 
import carb
from carb.input import KeyboardEventType
from carb.input import GamepadInput


#================================================================================================================||====#
class GamePadController :

    def __init__(self) :
        self.Inputs = {} 
        self.connection = carb.input.GamepadConnectionEventType(0)
        print("[GamePadController] ", self.connection)
        return 


    def End(self) : 
        self.Inputs.clear() 

        input = carb.input.acquire_input_interface()
        input.unsubscribe_to_gamepad_events(self.gamepad, self.gpid)
        print("[PadControllerEnd] gpid '%d' Removed:" % self.gpid)
        return  


    def Init(self, function) : 
        self.function = function 

        appwindow = omni.appwindow.get_default_app_window()
        self.gamepad = appwindow.get_gamepad(0)

        input = carb.input.acquire_input_interface()
        self.gpid = input.subscribe_to_gamepad_events(self.gamepad, self.OnInput)

        print("[PadControllerInit] gpid '%d' Created:" % self.gpid) 
        return  


    def OnInput(self, e) :
        self.Input = e.input
        self.Value = e.value
        self.Inputs[e.input] = e.value 

        self.function(self.Inputs) 
        print("[GamepadOnInput] {} ({})".format(e.input, e.value))
        return 


    def LeftStickGet(self, threshold=0.5) : 
        ## ?? if len(self.gpDriver.Inputs) < 1 : return 
        fortward = self.Inputs.get(GamepadInput.LEFT_STICK_UP,    0.0) > threshold  
        backward = self.Inputs.get(GamepadInput.LEFT_STICK_DOWN,  0.0) > threshold  
        left     = self.Inputs.get(GamepadInput.LEFT_STICK_LEFT,  0.0) > threshold  
        right    = self.Inputs.get(GamepadInput.LEFT_STICK_RIGHT, 0.0) > threshold 
        #print("[LeftStickGet]", fortward, backward, left, right ) 
        return fortward, backward, left, right


    def RightStickGet(self, threshold=0.5) : 
        ## ?? if len(self.gpDriver.Inputs) < 1 : return 

        fortward = self.Inputs.get(GamepadInput.RIGHT_STICK_UP,    0.0) > threshold  
        backward = self.Inputs.get(GamepadInput.RIGHT_STICK_DOWN,  0.0) > threshold  
        left     = self.Inputs.get(GamepadInput.RIGHT_STICK_LEFT,  0.0) > threshold  
        right    = self.Inputs.get(GamepadInput.RIGHT_STICK_RIGHT, 0.0) > threshold 
        #print("[RightStickGet]", fortward, backward, left, right ) 
        return fortward, backward, left, right




#================================================================================================================||====#
class KeyBoardController :

    def __init__(self) :
        self.UP = 0.0
        self.DOWN = 0.0 
        self.LEFT = 0.0
        self.RIGHT = 0.0 
        self.LEFT_SHIFT = 0.0 
        self.LEFT_CONTROL = 0.0 
        self.Inputs = {} 
        self.input = None 
        return 


    def End(self) : 
        self.Inputs.clear() 

        input = carb.input.acquire_input_interface()
        input.unsubscribe_to_keyboard_events(self.keyboard, self.kbid)
        print("[KeyBoardEnd] kbID '%d' Removed:" % self.kbid)
        return  


    def Init(self, function) : 
        self.function = function 

        appwindow = omni.appwindow.get_default_app_window()
        self.keyboard = appwindow.get_keyboard()

        input = carb.input.acquire_input_interface()
        self.kbid = input.subscribe_to_keyboard_events(self.keyboard, self.OnInput)
        
        print("[KeyBoardInit] kbID '%d' Created:" % self.kbid)
        return  


    def OnInput(self, e) :
        if e.input == carb.input.KeyboardInput.UP    : self.UP    = self.OnPressRelease(e) 
        if e.input == carb.input.KeyboardInput.DOWN  : self.DOWN  = self.OnPressRelease(e) 
        if e.input == carb.input.KeyboardInput.LEFT  : self.LEFT  = self.OnPressRelease(e) 
        if e.input == carb.input.KeyboardInput.RIGHT : self.RIGHT = self.OnPressRelease(e) 
        if e.input == carb.input.KeyboardInput.LEFT_SHIFT  : self.LEFT_SHIFT  = self.OnPressRelease(e) 
        if e.input == carb.input.KeyboardInput.LEFT_CONTROL : self.LEFT_CONTROL = self.OnPressRelease(e) 

        #self.input = (e.input, e.type)
        #self.Inputs[e.input] = self.OnPressRelease(e)  # ??  
        if not e.type == KeyboardEventType.CHAR : self.Inputs[ str(e.input) ] = self.OnPressRelease(e)  

        self.function(self.Inputs) 
        #print("[KeyBoardController] {} ({})".format(e.input, e.type)) 
        return 


    def OnPressRelease(self, e) : 
        if e.type == KeyboardEventType.KEY_PRESS or e.type == KeyboardEventType.KEY_REPEAT :
            return 1.0 
        elif e.type == KeyboardEventType.KEY_RELEASE :
            return 0.0 


#================================================================================================================||====#
#================================================================================================================||====#

def JointFixedCreate(path, bodyPath0=None, bodyPath1=None) : 
    stage = omni.usd.get_context().get_stage()
    stage.RemovePrim(path)

    fixedJoint = UsdPhysics.FixedJoint.Define(stage, path)
    JointCreate(fixedJoint, bodyPath0, bodyPath1) 
    return 


def JoinsCreateFixed(joins) : 
    for (key,value) in joins.items() : 
        b0 = PrimGetPath(value[0]) 
        b1 = PrimGetPath(value[1]) 
        JointFixedCreate("/"+key, b0, b1) 
    return 


#================================================================================================================||====#
def JointPrismaticJoint(path, bodyPath0=None, bodyPath1=None, 
                        axis=None, 
                        limits=[None,None], Enabled=True) : 
    stage = omni.usd.get_context().get_stage()
    stage.RemovePrim(path)

    prismaticJoint = UsdPhysics.PrismaticJoint.Define(stage, path)
    prismaticJoint.CreateBreakForceAttr().Set( sys.float_info.max) 
    prismaticJoint.CreateBreakTorqueAttr().Set(sys.float_info.max) 

    prismaticJoint.CreateAxisAttr(axis)
    prismaticJoint.CreateLowerLimitAttr(limits[0])
    prismaticJoint.CreateUpperLimitAttr(limits[1])

    JointCreate(prismaticJoint, bodyPath0, bodyPath1, Enabled) 
    return 


def JoinRevoluteCreate(path, bodyPath0, bodyPath1, axis=None, limits=[None,None], Enabled=True) : 
    stage = omni.usd.get_context().get_stage()
    stage.RemovePrim(path)

    revoluteJoint = UsdPhysics.RevoluteJoint.Define(stage, path)
    revoluteJoint.CreateBreakForceAttr().Set( sys.float_info.max) 
    revoluteJoint.CreateBreakTorqueAttr().Set(sys.float_info.max) 

    revoluteJoint.CreateAxisAttr(axis)
    revoluteJoint.CreateLowerLimitAttr(limits[0])
    revoluteJoint.CreateUpperLimitAttr(limits[1])    

    JointCreate(revoluteJoint, bodyPath0, bodyPath1, Enabled) 
    return 



def JoinsCreate(joins, Func, schema, axis, limits, driver) : 
    for (key,value) in joins.items() : 
        b0 = PrimGetPath(value[0]) 
        b1 = PrimGetPath(value[1]) 
        Func("/"+key, b0, b1, axis=axis, limits=limits) 
        if driver : DriverSetUp("/"+key, schema, velocity=0.0, position=0.0) 
    return 


def JointD6(path, bodyPath0=None, bodyPath1=None) : 
    stage = omni.usd.get_context().get_stage()
    stage.RemovePrim(path)

    joint = UsdPhysics.Joint.Define(stage, path)
    joint.CreateBreakForceAttr().Set( sys.float_info.max) 
    joint.CreateBreakTorqueAttr().Set(sys.float_info.max) 

    prim = stage.GetPrimAtPath(path)

    limitAPI = UsdPhysics.LimitAPI.Apply(prim, UsdPhysics.Tokens.transX)
    limitAPI.CreateLowAttr().Set(0.0)
    limitAPI.CreateHighAttr().Set(0.0)

    limitAPI = UsdPhysics.LimitAPI.Apply(prim, UsdPhysics.Tokens.transY)
    limitAPI.CreateLowAttr().Set(0.0)
    limitAPI.CreateHighAttr().Set(0.0)

    limitAPI = UsdPhysics.LimitAPI.Apply(prim, UsdPhysics.Tokens.transZ)
    limitAPI.CreateLowAttr().Set(0.0)
    limitAPI.CreateHighAttr().Set(0.0)

    limitAPI = UsdPhysics.LimitAPI.Apply(prim, UsdPhysics.Tokens.rotX)
    limitAPI.CreateLowAttr().Set(0.0)
    limitAPI.CreateHighAttr().Set(0.0)

    limitAPI = UsdPhysics.LimitAPI.Apply(prim, UsdPhysics.Tokens.rotY)
    limitAPI.CreateLowAttr().Set(0.0)
    limitAPI.CreateHighAttr().Set(0.0)

    limitAPI = UsdPhysics.LimitAPI.Apply(prim, UsdPhysics.Tokens.rotZ)
    limitAPI.CreateLowAttr().Set(0.0)
    limitAPI.CreateHighAttr().Set(0.0)

    JointCreate(joint, bodyPath0, bodyPath1) 
    return 


def JointsD6Create(joins) : 
    for (key,value) in joins.items() : 
        b0 = PrimGetPath(value[0]) 
        b1 = PrimGetPath(value[1]) 
        JointD6("/"+key, b0, b1) 
    return 


def LimitAPISet(primName,
                axis, 
                limits=[-sys.float_info.max, sys.float_info.max]) : 
    stage = omni.usd.get_context().get_stage()
    path = PrimGetPath(primName) 
    prim = stage.GetPrimAtPath(path)

    Token = {} 
    Token["transX"] = UsdPhysics.Tokens.transX 
    Token["transY"] = UsdPhysics.Tokens.transY 
    Token["transZ"] = UsdPhysics.Tokens.transZ 

    Token["rotX"] = UsdPhysics.Tokens.rotX 
    Token["rotY"] = UsdPhysics.Tokens.rotY 
    Token["rotZ"] = UsdPhysics.Tokens.rotZ 

    limitAPI = UsdPhysics.LimitAPI.Get(prim, Token.get(axis))
    limitAPI.CreateLowAttr().Set(limits[0])
    limitAPI.CreateHighAttr().Set(limits[1])


#================================================================================================================||====#
def DriverCreate(path, schema) : 
    ## schemas :  transX , transY , transZ , rotX , rotY , rotZ , linear, angular
    stage = omni.usd.get_context().get_stage()

    driveAPI = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(path), schema)
    driveAPI.CreateTypeAttr("force")
    #driveAPI.CreateMaxForceAttr(  100000000.0)
    driveAPI.CreateDampingAttr(   1e6 )
    driveAPI.CreateStiffnessAttr( 1e6 )
    return driveAPI


def DriverSetUp(path, schema, velocity, position) : 
    driver = DriverCreate(path, schema) 
    driver.CreateTargetVelocityAttr(velocity)
    driver.CreateTargetPositionAttr(position)
    return 



def DriverModify(path, schema, velocity, position) : 
    stage = omni.usd.get_context().get_stage()

    driver = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(path), schema)
    driver.CreateTargetVelocityAttr(velocity)
    driver.CreateTargetPositionAttr(position)


#================================================================================================================||====#
def JointPrismaticJoint(path, bodyPath0=None, bodyPath1=None, 
                        axis=None, 
                        limits=[None,None], Enabled=True) : 
    stage = omni.usd.get_context().get_stage()
    stage.RemovePrim(path)

    prismaticJoint = UsdPhysics.PrismaticJoint.Define(stage, path)
    prismaticJoint.CreateBreakForceAttr().Set( sys.float_info.max) 
    prismaticJoint.CreateBreakTorqueAttr().Set(sys.float_info.max) 

    prismaticJoint.CreateAxisAttr(axis)
    prismaticJoint.CreateLowerLimitAttr(limits[0])
    prismaticJoint.CreateUpperLimitAttr(limits[1])

    JointCreate(prismaticJoint, bodyPath0, bodyPath1, Enabled) 
    return 



#================================================================================================================||====#
def InvisibleSet(primName) : 
    ## https://openusd.org/dev/api/usd_2usd_geom_2tokens_8h_source.html
    stage = omni.usd.get_context().get_stage()
    path = PrimGetPath(primName) 

    collider = UsdGeom.Xform.Get(stage, path)
    collider.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    return 


#================================================================================================================||====#
def SceneGetGravity(path) :  
    stage = omni.usd.get_context().get_stage()
    scene = UsdPhysics.Scene.Define(stage, path) 
    print("[SceneGetGravity] g='%s' " % scene.GetGravityMagnitudeAttr().Get() ) 
    return scene.GetGravityMagnitudeAttr().Get() 


def StageUpAxisSet(axis) : 
    stage = omni.usd.get_context().get_stage()
    ##if axis == "X" : UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.x)
    if axis == "Y" : UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    if axis == "Z" : UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    return 

#================================================================================================================||====#



#================================================================================================================||====#
def VisibilitySet(primName, visibility) : 
    ## https://openusd.org/dev/api/usd_2usd_geom_2tokens_8h_source.html
    stage = omni.usd.get_context().get_stage()
    path = PrimGetPath(primName) 

    collider = UsdGeom.Xform.Get(stage, path)
    collider.CreateVisibilityAttr(visibility) 
    return 


def InvisibleSet(primName) : 
    return VisibilitySet(primName, UsdGeom.Tokens.invisible) 


def VisibleSet(primName) : 
    return VisibilitySet(primName, UsdGeom.Tokens.inherited) 


#================================================================================================================||====#
def PrimDuplicate(nameFrom, pathTo) : 
    import omni 
    stage = omni.usd.get_context().get_stage()
    primTo = stage.GetPrimAtPath(pathTo) 
    if primTo.IsValid() : 
        print("[PrimDuplicate] '%s' exist!!" % primTo) 

        children = primTo.GetChildren()
        for c in children : stage.RemovePrim(c.GetPath())  
        stage.RemovePrim(primTo.GetPath())
        #return 
    else : 
        print("[PrimDuplicate] '%s' copying ..." % pathTo) 

    pathFrom = PrimGetPath(nameFrom) 

    omni.kit.commands.execute('CopyPrim',
        path_from=pathFrom,
        path_to=pathTo,
        exclusive_select=False,
        copy_to_introducing_layer=False)

    primTo = stage.GetPrimAtPath(pathTo) 
    primFrom = stage.GetPrimAtPath(pathFrom) 
    print("[PrimDuplicate] '%s' -> '%s' " % (primFrom.GetName(), primTo.GetName())) 
    return 


#================================================================================================================||====#
def PrimRemove(primName) : 
    import omni 
    path = PrimGetPath(primName) 
    print("[PrimRemove] '%s' " % path ) 
    if path is None : return 

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid() : 
        print("[PrimRemove] '%s' exist, Removing..." % prim) 

        children = prim.GetChildren()
        for c in children : stage.RemovePrim(c.GetPath())  
        stage.RemovePrim(prim.GetPath()) 
        return 
    # 
    # import omni.kit.commands
    # omni.kit.commands.execute('DeletePrims', paths=[path],	destructive=False)
    #
    # if (prim.IsValid()) : print("'{}' is valid".format(prim.GetName()))
    # else : print("'{}' is not valid".format(prim.GetName()))
    # 
    print("[PrimRemove] '%s' no found!!" % path) 
    return 


#================================================================================================================||====#
import omni.usd
from pxr import Sdf
import os 


def BehaviorScriptAdd(primName, scripPath) : 
    from omni.kit.scripting import BehaviorScript ## ?? 

    isExisting = os.path.exists(scripPath) 
    assert isExisting 

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(primName)

    # Adding Component
    prim_paths = [prim.GetPath()]
    omni.kit.commands.execute("ApplyScriptingAPICommand", paths=prim_paths)
    omni.kit.commands.execute("RefreshScriptingPropertyWindowCommand")

    # Adding Script
    #PATH = [scripPath] 
    scripts = Sdf.AssetPathArray([scripPath])
    prim.GetAttribute("omni:scripting:scripts").Set(scripts)
    return 



#================================================================================================================||====#
#================================================================================================================||====#
#print("'%s' Ready..." % FileInfoGet(__file__).name) 
FileInfoGet(__file__)

#================================================================================================================||====#

def CylinderAtXform(parentPath, primName, center, dimensions, remove) :  
    wheelPrim = XformCreate(parentPath, center) 

    colliderPath = "/".join([parentPath,primName])
    CylinderCreate(colliderPath) 

    width, radius, direction = dimensions 
    CylinderSize(colliderPath, width, radius, direction)  

    stage = omni.usd.get_context().get_stage()
    return stage.GetPrimAtPath(colliderPath) 


def CubeAtXform(parentPath, primName, center, dimensions, remove) : 
    if remove : 
        vehiclePrim = XformCreate(parentPath, center) 

    colliderPath = "/".join([parentPath,primName])
    CubeCreate(colliderPath) 
    
    CubeSize(colliderPath, 1.0)  
    Transforms(colliderPath, scale=dimensions)

    stage = omni.usd.get_context().get_stage()
    return stage.GetPrimAtPath(colliderPath) 


def MassAPIGet(path, com=None, mass=None) :
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(path) 

    massApi = UsdPhysics.MassAPI.Get(stage, path) 
    if mass is not None : massApi.GetMassAttr().Set(mass)  
    if com is not None : massApi.GetCenterOfMassAttr().Set(com)  
    return 


def JointFixedGet(path) : 
    stage = omni.usd.get_context().get_stage()
    #stage.RemovePrim(path)

    fixedJoint = UsdPhysics.FixedJoint.Get(stage, path)
    print(fixedJoint) 

    localPos0 = fixedJoint.GetLocalPos0Attr() #.Get()
    localPos1 = fixedJoint.GetLocalPos1Attr().Get() 
    print(localPos0, localPos1)
    return 


#================================================================================================================||====#
def AssetLoader(primName, parentPath, modelName, translate=Gf.Vec3f(0.0), rotate=Gf.Vec3f(0.0))  : 
    assetPath = os.path.join(parentPath, modelName)
    isExisting = os.path.exists(assetPath) 
    print("[AssetLoader] '%s' " % assetPath)
    assert isExisting 

    primFork = AssetDownload(primName, assetPath) 
    print("[AssetLoader] '%s' " %primFork) 
    
    assert primFork is not None 

    OmniverseTools.Transforms(assetPath,  
                              translate=translate, 
                              rotateXYZ=rotate) 
    return primFork

    

#================================================================================================================||====#
def Euler2Quaternion(roll, pitch, yaw) : # x-axis, y-axis, z-axis
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  return Gf.Quatf(qx, qy, qz, qw)


def Parenting(parentName, childName) : 
    import omni.kit.commands
    childPath = PrimGetPath(childName) 
    parentPath = PrimGetPath(parentName) 

    omni.kit.commands.execute('ParentPrims',
        parent_path=parentPath,
        child_paths=[childPath]) # , keep_world_transform=False)
    return 


#================================================================================================================||====#
def PurposeSet(colliderName, token) : 
    colliderPath = PrimGetPath(colliderName) 

    stage = omni.usd.get_context().get_stage()
    collider = UsdGeom.Xform.Get(stage, colliderPath)
    collider.CreatePurposeAttr(token)
    return 


def PurposeSetGuide(colliderName) : 
    return PurposeSet(colliderName, UsdGeom.Tokens.guide)


def RigidBodyAPISet(name) :
    path = PrimGetPath(name) 
    stage = omni.usd.get_context().get_stage()

    parent = stage.GetPrimAtPath(path)
    UsdPhysics.RigidBodyAPI.Apply(parent) 
    return 


def CollisionAPISet(name) :
    path = PrimGetPath(name) 
    stage = omni.usd.get_context().get_stage()

    parent = stage.GetPrimAtPath(path)
    UsdPhysics.CollisionAPI.Apply(parent)
    return 


def MassAPISet(name) :
    stage = omni.usd.get_context().get_stage()

    path = PrimGetPath(name) 
    parent = stage.GetPrimAtPath(path)
    UsdPhysics.MassAPI.Apply(parent) 
    return 


#================================================================================================================||====#
def SphereLightCreate(lightPath, lightName, intensity, translate, radius=1.0, lengthScale=1.0) : 
    stage = omni.usd.get_context().get_stage()
    
    sphereLight = UsdLux.SphereLight.Define(stage, lightPath + "/%s" % lightName)
    sphereLight.CreateRadiusAttr(radius * lengthScale)
    sphereLight.CreateIntensityAttr(intensity) #(30000)
    sphereLight.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(translate * lengthScale)
    return 


def DomeLightCreate(lightPath, lightName, intensity, translate, exposure=0.0, lengthScale=1.0) : 
    stage = omni.usd.get_context().get_stage()
    
    sphereLight = UsdLux.DomeLight.Define(stage, lightPath + "/%s" % lightName)
    sphereLight.CreateIntensityAttr(intensity) 
    sphereLight.CreateExposureAttr(exposure) 
    sphereLight.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(translate * lengthScale)
    return 


def DistantLightCreate(lightPath, lightName, intensity, translate, lengthScale=1.0) : 
    stage = omni.usd.get_context().get_stage()
    
    sphereLight = UsdLux.DistantLight.Define(stage, lightPath + "/%s" % lightName)
    sphereLight.CreateIntensityAttr(intensity) 
    sphereLight.CreateExposureAttr(10.0) 
    sphereLight.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(translate * lengthScale)
    return 


def RectLightCreate(lightPath, lightName, 
                    width, height, 
                    translate, 
                    rotate, 
                    intensity, 
                    exposure, 
                    lengthScale=1.0) : 
    stage = omni.usd.get_context().get_stage()
    
    light = UsdLux.RectLight.Define(stage, lightPath + "/%s" % lightName)
    light.CreateIntensityAttr(intensity) 
    light.CreateExposureAttr(exposure) 
    light.CreateWidthAttr(width)  
    light.CreateHeightAttr(height)  
    light.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(translate * lengthScale) 
    light.AddRotateXYZOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(rotate) 

    return 


#================================================================================================================||====#
def GroundCreateXR(groundPath, groundName, normal, sizes, axis, lengthScale=1.0) : 
    XformCreate(groundPath, translate=Gf.Vec3f(0.0)) 

    stage = omni.usd.get_context().get_stage()

    groundPlanePath = "%s/%s_Ground" % (groundPath,groundName) 
    groundPlane = UsdGeom.Mesh.Define(stage, groundPlanePath)
    groundPlane.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(0, 0, 0) * lengthScale)
    groundPlane.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Quatf(1, 0, 0, 0))
    groundPlane.CreateDisplayColorAttr([Gf.Vec3f(0.5, 0.5, 0.5)])

    faceVertexCounts = [4]
    faceVertexIndices = [0, 1, 2, 3]

    purpose = UsdGeom.Tokens.default_ # guide | default_  
    #axis = UsdGeom.Tokens.y 

    Nx = normal[0]
    Ny = normal[1]
    Nz = normal[2]
    normals = [Gf.Vec3f(Nx,Ny,Nz), Gf.Vec3f(Nx,Ny,Nz), Gf.Vec3f(Nx,Ny,Nz), Gf.Vec3f(Nx,Ny,Nz)]

    Lx = sizes[0] * lengthScale
    Ly = sizes[1] * lengthScale
    Lz = sizes[2] * lengthScale
    points = [
        Gf.Vec3f(-Lx, -Ly, -Lz) ,
        Gf.Vec3f( Lx, -Ly, -Lz),
        Gf.Vec3f( Lx,  Ly,  Lz),
        Gf.Vec3f(-Lx,  Ly,  Lz),
    ]

    groundPlane.CreateFaceVertexCountsAttr(faceVertexCounts)
    groundPlane.CreateFaceVertexIndicesAttr(faceVertexIndices)
    groundPlane.CreateNormalsAttr(normals)
    groundPlane.CreatePointsAttr(points)

    collisionPlanePath = groundPlanePath + "/%s_Collider" % groundName 
    collisionPlane = PhysxSchema.Plane.Define(stage, collisionPlanePath)
    collisionPlane.CreatePurposeAttr(purpose) 
    collisionPlane.CreateAxisAttr(axis)

    collisionPlanePrim = collisionPlane.GetPrim() 
    collisionAPI = UsdPhysics.CollisionAPI.Apply(collisionPlanePrim)    
    #add_collision_to_collision_group(stage, collisionPlanePrim.GetPrimPath(), collisionGroupGroundSurfacePath)    
    #add_physics_material_to_prim(stage, collisionPlanePrim, Sdf.Path(tarmacMaterialPath))
    return collisionPlanePrim


def GroundCreateXRy(rootPath, groundName, lengthScale) : 
    return GroundCreateXR(rootPath, groundName, 
                            normal=(0,1,0), 
                            sizes=(15.0,0.0,15.0), 
                            axis=UsdGeom.Tokens.y, 
                            lengthScale=lengthScale) 


def GroundCreateXRz(rootPath, groundName, lengthScale) : 
    return GroundCreateXR(rootPath, groundName, 
                    normal=(0,0,1), 
                    sizes=(15.0,15.0,0.0), 
                    axis=UsdGeom.Tokens.z, 
                    lengthScale=lengthScale) 


#================================================================================================================||====#
def _VehicleContext(path, longitudinalAxis, verticalAxis, gravity) : 
    axis = {"X":PhysxSchema.Tokens.posX, "Y":PhysxSchema.Tokens.posY, "Z":PhysxSchema.Tokens.posZ}

    longitudinalAxis = axis.get(longitudinalAxis)
    assert longitudinalAxis is not None 

    verticalAxis = axis.get(verticalAxis)
    assert verticalAxis is not None 

    stage = omni.usd.get_context().get_stage() 
    PrimRemoveAtPath(path) 

    gravityDirection = None 
    if   verticalAxis == PhysxSchema.Tokens.posX : gravityDirection = Gf.Vec3f(-1.0,  0.0,  0.0)
    elif verticalAxis == PhysxSchema.Tokens.posY : gravityDirection = Gf.Vec3f( 0.0, -1.0,  0.0)
    elif verticalAxis == PhysxSchema.Tokens.posZ : gravityDirection = Gf.Vec3f( 0.0,  0.0, -1.0)
    assert gravityDirection is not None 

    scene = UsdPhysics.Scene.Define(stage, path)
    scene.CreateGravityDirectionAttr(gravityDirection)
    scene.CreateGravityMagnitudeAttr(gravity) 

    vehicleContextAPI = PhysxSchema.PhysxVehicleContextAPI.Apply(scene.GetPrim())
    vehicleContextAPI.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
    vehicleContextAPI.CreateVerticalAxisAttr(verticalAxis)
    vehicleContextAPI.CreateLongitudinalAxisAttr(longitudinalAxis)  
    return vehicleContextAPI


def SceneContext(rootPath, name, 
                verticalAxis, longitudinalAxis, gravity, 
                lengthScale = 1.0, 
                massScale = 1.0) : 
    ## X.1. 
    lengthScaleSqr = lengthScale * lengthScale
    kgmsScale = massScale * lengthScaleSqr

    ## X.1. 
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0 / lengthScale)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0 / massScale)


    ## X.1. Physics scene
    scenePath  = None 
    if verticalAxis == "Z" : 
        StageUpAxisSet(verticalAxis) 
        collisionPlanePrim = GroundCreateXRz(rootPath, name, lengthScale=lengthScale) 

        scenePath  = rootPath + "/PhysicsSceneZ"
        scene = UsdPhysics.Scene.Define(stage, scenePath ) 
        scene.CreateGravityDirectionAttr( Gf.Vec3f(0,0,-1) )
        scene.CreateGravityMagnitudeAttr( gravity ) 
        upAxis = PhysxSchema.Tokens.posZ 
    elif verticalAxis == "Y" : 
        StageUpAxisSet(verticalAxis) 
        collisionPlanePrim = GroundCreateXRy(rootPath, name, lengthScale=lengthScale) 

        scenePath  = rootPath + "/PhysicsSceneY"
        scene = UsdPhysics.Scene.Define(stage, scenePath )  
        scene.CreateGravityDirectionAttr( Gf.Vec3f(0,-1,0) )
        scene.CreateGravityMagnitudeAttr( gravity ) 
        upAxis = PhysxSchema.Tokens.posY 
    else : 
        assert False 

    if longitudinalAxis == "X" : forwardAxis = PhysxSchema.Tokens.posX 
    elif longitudinalAxis == "Y" : forwardAxis =PhysxSchema.Tokens.posY 
    elif longitudinalAxis == "Z" : forwardAxis =PhysxSchema.Tokens.posZ  
    else : assert False 

    ## X.1. 
    vehicleContextAPI = PhysxSchema.PhysxVehicleContextAPI.Apply(scene.GetPrim())
    vehicleContextAPI.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
    vehicleContextAPI.CreateVerticalAxisAttr(upAxis) 
    vehicleContextAPI.CreateLongitudinalAxisAttr(forwardAxis)  

    ## X.1. 
    #DomeLightCreate(rootPath, "DomeLight", intensity=1.0, translate=Gf.Vec3f())
    #DistantLightCreate(rootPath, "DistantLight", intensity=1.0, translate=Gf.Vec3f())
    return collisionPlanePrim, scenePath   




#================================================================================================================||====#
#================================================================================================================||====#
def GroundCreate(stage_units_in_meters=1.0) :  
    from omni.isaac.core import World 

    world = World(stage_units_in_meters=stage_units_in_meters)
    world.scene.add_default_ground_plane()

    groundPath = PrimGetPath("GroundPlane")
    print("[GroundCreate] '%s' " % groundPath)
    return groundPath 


def AssetDownload(path, asset_path) : 
    from omni.isaac.core.utils.stage import add_reference_to_stage

    add_reference_to_stage(usd_path=asset_path, prim_path=path)

    stage = omni.usd.get_context().get_stage()
    usdGeomXform = UsdGeom.Xform.Get(stage, path)
    ##usdGeomXform.CreateSizeAttr(size)
    try : usdGeomXform.AddTranslateOp().Set(Gf.Vec3f(0.0)) 
    except : pass 

    try : usdGeomXform.AddRotateXYZOp().Set(Gf.Vec3f(0.0))
    except : pass 

    try : usdGeomXform.AddScaleOp().Set(Gf.Vec3f(1.0)) 
    except : pass 

    return stage.GetPrimAtPath(path)


#================================================================================================================||====#
#================================================================================================================||====#
def SubLayerCreate(pathSubLayer) : 
    ##
    ## https://docs.omniverse.nvidia.com/kit/docs/omni.usd/latest/omni.usd.Functions.html
    ##
    stage = omni.usd.get_context().get_stage()
    rootLayer = stage.GetRootLayer()    
    ## usda = srootLayer.ExportToString() 

    subLayer = Sdf.Layer.CreateNew(pathSubLayer) # +"/sublayer.usd") 
    rootLayer.subLayerPaths.append(subLayer.identifier) 

    #GetEditTarget() 
    Sdf.Layer.SetEditTarget(subLayer.identifier) 

    loaded_layers = rootLayer.GetLoadedLayers()

    print("[LayerCreate] rootLayer:'%s' " % rootLayer) 
    print(loaded_layers) 
    return 


#================================================================================================================||====#
def SubLayerLoad(pathUSD) : 
    ## https://docs.omniverse.nvidia.com/kit/docs/omni.kit.usd_docs/latest/
    ## RemoveSublayer, FlattenLayers, CreateSublayer
    ## omni.kit.usd.layers  
    print("[SubLayerLoad] '%s' opening... " % pathUSD) 

    stage = omni.usd.get_context().get_stage()

    try : 
        omni.kit.commands.execute("CreateSublayer",
            layer_identifier=stage.GetRootLayer().identifier,
            sublayer_position=0,
            new_layer_path=pathUSD,
            transfer_root_content=False,
            create_or_insert=False) # When True, it will create the layer file for you too.
    except : 
        pass 

    layer = Sdf.Find(pathUSD) 
    layer.Reload() 
    print("[SubLayerLoad] '%s' reload!! " % layer.GetDisplayName()) 
    print( layer.GetAssetInfo() ) 

    #root = stage.GetRootLayer()
    #print( type(root) ) 
    #root.RemoveSubLayerPath() 
    """
    layers = stage.GetLayerStack(0)
    print( CheckMembers(layers[0]) )
    for layer in layers : 
        #print( layer.externalReferences ) 
        print( layer.GetDisplayName() ) 
        print( layer.GetAssetInfo() ) 
        print( layer.GetLoadedLayers() ) 
        print() 

    find = Sdf.Find(pathUSD+"_") 
    print(find)
    """
    return 



#================================================================================================================||====#
def AttributeModifier(path, attributeName, attributeValue) : 
    #path = PrimGetPath(name) 
    #if path is None : 
    #    print("[AttributeModifier] Path:'%s' " % path)
    #    exit() 

    stage = omni.usd.get_context().get_stage()

    prim = stage.GetPrimAtPath(path) 
    assert prim is not None 
    ##print( prim.GetAttributes() ) 

    if not prim.HasAttribute(attributeName) : 
        print("[AttributeModifier] '%s.%s' no found!" %(path, attributeName) )  
        return 

    attribute = prim.GetAttribute(attributeName) 
    valueOld = attribute.Get() 

    attribute.Set(attributeValue) 
    return valueOld 


def AttributeGet(name, attributeName) : 
    path = PrimGetPath(name) 
    assert path is not None 

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(path) 
    assert prim is not None 

    if not prim.HasAttribute(attributeName) : 
        print("[AttributeModifier] '%s.%s' no found!" %(path, attributeName) )  
        return 

    attribute = prim.GetAttribute(attributeName) 
    return attribute 


#================================================================================================================||====#
def LinearInterpolation(x, xmin,xmax, ymin,ymax) : 
    if x < xmin : return ymin 
    elif x > xmax : return ymax 
    else : return (x-xmin) / (xmax-xmin) * (ymax-ymin) + ymin
    #return np.clip(y, ymin, ymax)


#================================================================================================================||====#
"""
## X.1.  
groundPath = GroundCreate() 


CylinderAtXform("/Test3", "colliderCylinder2",  Gf.Vec3f(0.0,1.0,1.0), 1, 1, "Y") 
PhysicsSetUp( PrimGetPath("Test3") ) 


Z0 = 5.0
CubeAtXform("/Test1", "colliderCube1", Gf.Vec3f(-1.0,1.0,Z0), Gf.Vec3f(1.1,2.0,1.2) ) 
PhysicsSetUp( PrimGetPath("Test1") ) 

CubeAtXform("/Test2", "colliderCube2", Gf.Vec3f( 1.0,1.0,Z0), Gf.Vec3f(1.2,0.9,1.1) ) 
PhysicsSetUp( PrimGetPath("Test2") ) 


Transforms(PrimGetPath("colliderCube2"), 
scale=Gf.Vec3f(1.0), 
translate=Gf.Vec3f(0.0,0.0,-1.0), 
rotateXYZ=Gf.Vec3f(0.0,0.0,0.0))

MassAPIGet( PrimGetPath("Test2"), Gf.Vec3f(0.0,0.0,-1.0) ) 


JointFixedCreate("/Join", bodyPath0=PrimGetPath("Test2"), bodyPath1=PrimGetPath("Test1")) # :( 
#JointFixedCreate("/Join", bodyPath0=PrimGetPath("colliderCube2"), bodyPath1=PrimGetPath("colliderCube1")) # :/

JointFixedGet("/Join") 
"""

#================================================================================================================||====#
