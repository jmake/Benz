#================================================================================================================||====#
from pxr import Usd, UsdLux, UsdGeom, UsdShade, Sdf, Gf, UsdPhysics
from pxr import PhysxSchema

from omni.physx.scripts.physicsUtils import *
from omni.physx.scripts.utils import set_custom_metadata

import numpy as np 


#================================================================================================================||====#
import os 
import sys
import pathlib
path_root = pathlib.Path(__file__)
sys.path.append(str(path_root.parent))
#print("\n\nRunning:'%s'... " % path_root) 

import _OmniverseTools as OmniverseTools


#================================================================================================================||====#
#================================================================================================================||====#
def VehicleContextCreate(scene) : 
    vehicleContextAPI = PhysxSchema.PhysxVehicleContextAPI.Apply( scene.GetPrim() )
    vehicleContextAPI.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
    vehicleContextAPI.CreateVerticalAxisAttr(PhysxSchema.Tokens.posY)
    vehicleContextAPI.CreateLongitudinalAxisAttr(PhysxSchema.Tokens.posZ)
    return 


def _PhysicsSceneCreate(stage, rootPath, name, gravityMagnitude, lengthScale, massScale) :
    ## X.1. 
    scene = UsdPhysics.Scene.Define(stage, rootPath + name)
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, -1, 0))
    scene.CreateGravityMagnitudeAttr(gravityMagnitude) 
 
    ## X.1. 
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0 / lengthScale)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0 / massScale)
 
    ## X.1. 
    VehicleContextCreate(scene)
    return scene 


#================================================================================================================||====#
def TarmacMaterialCreate(stage, rootPath, name) : 
    # materials
    tarmacMaterialPath = rootPath + name #"/TarmacMaterial"
    UsdShade.Material.Define(stage, tarmacMaterialPath) 

    tarmacMaterial = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(tarmacMaterialPath))
    tarmacMaterial.CreateStaticFrictionAttr(0.9)
    tarmacMaterial.CreateDynamicFrictionAttr(0.7)
    tarmacMaterial.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(tarmacMaterial.GetPrim()) 
    return tarmacMaterialPath


def GravelMaterialCreate(stage, rootPath, name) : 
    gravelMaterialPath = rootPath + name #"/GravelMaterial"
    UsdShade.Material.Define(stage, gravelMaterialPath) 

    gravelMaterial = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(gravelMaterialPath))
    gravelMaterial.CreateStaticFrictionAttr(0.6)
    gravelMaterial.CreateDynamicFrictionAttr(0.6)
    gravelMaterial.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(gravelMaterial.GetPrim()) 
    return gravelMaterialPath 


def WinterTireFrictionTableCreate(stage, rootPath, name, tarmacMaterialPath, gravelMaterialPath) : 
    winterTireFrictionTablePath = rootPath + name #"/WinterTireFrictionTable"
    winterTireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable.Define(stage, winterTireFrictionTablePath)
    winterTireFrictionTableGroundMaterialsRel = winterTireFrictionTable.CreateGroundMaterialsRel()
    winterTireFrictionTableGroundMaterialsRel.AddTarget(tarmacMaterialPath)
    winterTireFrictionTableGroundMaterialsRel.AddTarget(gravelMaterialPath)
    winterTireFrictionTable.CreateFrictionValuesAttr([0.75, 0.6]) 
    return winterTireFrictionTablePath 


def SummerTireFrictionTableCreate(stage, rootPath, name, tarmacMaterialPath, gravelMaterialPath) : 
    summerTireFrictionTablePath = rootPath + name #"/SummerTireFrictionTable"
    summerTireFrictionTable = PhysxSchema.PhysxVehicleTireFrictionTable.Define(stage, summerTireFrictionTablePath)
    summerTireFrictionTableGroundMaterialsRel = summerTireFrictionTable.CreateGroundMaterialsRel()
    summerTireFrictionTableGroundMaterialsRel.AddTarget(tarmacMaterialPath)
    summerTireFrictionTableGroundMaterialsRel.AddTarget(gravelMaterialPath)
    summerTireFrictionTable.CreateFrictionValuesAttr([0.7, 0.6])
    return summerTireFrictionTablePath 


#================================================================================================================||====#
def CollisionGroupsCreate(stage, rootPath, name) : 
    groupName = "Groups_%s" % name

    collisionGroupGroundSurfacePath = "%s/GroundSurfacePath" % rootPath
    collisionGroupVehicleGroundQueryPath = "%s/VehicleGroundQueryPath" % rootPath
    collisionGroupVehicleChassisPath = "%s/VehicleChassisPath"  % rootPath
    collisionGroupVehicleWheelPath = "%s/VehicleWheelPath" % rootPath

    collisionGroupVehicleChassis = UsdPhysics.CollisionGroup.Define(stage, collisionGroupVehicleChassisPath)
    collisionGroupVehicleChassisRel = collisionGroupVehicleChassis.CreateFilteredGroupsRel()
    collisionGroupVehicleChassisRel.AddTarget(collisionGroupVehicleGroundQueryPath)

    collisionGroupVehicleWheel = UsdPhysics.CollisionGroup.Define(stage, collisionGroupVehicleWheelPath)
    collisionGroupVehicleWheelRel = collisionGroupVehicleWheel.CreateFilteredGroupsRel()
    collisionGroupVehicleWheelRel.AddTarget(collisionGroupVehicleGroundQueryPath)
    collisionGroupVehicleWheelRel.AddTarget(collisionGroupGroundSurfacePath)

    collisionGroupVehicleGroundQuery = UsdPhysics.CollisionGroup.Define(stage, collisionGroupVehicleGroundQueryPath)
    collisionGroupVehicleGroundQueryRel = collisionGroupVehicleGroundQuery.CreateFilteredGroupsRel()
    collisionGroupVehicleGroundQueryRel.AddTarget(collisionGroupVehicleChassisPath)
    collisionGroupVehicleGroundQueryRel.AddTarget(collisionGroupVehicleWheelPath)

    collisionGroupGroundSurface = UsdPhysics.CollisionGroup.Define(stage, collisionGroupGroundSurfacePath)
    collisionGroupGroundSurfaceRel = collisionGroupGroundSurface.CreateFilteredGroupsRel()
    collisionGroupGroundSurfaceRel.AddTarget(collisionGroupGroundSurfacePath)
    collisionGroupGroundSurfaceRel.AddTarget(collisionGroupVehicleWheelPath) 

    paths = [collisionGroupGroundSurfacePath, collisionGroupVehicleGroundQueryPath, 
            collisionGroupVehicleWheelPath, collisionGroupVehicleChassisPath] 
    return paths 


#================================================================================================================||====#
#==========================================================================================================| Body |====#
def ChassisRenderCreate(stage, vehiclePath, name, 
                        chassisOffset, 
                        chassisHalfExtents, 
                        lengthScale, 
                        show) : 

    rootPath = "%s/%s" % (vehiclePath, name) 
    UsdGeom.Xform.Define(stage, rootPath)

    vehicleChassisName = "%s_Model" % name 
    vehicleChassisPath = "%s/%s" % (rootPath, vehicleChassisName) 

    #vehicleChassisPath = vehiclePath + name  
    vehicleChassis = UsdGeom.Mesh.Define(stage, vehicleChassisPath)
    vehicleChassis.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(chassisOffset)
    vehicleChassis.CreateDisplayColorAttr([Gf.Vec3f(0.2784314, 0.64705884, 1)])

    faceVertexCounts = [4, 4, 4, 4, 4, 4]

    faceVertexIndices = [0, 1, 3, 2, 4, 5, 7, 6, 10, 11, 13, 12, 14, 15, 9, 8, 17, 23, 21, 19, 22, 16, 18, 20]

    normals = [
        Gf.Vec3f(0, 1, 0),
        Gf.Vec3f(0, 1, 0),
        Gf.Vec3f(0, 1, 0),
        Gf.Vec3f(0, 1, 0),
        Gf.Vec3f(0, -1, 0),
        Gf.Vec3f(0, -1, 0),
        Gf.Vec3f(0, -1, 0),
        Gf.Vec3f(0, -1, 0),
        Gf.Vec3f(0, 0, -1),
        Gf.Vec3f(0, 0, -1),
        Gf.Vec3f(0, 0, 1),
        Gf.Vec3f(0, 0, 1),
        Gf.Vec3f(0, 0, 1),
        Gf.Vec3f(0, 0, 1),
        Gf.Vec3f(0, 0, -1),
        Gf.Vec3f(0, 0, -1),
        Gf.Vec3f(-1, 0, 0),
        Gf.Vec3f(1, 0, 0),
        Gf.Vec3f(-1, 0, 0),
        Gf.Vec3f(1, 0, 0),
        Gf.Vec3f(-1, 0, 0),
        Gf.Vec3f(1, 0, 0),
        Gf.Vec3f(-1, 0, 0),
        Gf.Vec3f(1, 0, 0),
    ]

    points = [
        Gf.Vec3f(-chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
        Gf.Vec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
        Gf.Vec3f(chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
    ]

    vehicleChassis.CreateFaceVertexCountsAttr(faceVertexCounts)
    vehicleChassis.CreateFaceVertexIndicesAttr(faceVertexIndices)
    vehicleChassis.CreateNormalsAttr(normals)
    vehicleChassis.CreatePointsAttr(points) 

    if not show : OmniverseTools.InvisibleSet(vehicleChassisName) 
    return vehicleChassisPath 


#================================================================================================================||====#
def ChassiscollisionCreate(stage, vehiclePath, name, 
                            collisionGroupVehicleChassisPath, 
                            chassisOffset, chassisHalfExtents, 
                            lengthScale) :
    vehicleChassisPath = vehiclePath + name 

    vehicleChassis = UsdGeom.Cube.Define(stage, vehicleChassisPath)
    vehicleChassis.CreatePurposeAttr(UsdGeom.Tokens.guide)
    vehicleChassis.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(chassisOffset)
    vehicleChassis.AddScaleOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(chassisHalfExtents)


    vehicleChassisPrim = vehicleChassis.GetPrim()
    collisionAPI = UsdPhysics.CollisionAPI.Apply(vehicleChassisPrim)
    add_collision_to_collision_group(stage, vehicleChassisPrim.GetPrimPath(), collisionGroupVehicleChassisPath)

    physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(vehicleChassisPrim)
    physxCollisionAPI.CreateRestOffsetAttr(0.0 * lengthScale)
    physxCollisionAPI.CreateContactOffsetAttr(0.02 * lengthScale)
    return 


#================================================================================================================||====#
def PhysxVehicleAPI(vehiclePrim, lengthScale) : 
    vehicleAPI = PhysxSchema.PhysxVehicleAPI.Apply(vehiclePrim)
    vehicleAPI.CreateVehicleEnabledAttr(True)
    vehicleAPI.CreateSubStepThresholdLongitudinalSpeedAttr(5.0 * lengthScale)
    vehicleAPI.CreateLowForwardSpeedSubStepCountAttr(3) 
    vehicleAPI.CreateHighForwardSpeedSubStepCountAttr(1) 
    vehicleAPI.CreateMinPassiveLongitudinalSlipDenominatorAttr(4.0 * lengthScale) 
    vehicleAPI.CreateMinActiveLongitudinalSlipDenominatorAttr(0.1 * lengthScale) 
    vehicleAPI.CreateMinLateralSlipDenominatorAttr(1.0 * lengthScale) 
    set_custom_metadata(vehiclePrim, PhysxSchema.Tokens.referenceFrameIsCenterOfMass, False) 
    return vehicleAPI 


#================================================================================================================||====#
def PhysxRigidBodyAPI(vehiclePrim, mass, vehicleMassBoxDim, lengthScale) : 
    UsdPhysics.RigidBodyAPI.Apply(vehiclePrim)

    massAPI = UsdPhysics.MassAPI.Apply(vehiclePrim)
    massAPI.CreateMassAttr(mass)

    massAPI.CreateDiagonalInertiaAttr(
        Gf.Vec3f(
            (vehicleMassBoxDim[1] * vehicleMassBoxDim[1]) + (vehicleMassBoxDim[2] * vehicleMassBoxDim[2]),
            (vehicleMassBoxDim[0] * vehicleMassBoxDim[0]) + (vehicleMassBoxDim[2] * vehicleMassBoxDim[2]),
            (vehicleMassBoxDim[0] * vehicleMassBoxDim[0]) + (vehicleMassBoxDim[1] * vehicleMassBoxDim[1]),
        )
        * (1 / 12)
        * mass
    )
    massAPI.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0)) 

    rigidBodyAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(vehiclePrim)
    rigidBodyAPI.CreateDisableGravityAttr(True) 
    return 


#================================================================================================================||====#
def VehiclePrimCreate(stage, rootPath, name, center) : 
    vehiclePath = "%s/%s" % (rootPath,name) 
    vehicle = UsdGeom.Xform.Define(stage, vehiclePath)
    vehicle.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(center) 
    vehicle.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(Gf.Quatf(1, 0, 0, 0))
    return vehicle.GetPrim() 


#================================================================================================================||====#




#================================================================================================================||====#
#========================================================================================================| Wheels |====#
def WheelRenderCreate(stage, wheelPath, name, wheelWidth, wheelRadius, wheelAxis, wheelShow) : 
    rootPath = "%s/%s" % (wheelPath, name) 
    UsdGeom.Xform.Define(stage, rootPath)

    vehicleWheelRenderName = "%s_Model" % name 
    vehicleWheelRenderPath = "%s/%s" % (rootPath, vehicleWheelRenderName) 

    #vehicleWheelRenderPath = "%s/%s" % (wheelPath, name) 
    vehicleWheelRender = UsdGeom.Cylinder.Define(stage, vehicleWheelRenderPath)
    vehicleWheelRender.CreateHeightAttr(wheelWidth)
    vehicleWheelRender.CreateRadiusAttr(wheelRadius) 

    if wheelAxis == "X" : vehicleWheelRender.CreateAxisAttr(UsdGeom.Tokens.x) 
    elif wheelAxis == "Y" : vehicleWheelRender.CreateAxisAttr(UsdGeom.Tokens.y) 
    elif wheelAxis == "Z" : vehicleWheelRender.CreateAxisAttr(UsdGeom.Tokens.z) 
    else : assert False 

    # if height or radius is authored, USD expects extent to be authored too
    cylExtent = UsdGeom.Cylinder.ComputeExtentFromPlugins(vehicleWheelRender, 0)
    vehicleWheelRender.CreateExtentAttr(cylExtent)

    if not wheelShow : OmniverseTools.InvisibleSet(vehicleWheelRenderName) 
    return vehicleWheelRenderPath 


#================================================================================================================||====#
def WheelCollisionShapesCreate(stage, wheelPath, name, 
                                wheelWidth, wheelRadius, wheelAxis, 
                                collisionGroupVehicleWheelPath, lengthScale) :  
    vehicleWheelCollPath = "%s/%s" % (wheelPath, name) 
    vehicleWheelColl = UsdGeom.Cylinder.Define(stage, vehicleWheelCollPath)
    vehicleWheelColl.CreatePurposeAttr(UsdGeom.Tokens.guide)
    vehicleWheelColl.CreateHeightAttr(wheelWidth)
    vehicleWheelColl.CreateRadiusAttr(wheelRadius) 

    if wheelAxis == "X" : vehicleWheelColl.CreateAxisAttr(UsdGeom.Tokens.x) 
    elif wheelAxis == "Y" : vehicleWheelColl.CreateAxisAttr(UsdGeom.Tokens.y) 
    elif wheelAxis == "Z" : vehicleWheelColl.CreateAxisAttr(UsdGeom.Tokens.z) 
    else : assert False 

    # if height or radius is authored, USD expects extent to be authored too
    cylExtent = UsdGeom.Cylinder.ComputeExtentFromPlugins(vehicleWheelColl, 0)
    vehicleWheelColl.CreateExtentAttr(cylExtent)

    vehicleWheelCollPrim = vehicleWheelColl.GetPrim()
    collisionAPI = UsdPhysics.CollisionAPI.Apply(vehicleWheelCollPrim)
    add_collision_to_collision_group(stage, vehicleWheelCollPath, collisionGroupVehicleWheelPath)

    physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(vehicleWheelCollPrim)
    physxCollisionAPI.CreateRestOffsetAttr(0.0 * lengthScale)
    physxCollisionAPI.CreateContactOffsetAttr(0.02 * lengthScale) 
    return 


#================================================================================================================||====#
def PhysxVehicleTireAPI(wheelAttachmentAPI, stiffnessGraph, 
                        tireLongStiffness, tireCamberStiffness, 
                        winterTireFrictionTablePath) : 
    tireAPI = PhysxSchema.PhysxVehicleTireAPI.Apply(wheelAttachmentAPI.GetPrim())
    tireAPI.CreateLateralStiffnessGraphAttr(stiffnessGraph)  
    tireAPI.CreateLongitudinalStiffnessAttr(tireLongStiffness)
    tireAPI.CreateCamberStiffnessAttr(tireCamberStiffness)
    tireAPI.CreateFrictionVsSlipGraphAttr([Gf.Vec2f(0.0, 1.0), Gf.Vec2f(0.1, 1.0), Gf.Vec2f(1.0, 1.0)])
    tireFrictionTableRel = tireAPI.CreateFrictionTableRel()
    tireFrictionTableRel.AddTarget(winterTireFrictionTablePath)
    return 


def PhysxVehicleSuspensionAPI(wheelAttachmentAPI, springStrength, springDamping, suspensionTravelDistance) : 
    suspensionAPI = PhysxSchema.PhysxVehicleSuspensionAPI.Apply(wheelAttachmentAPI.GetPrim())
    suspensionAPI.CreateSpringStrengthAttr(springStrength)
    suspensionAPI.CreateSpringDamperRateAttr(springDamping)
    suspensionAPI.CreateTravelDistanceAttr(suspensionTravelDistance)
    return 


def PhysxVehicleWheelAPI(wheelAttachmentAPI, wheelRadius, wheelWidth, massScale, kgmsScale) : 
    wheelAPI = PhysxSchema.PhysxVehicleWheelAPI.Apply(wheelAttachmentAPI.GetPrim())
    wheelAPI.CreateRadiusAttr(wheelRadius)
    wheelAPI.CreateWidthAttr(wheelWidth)
    wheelAPI.CreateMassAttr(20 * massScale)
    wheelAPI.CreateMoiAttr(1.225 * kgmsScale)
    wheelAPI.CreateDampingRateAttr(0.25 * kgmsScale) 
    return wheelAPI 


def PhysxVehicleWheelAttachmentAPI(vehicleWheel, idx, collisionGroupVehicleGroundQueryPath, travelDirection, suspFramePos) : 
    wheelAttachmentAPI = PhysxSchema.PhysxVehicleWheelAttachmentAPI.Apply(vehicleWheel.GetPrim())

    collisionGroupRel = wheelAttachmentAPI.CreateCollisionGroupRel()
    collisionGroupRel.AddTarget(collisionGroupVehicleGroundQueryPath)

    wheelAttachmentAPI.CreateSuspensionTravelDirectionAttr(travelDirection) 
    wheelAttachmentAPI.CreateSuspensionFramePositionAttr(suspFramePos)
    wheelAttachmentAPI.CreateIndexAttr(idx)
    return wheelAttachmentAPI 


#================================================================================================================||====#
def WheelCreate(stage, vehiclePath, name, idx, 
                wheelRadius, wheelWidth, wheelPos, suspFramePos, 
                props) :  
    vehicleWheelPath = vehiclePath + name 

    ## X.1 
    vehicleWheel = UsdGeom.Xform.Define(stage, vehicleWheelPath)
    vehicleWheel.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(wheelPos)

    ## X.1 
    travelDirection = props.get("travelDirection")  
    collisionGroupVehicleGroundQueryPath = props.get("collisionGroupVehicleGroundQueryPath")  
    wheelAttachmentAPI = PhysxVehicleWheelAttachmentAPI(vehicleWheel, idx, 
                                                        collisionGroupVehicleGroundQueryPath, 
                                                        travelDirection, suspFramePos) 

    massScale = props.get("massScale")  
    kgmsScale = props.get("kgmsScale")  
    wheelAPI = PhysxVehicleWheelAPI(wheelAttachmentAPI, wheelRadius, wheelWidth, massScale, kgmsScale)

    ## X.1 
    stiffnessGraph = props.get("stiffnessGraph")   
    tireLongStiffness = props.get("tireLongStiffness")  
    tireCamberStiffness = props.get("tireCamberStiffness")  
    winterTireFrictionTablePath = props.get("winterTireFrictionTablePath")  
    PhysxVehicleTireAPI(wheelAttachmentAPI, stiffnessGraph, tireLongStiffness, tireCamberStiffness, winterTireFrictionTablePath) 

    ## X.1 
    springDamping = props.get("springDamping")  
    springStrength = props.get("springStrength")  
    suspensionTravelDistance = props.get("suspensionTravelDistance")  
    PhysxVehicleSuspensionAPI(wheelAttachmentAPI, springStrength, springDamping, suspensionTravelDistance) 
    return vehicleWheelPath 


#================================================================================================================||====#
def ControllerPropsCreate(kgmsScale) : 
    props = {} 

    props["brakes0MaxBrakeTorque"] = 3600 * kgmsScale
    props["brakes1MaxBrakeTorque"] = 3000 * kgmsScale
    props["brakes1Wheels"] = [2, 3] 

    props["steerAngle"] = 0.554264
    props["steerWheels"] = [0, 1]  

    props["differentialWheels"] = [0, 1] 
    props["differentialTorqueRatios"] = [0.5, 0.5] 

    props["drivePeakTorque"] = 1000.0 * kgmsScale 
    return props 


def PhysxVehicleDriveBasicAPI(vehiclePrim, props) :  
    ## X.1.
    brakes0MaxBrakeTorque = props.get("brakes0MaxBrakeTorque")   

    brakes1MaxBrakeTorque = props.get("brakes1MaxBrakeTorque")  
    brakes1Wheels = props.get("brakes1Wheels")  

    steerAngle = props.get("steerAngle")  
    steerWheels = props.get("steerWheels")  

    differentialWheels = props.get("differentialWheels")  
    differentialTorqueRatios = props.get("differentialTorqueRatios")  

    drivePeakTorque = props.get("drivePeakTorque")  

    ## X.1. brake for all wheels
    brakes0API = PhysxSchema.PhysxVehicleBrakesAPI.Apply(vehiclePrim, PhysxSchema.Tokens.brakes0)
    brakes0API.CreateMaxBrakeTorqueAttr(brakes0MaxBrakeTorque)

    ## X.1. handbrake for rear wheels only
    brakes1API = PhysxSchema.PhysxVehicleBrakesAPI.Apply(vehiclePrim, PhysxSchema.Tokens.brakes1)
    brakes1API.CreateMaxBrakeTorqueAttr(brakes1MaxBrakeTorque) 
    brakes1API.CreateWheelsAttr(brakes1Wheels) 

    ## X.1. steering for front wheels
    steeringAPI = PhysxSchema.PhysxVehicleSteeringAPI.Apply(vehiclePrim)
    steeringAPI.CreateMaxSteerAngleAttr(steerAngle)
    steeringAPI.CreateWheelsAttr(steerWheels)

    ## X.1.  
    differentialAPI = PhysxSchema.PhysxVehicleMultiWheelDifferentialAPI.Apply(vehiclePrim)
    differentialAPI.CreateTorqueRatiosAttr(differentialTorqueRatios)
    differentialAPI.CreateWheelsAttr(differentialWheels)

    ## X.1.  
    driveAPI = PhysxSchema.PhysxVehicleDriveBasicAPI.Apply(vehiclePrim)
    driveAPI.CreatePeakTorqueAttr(drivePeakTorque) 

    ## X.1.  
    vehicleControllerAPI = PhysxSchema.PhysxVehicleControllerAPI.Apply(vehiclePrim)
    vehicleControllerAPI.CreateAcceleratorAttr(0.0)
    vehicleControllerAPI.CreateBrake0Attr(0)
    vehicleControllerAPI.CreateBrake1Attr(0)
    vehicleControllerAPI.CreateSteerAttr(0.0) 
    vehicleControllerAPI.CreateTargetGearAttr(0.0)
    return 


#================================================================================================================||====#
class PhysxVehicleDriveBasicAPICreate : 

    massScale = 1.0
    lengthScale = 1.0

    lengthScaleSqr = lengthScale * lengthScale
    kgmsScale = massScale * lengthScaleSqr
    forceScale = massScale * lengthScale

    gravityMagnitude = 9.8 * lengthScale

    wheels = {} 
    wheelPaths = {}   
    parentNames = {}

    def __init__(self, stage, rootPath, translate) : 
        OmniverseTools.XformCreate(rootPath, translate)  

        self.mass = 1.0 
        self.stage = stage 
        self.rootPath = rootPath  
        self.scenePath = rootPath
        self.chassisPath = rootPath 
        return 


    def GroundSurface(self, sceneName, verticalAxis, longitudinalAxis) : 
        path = "/%s/GroundSurface" % sceneName

        gravityMagnitude = self.gravityMagnitude  
        collisionPlanePrim,contextPath = OmniverseTools.SceneContext(path, "Plane", 
                                                                    verticalAxis, 
                                                                    longitudinalAxis, 
                                                                    gravityMagnitude)  
        self.verticalAxis = verticalAxis
        self.longitudinalAxis = longitudinalAxis 

        self.contextPath = contextPath 
        self.collisionPlanePrim = collisionPlanePrim                                                                   
        return 


    def SceneContext(self, sceneName, rootName, verticalAxis, longitudinalAxis) : 
        stage =  self.stage 
        massScale = self.massScale  
        lengthScale = self.lengthScale  
        collisionPlanePrim = self.collisionPlanePrim 

        scenePath = "/%s" % sceneName  

        materialsName = "Materials_%s" % rootName
        tarmacMaterialPath = TarmacMaterialCreate(stage, scenePath, "/%s/Tarmac_%s" % (materialsName,rootName)) 
        gravelMaterialPath = GravelMaterialCreate(stage, scenePath, "/%s/Gravel_%s" % (materialsName,rootName)) 

        winterTireFrictionTablePath = WinterTireFrictionTableCreate(stage, scenePath, 
                                                                    "/%s/WinterTable_%s" % (materialsName,rootName), 
                                                                    tarmacMaterialPath, gravelMaterialPath)
        summerTireFrictionTablePath = SummerTireFrictionTableCreate(stage, scenePath, 
                                                                    "/%s/SummerTable_%s" % (materialsName,rootName), 
                                                                    tarmacMaterialPath, gravelMaterialPath) 

        groupGroundSurface, groupVehicleGroundQuery, groupVehicleWheel, groupVehicleChassis = CollisionGroupsCreate(stage, scenePath, rootName) 

        add_collision_to_collision_group(stage, collisionPlanePrim.GetPrimPath(), groupGroundSurface)    
        add_physics_material_to_prim(stage, collisionPlanePrim, Sdf.Path(tarmacMaterialPath))

        self.stage = stage 
        self.scenePath = scenePath 
        self.groupVehicleWheel = groupVehicleWheel  
        self.groupVehicleChassis = groupVehicleChassis   
        self.groupVehicleGroundQuery = groupVehicleGroundQuery   
        self.winterTireFrictionTablePath = winterTireFrictionTablePath 
        self.summerTireFrictionTablePath = summerTireFrictionTablePath 
        return 


    def ChassisCreate(self, name, dims, offset, show) : 
        stage = self.stage 
        rootPath = self.rootPath
        massScale = self.massScale  
        kgmsScale = self.kgmsScale 
        lengthScale = self.lengthScale  
        collisionGroupVehicleChassisPath = self.groupVehicleChassis

        ## X.1. 
        mass = 1800 * massScale
        vehicleMassBoxDim = dims * lengthScale

        # Rigid body 
        vehiclePrim = VehiclePrimCreate(stage, rootPath, name, Gf.Vec3f(0.0))  
        PhysxRigidBodyAPI(vehiclePrim, mass, vehicleMassBoxDim, lengthScale) 
        PhysxVehicleAPI(vehiclePrim, lengthScale) 

        ## X.1. 
        props = ControllerPropsCreate(kgmsScale) 
        PhysxVehicleDriveBasicAPI(vehiclePrim, props) 

        ## X.1. 
        chassisHalfExtents = vehicleMassBoxDim * 0.5 

        chassisPath = str(vehiclePrim.GetPath())   
        ChassiscollisionCreate(stage, chassisPath, "/%sGuide" % name, 
                                collisionGroupVehicleChassisPath, 
                                offset, 
                                chassisHalfExtents, 
                                lengthScale) 

        chassiName = "%sRender" % name
        ChassisRenderCreate(stage, chassisPath, "%s" % chassiName, 
                            offset, 
                            chassisHalfExtents, 
                            lengthScale, show)  
                            
        self.parentNames[name] = {"parent":chassiName}   

        self.mass = mass 
        self.chassisPath = chassisPath  
        return 


    def WheelAdd(self, name, idx, wheelRadius, wheelWidth, wheelPos) :  
        self.wheels[name] = (idx, wheelRadius, wheelWidth, wheelPos)  
        return 


    def WheelsCreate(self, name, normalAxis, show, TravelDistanceTimes=2.0) :  
        stage = self.stage 
        lengthScale = self.lengthScale  

        verticalAxis = self.verticalAxis
        longitudinalAxis = self.longitudinalAxis 
        assert not verticalAxis == normalAxis 
        assert not longitudinalAxis == normalAxis 

        suspProps = self.SuspensionsPropertiesCreate(TravelDistanceTimes=TravelDistanceTimes)  
        collisionGroupVehicleWheelPath = self.groupVehicleWheel

        wheelsPath = "%s/%s" % (self.chassisPath, name) 
        for name,(idx, radius, width, pos) in self.wheels.items() : 
            self.WheelCreate(wheelsPath, name, idx, radius, width, pos, suspProps) 

        ## X.1. 
        for name,(idx, radius, width, pos) in self.wheels.items() : 
            wheelPath = self.wheelPaths.get(name) 
            WheelCollisionShapesCreate(stage, wheelPath, "%s_Guide" % name, 
                                        width, radius, normalAxis, 
                                        collisionGroupVehicleWheelPath, 
                                        lengthScale)

        ## X.1. 
        for name,(idx, radius, width, pos) in self.wheels.items() : 
            wheelPath = self.wheelPaths.get(name) 
            renderName = "%s_Render" % name
            renderPath = WheelRenderCreate(stage, wheelPath, 
                                            renderName, width, radius, 
                                            normalAxis, show)  
            self.parentNames[name] = {"parent":renderName}   
        
        self.wheels.clear()  
        return 


    def WheelCreate(self, path, name, idx, wheelRadius, wheelWidth, wheelPos, suspProps) :
        stage = self.stage 
        lengthScale = self.lengthScale  

        wheelRestSuspLength = suspProps.get("wheelRestSuspLength")
        travelDirection = suspProps.get("travelDirection")
        suspFramePos = wheelPos + wheelRestSuspLength * travelDirection 

        vehicleWheelPath = WheelCreate(stage, path, name, idx, 
                                        wheelRadius, wheelWidth, wheelPos, 
                                        suspFramePos, suspProps)  

        self.wheelPaths[name] = vehicleWheelPath 
        return 


    def SuspensionsPropertiesCreate(self, TravelDistanceTimes=2.0) :
        mass = self.mass 
        massScale = self.massScale 
        kgmsScale = self.kgmsScale  
        forceScale = self.forceScale  
        contextPath = self.contextPath
        gravityMagnitude = self.gravityMagnitude   
        summerTireFrictionTablePath = self.summerTireFrictionTablePath 

        sprungMass = mass / 4
        springStrength = 45000 * massScale
        maxDroop = (sprungMass * gravityMagnitude) / springStrength

        suspensionTravelDistance =  maxDroop * TravelDistanceTimes  
        tireRestLoad = sprungMass * gravityMagnitude
        
        gravityDirection = OmniverseTools.SceneGetGravityDirectionAttr(contextPath) 

        props = {}  
        props["travelDirection"] = gravityDirection # Gf.Vec3f(0, -1, 0) 
        props["collisionGroupVehicleGroundQueryPath"] = self.groupVehicleGroundQuery 
        props["massScale"] = massScale 
        props["kgmsScale"] = kgmsScale 
        props["stiffnessGraph"] = Gf.Vec2f(2.0, 17.0 * tireRestLoad)  
        props["tireLongStiffness"] = 5000 * forceScale 
        props["tireCamberStiffness"] = 0 * forceScale 
        props["winterTireFrictionTablePath"] = self.winterTireFrictionTablePath 
        props["springDamping"] = 4500 * massScale 
        props["springStrength"] = springStrength 
        props["suspensionTravelDistance"] = suspensionTravelDistance 
        props["wheelRestSuspLength"] = (suspensionTravelDistance - maxDroop)  
        return props 


#================================================================================================================||====#
#================================================================================================================||====#