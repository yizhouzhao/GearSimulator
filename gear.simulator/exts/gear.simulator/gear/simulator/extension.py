################### python import #####################
import sys
import os
import numpy as np

################### omniverse import #############################
from re import S
import omni
import omni.ext
import omni.ui as ui

#################### pxr import ##################################
from pxr import UsdGeom, Vt, Sdf, Gf, UsdPhysics, Usd

#################################  ui #######################################
from  .ui.style import julia_modeler_style
from .ui.custom_ui_widget import *

# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class MyExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[gear.simulator] MyExtension startup")

        # stage
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)

        # gear
        self.currect_gear_is_driver = False

        self.current_teethNum = 12
        self.current_radius = 1.0
        self.current_Ad = 0.1
        self.current_De = 0.1 
        self.current_base = 0.2
        self.current_p_angle = 20 * 3.1415936 / 180
        self.current_width=0.2
        self.current_skew=0
        self.current_conangle=0
        self.current_rack=0
        self.current_crown=0.0

        self._window = ui.Window("My Window", width=300)
        with self._window.frame:
            self._window.frame.style = julia_modeler_style
            with ui.VStack(height = 0):
                # eco mode
                CustomBoolWidget(label ="Driver:", default_value=False, tooltip = "Turn on/off eco mode in the render setting.", on_checked_fn = self.toggle_driver)
                CustomSliderWidget(min=3, max=20, num_type = "int", label="Teeth number:", default_val=12, 
                                        tooltip = "", on_slide_fn=self.set_teethNum) 

                ui.Button("Create mesh", clicked_fn=self.create_mesh)
                ui.Button("debug_rig_d6", clicked_fn=self.debug_rig_d6)
                ui.Button("Create tooth", clicked_fn=self.create_tooth)

                with ui.CollapsableFrame("SCENE UTILITY"):
                    with ui.VStack(height=0, spacing=4):
                        ui.Line(style_type_name_override="HeaderLine")

                        # eco mode
                        CustomBoolWidget(label ="Eco mode:", default_value=False, tooltip = "Turn on/off eco mode in the render setting.", on_checked_fn = self.toggle_eco_mode)
                        # open a new stage
                        ui.Button("New scene", height = 40, name = "load_button", clicked_fn=self.new_scene, style={ "margin": 4}, tooltip = "open a new empty stage")
                        # ground plane
                        ui.Button("Add/Remove ground plane", height = 40, name = "load_button", clicked_fn=self.toggle_ground_plane, style={ "margin": 4}, tooltip = "Add or remove the ground plane")
                        # light intensity
                        ui.Line(style={"color":"gray", "margin_height": 8, "margin_width": 20})
                        CustomSliderWidget(min=0, max=3000, label="Light intensity:", default_val=1000, on_slide_fn = self.change_light_intensity)


    # ui function
    def set_teethNum(self, teethNum):
        self.current_teethNum = teethNum


    def on_shutdown(self):
        print("[gear.simulator] MyExtension shutdown")

    def create_mesh(self):
        print("create mesh")

        # box = UsdGeom.Mesh.Define(omni.usd.get_context().get_stage(), "/World/box")
        # box.CreatePointsAttr([(-50, -50, -50), (50, -50, -50), (-50, -50, 50), (50, -50, 50), 
        #                       (-50, 50, -50), (50, 50, -50), (50, 50, 50), (-50, 50, 50)])
        # box.CreateFaceVertexCountsAttr([4, 4, 4, 4, 4, 4])
        # box.CreateFaceVertexIndicesAttr([0, 1, 3, 2, 0, 4, 5, 1, 1, 5, 6, 3, 2, 3, 6, 7, 0, 2, 7, 4, 4, 7, 6, 5])
        # box.CreateSubdivisionSchemeAttr("none")

        # return
        
        stage = omni.usd.get_context().get_stage()

        # create xform as root
        gear_xform_path_str = omni.usd.get_stage_next_free_path(stage, f"/World/gear", False)

        omni.kit.commands.execute(
            "CreatePrim",
            prim_path=gear_xform_path_str,
            prim_type="Xform", # Xform
            select_new_prim=False,
        ) 

        # create xform as root
        model_xform_path_str = omni.usd.get_stage_next_free_path(stage, f"{gear_xform_path_str}/model", False)

        omni.kit.commands.execute(
            "CreatePrim",
            prim_path=model_xform_path_str,
            prim_type="Xform", # Xform
            select_new_prim=False,
        ) 

        gear_model_prim = stage.GetPrimAtPath(model_xform_path_str)

        prim_path = f"{model_xform_path_str}/Mesh"
        mesh = UsdGeom.Mesh.Define(stage, prim_path)

        # add gear
        from .model.add_mesh_gears import add_gear

        verts, faces, verts_tip, verts_valley = add_gear(
            self.current_teethNum,
            self.current_radius,
            self.current_Ad,
            self.current_De,
            self.current_base,
            self.current_p_angle,
            self.current_width,
            self.current_skew,
            self.current_conangle,
            self.current_crown
            )
        
        # triangulate
        # new_faces = []
        # for f in faces:
        #     new_faces.extend([f[0], f[1], f[2]])
        #     new_faces.extend([f[2], f[3], f[0]])

        # scale
        new_verts = []
        for v in verts:
            new_verts.append((100 * v[0], 100 * v[1], 100 * v[2]))
            

        mesh.GetPointsAttr().Set(Vt.Vec3fArray(new_verts))
        point_indices = [item for sublist in faces for item in sublist]
        face_vertex_counts = [len(sublist) for sublist in faces]

        # mesh.GetNormalsAttr().Set(Vt.Vec3fArray(normals))
        mesh.GetFaceVertexIndicesAttr().Set(point_indices) # 
        mesh.GetFaceVertexCountsAttr().Set(face_vertex_counts) # [3] * (len(new_faces) // 3)
        mesh.SetNormalsInterpolation("faceVarying")
        mesh.CreateSubdivisionSchemeAttr("none")


        # add rigid body
        from omni.physx.scripts.utils import setRigidBody
        setRigidBody(gear_model_prim, "convexDecomposition", False)

        # add joint
        
        self.debug_rig_d6(gear_root=gear_xform_path_str, is_driver=self.currect_gear_is_driver)

    def debug_rig_d6(self, gear_root = "/World/gear", is_driver = True):
        self._stage = omni.usd.get_context().get_stage()
        self._damping = 1e5
        self._stiffness = 2e5

        # create anchor:
        self._anchorXform = UsdGeom.Xform.Define(
            self._stage, Sdf.Path(f"{gear_root}/AnchorXform") 
        )
        # these are global coords because world is the xform's parent
        xformLocalToWorldTrans = Gf.Vec3f(0)
        xformLocalToWorldRot = Gf.Quatf(1.0)
        self._anchorXform.AddTranslateOp().Set(xformLocalToWorldTrans)
        self._anchorXform.AddOrientOp().Set(xformLocalToWorldRot)
      
        xformPrim = self._anchorXform.GetPrim()
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(xformPrim)
        physicsAPI.CreateRigidBodyEnabledAttr(True)
        physicsAPI.CreateKinematicEnabledAttr(True)

        # setup joint to floating hand base
        component = UsdPhysics.Joint.Define(
            self._stage, Sdf.Path(f"{gear_root}/AnchorToHandBaseD6") # allegro/
        ) 

        
        self._articulation_root = self._stage.GetPrimAtPath(f"{gear_root}/model")   
        baseLocalToWorld = UsdGeom.Xformable(self._articulation_root).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        jointPosition = baseLocalToWorld.GetInverse().Transform(xformLocalToWorldTrans)
        jointPose = Gf.Quatf(baseLocalToWorld.GetInverse().RemoveScaleShear().ExtractRotationQuat())

        component.CreateExcludeFromArticulationAttr().Set(True)
        component.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0))
        component.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
        component.CreateBody0Rel().SetTargets([self._anchorXform.GetPath()])

        component.CreateBody1Rel().SetTargets([self._articulation_root.GetPath()])
        component.CreateLocalPos1Attr().Set(jointPosition)
        component.CreateLocalRot1Attr().Set(jointPose)

        component.CreateBreakForceAttr().Set(sys.float_info.max)
        component.CreateBreakTorqueAttr().Set(sys.float_info.max)

        rootJointPrim = component.GetPrim()
        if is_driver:
            for dof in ["transX", "transY", "transZ"]:
                driveAPI = UsdPhysics.DriveAPI.Apply(rootJointPrim, dof)
                driveAPI.CreateTypeAttr("force")
                # driveAPI.CreateMaxForceAttr(self._drive_max_force)
                driveAPI.CreateTargetPositionAttr(0.0)
                driveAPI.CreateDampingAttr(self._damping)
                driveAPI.CreateStiffnessAttr(self._stiffness)

            for rotDof in ["rotX", "rotY", "rotZ"]:
                driveAPI = UsdPhysics.DriveAPI.Apply(rootJointPrim, rotDof)
                driveAPI.CreateTypeAttr("force")
                # driveAPI.CreateMaxForceAttr(self._drive_max_force)
                driveAPI.CreateTargetPositionAttr(0.0)
                driveAPI.CreateDampingAttr(self._damping)
                driveAPI.CreateStiffnessAttr(self._stiffness)

        # set limiter
        prim = component.GetPrim()
        for limit_name in ["transX", "transY", "transZ", "rotX", "rotY"]: # "rotZ"
            limit_api = UsdPhysics.LimitAPI.Apply(prim, limit_name)
            limit_api.CreateLowAttr(1.0)
            limit_api.CreateHighAttr(-1.0)

    
    def toggle_driver(self, is_driver:bool):
        """
        Current gear is driver
        """
        self.currect_gear_is_driver = is_driver

    def create_tooth(self):
        print("create_tooth")
        from .model.utils import create_gear

        stage = omni.usd.get_context().get_stage()

        # create xform as root
        gear_xform_path_str = omni.usd.get_stage_next_free_path(stage, f"/World/gear", False)

        omni.kit.commands.execute(
            "CreatePrim",
            prim_path=gear_xform_path_str,
            prim_type="Xform", # Xform
            select_new_prim=False,
        ) 

        create_gear(
            self.current_teethNum,
            self.current_radius,
            self.current_Ad,
            self.current_De,
            self.current_base,
            self.current_p_angle,
            self.current_width,
            self.current_skew,
            self.current_conangle,
            self.current_crown,
            gear_root=gear_xform_path_str
            )

        # add rigid body
        from omni.physx.scripts.utils import setRigidBody
        gear_xform_prim = stage.GetPrimAtPath(gear_xform_path_str)
        setRigidBody(gear_xform_prim, "convexDecomposition", False)

    
    ############################################## scene utiltiy #####################################