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

# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class MyExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[gear.simulator] MyExtension startup")

        self._window = ui.Window("My Window", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                ui.Label("Some Label")

                ui.Button("Create mesh", clicked_fn=self.create_mesh)
                ui.Button("debug_rig_d6", clicked_fn=self.debug_rig_d6)

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

        
        from .model.add_mesh_gears import add_gear

        teethNum = np.random.randint(5, 20)
        radius = 1.0
        Ad = 0.1
        De = 0.1 
        base = 0.2
        p_angle = 20 * 3.1415936 / 180
        width=0.2
        skew=0
        conangle=0
        rack=0
        crown=0.0

        verts, faces, verts_tip, verts_valley = add_gear(
            teethNum,
            radius,
            Ad,
            De,
            base,
            p_angle,
            width,
            skew,
            conangle,
            crown
            )
        # verts = [(1.0, 1.0, -1.0),
        #  (1.0, -1.0, -1.0),
        #  (-1.0, -1.0, -1.0),
        #  (-1.0, 1.0, -1.0),
        #  (1.0, 1.0, 1.0),
        #  (1.0, -1.0, 1.0),
        #  (-1.0, -1.0, 1.0),
        #  (-1.0, 1.0, 1.0)]

        # faces = [(0, 1, 2, 3),
        #  (4, 7, 6, 5),         # (4, 7, 6, 5) --> (4, 5, 6, 7) [4567 is wrong]
        #  (0, 4, 5, 1),
        #  (1, 5, 6, 2),
        #  (2, 6, 7, 3),
        #  (4, 0, 3, 7)]

        # normals = self.get_normals(verts, faces)    
        # print("normals", normals)
        
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
        setRigidBody(gear_model_prim, "meshSimplification", False)


    def debug_rig_d6(self, gear_root = "/World/gear"):
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

        
        self._articulation_root = self._stage.GetPrimAtPath("/World/gear/model")   
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

        prim = component.GetPrim()
        for limit_name in ["transX", "transY", "transZ", "rotX", "rotY"]: # "rotZ"
            limit_api = UsdPhysics.LimitAPI.Apply(prim, limit_name)
            limit_api.CreateLowAttr(1.0)
            limit_api.CreateHighAttr(-1.0)

            