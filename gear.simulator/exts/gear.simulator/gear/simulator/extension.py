################### python import #####################
import sys
import os
import numpy as np
import math

################### omniverse import #############################
from re import S
import omni
import omni.ext
import omni.ui as ui
from omni.physx.scripts import physicsUtils
from omni.physx.scripts.utils import setRigidBody

#################### pxr import ##################################
from pxr import UsdGeom, Vt, Sdf, Gf, UsdPhysics, Usd, UsdLux

#################################  ui #######################################
from  .ui.style import julia_modeler_style
from .ui.custom_ui_widget import *
from .ui.custom_color_widget import CustomColorWidget

########################## gear import ##############################

from .model.utils import create_gear

GEAR_PROPERTIES = ["teethNum", "radius", "Ad", "De", "base", "p_angle", "width", "skew", "conangle", "crown"]

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
        self.is_update_gear = False # create new gear of update gear
        self.current_gear_path_str = ""
        self.current_gear_driver_speed = 0.05
        self.current_gear_is_driver = False
        self.drivers = {}

        self.current_teethNum = 12
        self.current_radius = 1.0
        self.current_Ad = 0.1
        self.current_De = 0.1 
        self.current_base = 0.2
        self.current_p_angle = 20 
        self.current_width=0.2
        self.current_skew=0
        self.current_conangle=0
        self.current_rack=0
        self.current_crown=0.0

 
        # subscribe to Physics updates:
        self.total_time = 0

        # Track selection change
        events = omni.usd.get_context().get_stage_event_stream()
        self.stage_event_sub = events.create_subscription_to_pop(
            self._on_stage_event, name="Light Manipulator Selection Change"
        )
        
        # Stream
        stream = omni.timeline.get_timeline_interface().get_timeline_event_stream()
        self._timeline_sub = stream.create_subscription_to_pop(self._on_timeline_event)
        self._physics_update_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self.on_physics_step)
 
        # window
        self.build_window()

    def build_window(self):
        """
        Build ui
        """
        self._window = ui.Window("My Window", width=300)
        with self._window.frame:
            self._window.frame.style = julia_modeler_style
            with ui.VStack(height = 0):
                with ui.CollapsableFrame("GEAR"):
                    with ui.VStack(height=0, spacing=0):
                        ui.Line(style_type_name_override="HeaderLine")
                        ui.Spacer(height = 4)
                        self.gear_teethNum_ui = CustomSliderWidget(min=3, max=20, num_type = "int", label="Teeth number:", default_val=12, 
                                                tooltip = "", on_slide_fn=self.get_teethNum, display_range = True)
                        self.gear_radius_ui = CustomSliderWidget(min=-2, max=2, num_type = "float", label="Radius:", default_val=1, 
                                                tooltip = "", on_slide_fn=self.get_radius, display_range = True)                       
                        self.gear_width_ui = CustomSliderWidget(min=0, max=5, num_type = "float", label="Width:", default_val=0.2, 
                                                tooltip = "", on_slide_fn=self.get_width, display_range = True)
                        self.gear_base_ui = CustomSliderWidget(min=0, max=2, num_type = "float", label="Base:", default_val=0.2, 
                                                tooltip = "", on_slide_fn=self.get_base, display_range = True)                       
                        
                        with ui.CollapsableFrame("OTHER PROPERTIES", collapsed = True):
                            with ui.VStack(height=0, spacing=0):
                                ui.Line(style={"color":"gray", "margin_height": 8, "margin_width": 20})

                                self.gear_De_ui = CustomSliderWidget(min=0, max=2, num_type = "float", label="Dedendum:", default_val=0.1, 
                                                        tooltip = "", on_slide_fn=self.get_de, display_range = True)
                                self.gear_Ad_ui = CustomSliderWidget(min=0, max=1, num_type = "float", label="Addendum:", default_val=0.1, 
                                                        tooltip = "", on_slide_fn=self.get_ad, display_range = True)                       

                                ui.Line(style={"color":"gray", "margin_height": 8, "margin_width": 20})

                                self.gear_p_angle_ui = CustomSliderWidget(min=0, max=45, num_type = "float", label="Pressure Angle:", default_val=20, 
                                                        tooltip = "", on_slide_fn=self.get_p_angle, display_range = True)
                                self.gear_skew_ui = CustomSliderWidget(min=-20, max=20, num_type = "float", label="Skewness:", default_val=0, 
                                                        tooltip = "", on_slide_fn=self.get_skew, display_range = True)                       

                                self.gear_conangle_ui = CustomSliderWidget(min=-20, max=20, num_type = "float", label="Conical angle:", default_val=0, 
                                                        tooltip = "", on_slide_fn=self.get_conangle, display_range = True)
                                self.gear_crown_ui = CustomSliderWidget(min=-1, max=1, num_type = "float", label="Crown:", default_val=0, 
                                                        tooltip = "", on_slide_fn=self.get_crown, display_range = True)                       
                        
                        self.create_gear_botton = ui.Button("Create Gear", height = 40, name = "load_button", clicked_fn=self.create_mesh, style={ "margin": 4}, tooltip = "Add/Update gear into scene")
                   
                        ui.Line(style={"color":"gray", "margin_height": 8, "margin_width": 20})

                        CustomSliderWidget(min=-2, max=2, num_type = "float", label="Speed:", default_val=0.05, 
                                                        tooltip = "Gear rotation speed", on_slide_fn=self.get_driver_speed, display_range = True)                       
                     
                        self.add_driver_botton = ui.Button("Add Driver", height = 40, name = "load_button", clicked_fn=self.add_d6_driver)

                        self.remove_driver_botton = ui.Button("Remove Driver", height = 20, name = "load_button", clicked_fn=self.add_d6_driver, visible = False)
                        


                with ui.CollapsableFrame("SCENE UTILITY"):
                    with ui.VStack(height=0, spacing=4):
                        ui.Line(style_type_name_override="HeaderLine")

                        # eco mode
                        CustomBoolWidget(label ="Eco mode:", default_value=False, tooltip = "Turn on/off eco mode in the render setting.", on_checked_fn = self.toggle_eco_mode)
                        # open a new stage
                        ui.Button("New scene", height = 40, name = "load_button", clicked_fn=self.new_scene, style={ "margin": 4}, tooltip = "open a new empty stage")
                        # ground plane
                        self.ground_color_ui = CustomColorWidget(0.86, 0.626, 0.273, label="Ground color:")
                        ui.Button("Add/Remove ground plane", height = 40, name = "load_button", clicked_fn=self.toggle_ground_plane, style={ "margin": 4}, tooltip = "Add or remove the ground plane")

                        # light intensity
                        ui.Line(style={"color":"gray", "margin_height": 8, "margin_width": 20})
                        CustomSliderWidget(min=0, max=3000, label="Light intensity:", default_val=1000, on_slide_fn = self.change_light_intensity)

    def on_shutdown(self):
        self._timeline_sub = None
        self._physics_update_sub = None
        self.stage_event_sub = None

        print("[gear.simulator] MyExtension shutdown")

    ################################################# gear ##############################################
    
    # ui function
    def get_teethNum(self, teethNum):
        self.current_teethNum = teethNum

    def get_radius(self, radius):
        self.current_radius = radius

    def get_width(self, width):
        self.current_width = width

    def get_base(self, base):
        self.current_base = base

    def get_de(self, de):
        self.current_De = de

    def get_ad(self, ad):
        self.current_Ad = ad

    def get_p_angle(self, angle):
        self.current_p_angle = angle 

    def get_skew(self, skew):
        self.current_skew = skew
    
    def get_conangle(self, coangle):
        self.current_conangle = coangle

    def get_crown(self, crown):
        self.current_crown = crown

    def get_driver_speed(self, speed):
        """
        Current gear is driver
        """
        self.current_gear_driver_speed = speed

    ######################### gear model #########################################

    def rig_d6(self, gear_root = "/World/gear", is_driver = True):
        """
        Rig gear with D6 joint
        """
        stage = omni.usd.get_context().get_stage()
        damping = 1e8
        stiffness = 0 #2e5

        # create anchor:
        self._anchorXform = UsdGeom.Xform.Define(
            stage, Sdf.Path(f"{gear_root}/Anchor") 
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
            stage, Sdf.Path(f"{gear_root}/D6Joint") # allegro/
        ) 

        
        self._articulation_root = stage.GetPrimAtPath(f"{gear_root}/model")   
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
                driveAPI.CreateDampingAttr(damping)
                driveAPI.CreateStiffnessAttr(stiffness)

            for rotDof in ["rotX", "rotY", "rotZ"]:
                driveAPI = UsdPhysics.DriveAPI.Apply(rootJointPrim, rotDof)
                driveAPI.CreateTypeAttr("force")
                # driveAPI.CreateMaxForceAttr(self._drive_max_force)
                driveAPI.CreateTargetPositionAttr(0.0)
                driveAPI.CreateDampingAttr(damping)
                driveAPI.CreateStiffnessAttr(stiffness)

        # set limiter
        prim = component.GetPrim()
        for limit_name in ["transX", "transY", "transZ", "rotX", "rotY"]: # "rotZ"
            limit_api = UsdPhysics.LimitAPI.Apply(prim, limit_name)
            limit_api.CreateLowAttr(1.0)
            limit_api.CreateHighAttr(-1.0)

    def add_d6_driver(self):
        """
        Add driver to the gear
        """
        # setup joint to floating hand base
        stage = omni.usd.get_context().get_stage()
        damping = 1e8
        stiffness = 0 #2e5

        gear_root = self.current_gear_path_str

        component = UsdPhysics.Joint.Get(
            stage, Sdf.Path(f"{gear_root}/D6Joint") # allegro/
        ) 

        rootJointPrim = component.GetPrim()
        for rotDof in ["rotZ"]:
            if not UsdPhysics.DriveAPI(rootJointPrim, rotDof):
                driveAPI = UsdPhysics.DriveAPI.Apply(rootJointPrim, rotDof)
                driveAPI.CreateTypeAttr("force")
                # driveAPI.CreateMaxForceAttr(self._drive_max_force)
                driveAPI.CreateTargetPositionAttr(0.0)
                driveAPI.CreateDampingAttr(damping) 
                driveAPI.CreateStiffnessAttr(stiffness)

        # record driver
        self.drivers[gear_root] = {
            "Anchor":f"{gear_root}/Anchor",
            "D6Joint":f"{gear_root}/D6Joint",
            "speed": self.current_gear_driver_speed,
        }
   
    def create_mesh(self):
        """
        Greate gear mesh with rigidbody and collision
        """
        
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

        create_gear(
            int(self.current_teethNum),
            self.current_radius,
            self.current_Ad,
            self.current_De,
            self.current_base,
            self.current_p_angle * np.pi / 180,
            self.current_width,
            self.current_skew,
            self.current_conangle,
            self.current_crown,
            gear_root=model_xform_path_str
            )

        # add rigid body
        model_xform_prim = stage.GetPrimAtPath(model_xform_path_str)
        if self.current_radius >= 0:
            setRigidBody(model_xform_prim, "convexHull", False)
        else:
            setRigidBody(model_xform_prim, "convexDecomposition", False)

        # add d6 joint
        self.rig_d6(gear_root=gear_xform_path_str, is_driver=False)

        # selection
        selection = omni.usd.get_context().get_selection()
        selection.clear_selected_prim_paths()
        selection.set_prim_path_selected(gear_xform_path_str, True, True, True, True)

        self.current_gear_path_str = gear_xform_path_str

        # set attribute
        gear_root_prim = stage.GetPrimAtPath(gear_xform_path_str)
        gear_root_prim.CreateAttribute("gear:name",  Sdf.ValueTypeNames.String, False).Set(gear_xform_path_str.split("/")[-1])
        
        for attr_name in GEAR_PROPERTIES:  
            if attr_name == "teethNum":
                  gear_root_prim.CreateAttribute(f"gear:{attr_name}",  Sdf.ValueTypeNames.Int, False).Set(getattr(self, f"current_{attr_name}"))
            else:
                gear_root_prim.CreateAttribute(f"gear:{attr_name}",  Sdf.ValueTypeNames.Float, False).Set(getattr(self, f"current_{attr_name}"))
        
        # reset driver
        if gear_xform_path_str in self.drivers:
            del self.drivers[gear_xform_path_str]
    ############################################## scene utiltiy #####################################

    def new_scene(self):
        """
        Start a new scene
        """
        # new scene
        omni.kit.window.file.new()
    
    def toggle_eco_mode(self, eco_mode = True):
        """
        Turn on/off eco mode when rendering
        """
        omni.kit.commands.execute("ChangeSetting", path="/rtx/ecoMode/enabled", value=eco_mode)


    def toggle_ground_plane(self):
        """
        Add or remove ground plane 
        """
        stage = omni.usd.get_context().get_stage()
        ground_prim = stage.GetPrimAtPath("/World/groundPlane")
        if not ground_prim:
            # ground_colors = [float(s) for s in self.ground_color_ui.get_color_stringfield().split(",")]
            # ground_color_vec = Gf.Vec3f(*ground_colors)
            up_axis = UsdGeom.GetStageUpAxis(stage)
            physicsUtils.add_ground_plane(stage, "/World/groundPlane", up_axis, 1000, Gf.Vec3f(0.0), Gf.Vec3f(0.3))
            self.change_ground_color()
            
            # select ground
            selection = omni.usd.get_context().get_selection()
            selection.clear_selected_prim_paths()
            selection.set_prim_path_selected("/World/groundPlane", True, True, True, True)
        else:
            omni.kit.commands.execute("DeletePrims", paths=["/World/groundPlane"])

    def change_ground_color(self):
        """
        Change ground color from color ui
        """
        entityPlane = UsdGeom.Mesh.Get(omni.usd.get_context().get_stage(), "/World/groundPlane/CollisionMesh")
        ground_colors = [float(s) for s in self.ground_color_ui.get_color_stringfield().split(",")]
        ground_color_vec = Gf.Vec3f(*ground_colors)
        entityPlane.CreateDisplayColorAttr().Set([ground_color_vec])

    def change_light_intensity(self, intensity:float = 1000):
        """
        Change light intensity
        """
        stage = omni.usd.get_context().get_stage()
        light_prim = stage.GetPrimAtPath("/World/defaultLight")

        if not light_prim:
            # Create basic DistantLight
            omni.kit.commands.execute(
                "CreatePrim",
                prim_path="/World/defaultLight",
                prim_type="DistantLight",
                select_new_prim=False,
                attributes={UsdLux.Tokens.angle: 1.0, UsdLux.Tokens.intensity: 1000},
                create_default_xform=True,
            )

            light_prim = stage.GetPrimAtPath("/World/defaultLight")

        light_prim.GetAttribute("intensity").Set(float(intensity))

    ############################################## event / stream #####################################

    def on_physics_step(self, dt):
        """
        Physics update
        """
        on_start = self.total_time == 0
        self.total_time += dt
        # print("dt", dt)
        delete_driver_list = [] # delete invalid drivers        
        for gear_root in self.drivers:
            # print("gear_root", gear_root)
            anchor = self.stage.GetPrimAtPath(self.drivers[gear_root]["Anchor"])
            if anchor.IsValid():
                anchor_xform = UsdGeom.Xform(anchor)
                # anchor
                ops = anchor_xform.GetOrderedXformOps()
                translateOp = ops[0]
                assert translateOp.GetOpType() == UsdGeom.XformOp.TypeTranslate
                orientOp = ops[1]
                assert orientOp.GetOpType() == UsdGeom.XformOp.TypeOrient

                # q0 = orientOp.Get()

                angle = self.drivers[gear_root]["speed"] *  self.total_time 
                s = math.sin(0.5 * angle)
                q1 = Gf.Quatf(math.cos(angle * 0.5), s * 0, s * 0, s * 1)
                orientOp.Set(q1)
                # print("orientOp", angle, q1)
            # translateOp.Set(self._anchorPositionRateLimiter.current_value)
            else:
                # print("invalid gear", gear_root)
                delete_driver_list.append(gear_root)

        # delete invalid drivers
        if on_start:
            for d in delete_driver_list:
                del self.drivers[d]

        
    def _on_timeline_event(self, e):
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            # self._physics_update_sub = None
            self.total_time = 0.0
        if e.type == int(omni.timeline.TimelineEventType.PLAY):
            self.stage = omni.usd.get_context().get_stage()

    def _on_stage_event(self, event):
        """Called by stage_event_stream"""
        if event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):

            # reset gear button
            self.is_update_gear = False
            self.create_gear_botton.text = "Create Gear"
            self.add_driver_botton.text = "Add Driver"
            self.remove_driver_botton.visible = False

            context  = omni.usd.get_context()
            stage = context.get_stage()
            prim_paths = context.get_selection().get_selected_prim_paths()

            # if selected gear
            if len(prim_paths) == 1:
                prim = stage.GetPrimAtPath(prim_paths[0])
                gear_root = None
                if prim.HasAttribute("gear:name"):
                    gear_root = prim
                elif prim.GetParent().IsValid() and prim.GetParent().HasAttribute("gear:name"):
                    gear_root = prim.GetParent()
                
                # if select gear, update current ui information
                if gear_root:
                    self.is_update_gear = True # updating gear rather than create new gear
                    for attr_name in GEAR_PROPERTIES:
                        attr_value = gear_root.GetAttribute(f"gear:{attr_name}").Get()
                        # if attr_name == "teethNum":
                        #     attr_value = int(attr_value)
                        # set current value
                        # setattr(self,f"current_{attr_name}", attr_value)

                        ui_piece = getattr(self, f"gear_{attr_name}_ui")
                        ui_piece.model.set_value(attr_value)
 
                        self.create_gear_botton.text = "Update Gear"
                    
            