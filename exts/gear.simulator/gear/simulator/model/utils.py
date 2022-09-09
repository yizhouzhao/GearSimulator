from math import (
        atan, asin, cos,
        sin, tan, pi,
        radians,
        )

import omni
from pxr import UsdGeom, Vt, Sdf, Gf, UsdPhysics, Usd

from .add_mesh_gears import add_tooth, createFaces 

def create_mesh_from_verts_faces(verts, faces, mesh_path_str="/World/gear", scale = 100):
    """
    Create mesh from vertics and faces
    """
    stage = omni.usd.get_context().get_stage()

    # Defien mesh path
    mesh_path_str = omni.usd.get_stage_next_free_path(stage, mesh_path_str, False)
    mesh = UsdGeom.Mesh.Define(stage, mesh_path_str)

    # scale
    new_verts = [(scale * v[0], scale * v[1], scale * v[2]) for v in verts]        

    mesh.GetPointsAttr().Set(Vt.Vec3fArray(new_verts))
    point_indices = [item for sublist in faces for item in sublist]
    face_vertex_counts = [len(sublist) for sublist in faces]

    # mesh.GetNormalsAttr().Set(Vt.Vec3fArray(normals))
    mesh.GetFaceVertexIndicesAttr().Set(point_indices) # 
    mesh.GetFaceVertexCountsAttr().Set(face_vertex_counts) # [3] * (len(new_faces) // 3)
    mesh.SetNormalsInterpolation("faceVarying")
    mesh.CreateSubdivisionSchemeAttr("none")

def create_gear(teethNum, radius, Ad, De, base, p_angle,
             width=1, skew=0, conangle=0, rack=0, crown=0.0, gear_root = "/World/gear"):
    """
    Create gear with separate convex teeth
    """

    if teethNum < 2:
        return 
    
    # rotation each
    t = 2 * pi / teethNum

    if rack:
        teethNum = 1

    #print(radius, width, conangle)
    if radius != 0:
        scale = (radius - 2 * width * tan(conangle)) / radius
    else:
        scale = radius - 2 * width * tan(conangle)
    
    
    body_verts = []
    body_faces = []

    body_verts_outer_top = []
    body_verts_outer_bottom = []
    body_verts_inner_top = []
    body_verts_inner_bottom = []
    

    verts_bridge_prev = []
    for toothCnt in range(teethNum): # 
        a = toothCnt * t

        verts_bridge_start = []
        verts_bridge_end = []

        verts_outside_top = []
        verts_outside_bottom = []

        tooth_verts = []
        tooth_faces = []
        for (s, d, c, top) \
            in [(0, -width, 1, True), (skew, width, scale, False)]:

                verts1, verts2, verts3, verts4 = add_tooth(a + s, t, d,
                    radius * c, Ad * c, De * c, base * c, p_angle,
                    rack, crown)
                
                # body
                vertsIdx1 = list(range(len(body_verts), len(body_verts) + len(verts1)))
                body_verts.extend(verts1)

                vertsIdx2 = list(range(len(body_verts), len(body_verts) + len(verts2)))
                body_verts.extend(verts2)
                
                body_top = createFaces(vertsIdx1, vertsIdx2, flipped=top)
                body_faces.extend(body_top)

                if top:
                    body_verts_inner_top.extend(vertsIdx1)
                    body_verts_outer_top.extend(vertsIdx2)

                    verts_bridge_start.append(vertsIdx1[0])
                    verts_bridge_start.append(vertsIdx2[0])
                    verts_bridge_end.append(vertsIdx1[-1])
                    verts_bridge_end.append(vertsIdx2[-1])
                    
                else:
                    body_verts_inner_bottom.extend(vertsIdx1)
                    body_verts_outer_bottom.extend(vertsIdx2)

                    verts_bridge_start.append(vertsIdx2[0])
                    verts_bridge_start.append(vertsIdx1[0])
                    verts_bridge_end.append(vertsIdx2[-1])
                    verts_bridge_end.append(vertsIdx1[-1])

                # tooth
                vertsIdx2 = list(range(len(tooth_verts), len(tooth_verts) + len(verts2[1:]))) # overate vertxIdx2
                tooth_verts.extend(verts2[1:])
                vertsIdx3 = list(range(len(tooth_verts), len(tooth_verts) + len(verts3)))
                tooth_verts.extend(verts3)
                vertsIdx4 = list(range(len(tooth_verts), len(tooth_verts) + len(verts4)))
                tooth_verts.extend(verts4)

                verts_outside = []
                verts_outside.append(vertsIdx2[0])
                verts_outside.append(vertsIdx3[0])
                verts_outside.extend(vertsIdx4)
                verts_outside.append(vertsIdx3[-1])
                verts_outside.append(vertsIdx2[-1])
                verts_outside.append(vertsIdx2[1])
                

                if top:
                    verts_outside_top = verts_outside
                else:
                    verts_outside_bottom = verts_outside


                faces_tooth_middle_top = createFaces(vertsIdx2, vertsIdx3,
                    flipped=top)
                faces_tooth_outer_top = createFaces(vertsIdx3, vertsIdx4,
                    flipped=top)

                tooth_faces.extend(faces_tooth_middle_top)
                tooth_faces.extend(faces_tooth_outer_top)

        # tooth outside
        faces_outside = createFaces(verts_outside_top, verts_outside_bottom, closed=True, flipped=True)
        tooth_faces.extend(faces_outside)

        # tooth mesh
        create_mesh_from_verts_faces(tooth_verts, tooth_faces, mesh_path_str=f"{gear_root}/tooth_{toothCnt}")
        
        if toothCnt == 0:
            verts_bridge_first = verts_bridge_start

        # Bridge one tooth to the next
        if verts_bridge_prev:
            faces_bridge = createFaces(verts_bridge_prev, verts_bridge_start)
            body_faces.extend(faces_bridge)

        # Remember "end" vertices for next tooth.
        verts_bridge_prev = verts_bridge_end

    # Bridge the first to the last tooth.
    faces_bridge_f_l = createFaces(verts_bridge_prev, verts_bridge_first)
    body_faces.extend(faces_bridge_f_l)

    body_outside_faces = createFaces(body_verts_outer_top, body_verts_outer_bottom, closed=True, flipped=False)
    body_faces.extend(body_outside_faces)

    body_inside_faces = createFaces(body_verts_inner_top, body_verts_inner_bottom, closed=True, flipped=False)
    body_faces.extend(body_inside_faces)

    # print("gear body", len(body_verts), len(body_faces), body_verts, body_faces)
    create_mesh_from_verts_faces(body_verts, body_faces, mesh_path_str=f"{gear_root}/cubo")

