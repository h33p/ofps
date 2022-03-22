import bpy, os
import mathutils
import math

scene = bpy.context.scene
camera = scene.camera

out = []

out.append("frame,fov_x,fov_y,rot_w,rot_i,rot_j,rot_k,pos_x,pos_y,pos_z")

aspect = float(scene.render.resolution_x) / float(scene.render.resolution_y)
aspect *= scene.render.pixel_aspect_x / scene.render.pixel_aspect_y

start_frame = scene.frame_current

start_angle = None

for i in range(scene.frame_start, scene.frame_end):
    scene.frame_set(i)
    fov = camera.data.angle
    if aspect < 1.0:
        fov_x = fov * aspect
        fov_y = fov
    else:
        fov_x = fov
        fov_y = fov / aspect
    if camera.rotation_mode == 'QUATERNION':
        rot = camera.rotation_quaternion
    else:
        rot = camera.rotation_euler.to_quaternion()

    if start_angle is None:
        start_angle = rot

    rot = start_angle.rotation_difference(rot)

    pos = camera.location
    out.append("{},{:.3f},{:.3f},{:.5f},{:.5f},{:.5f},{:.5f},{:.3f},{:.3f},{:.3f}".format(i, fov_x, fov_y, rot.w, rot.x, -rot.z, rot.y, pos.x, pos.y, pos.z))

with open(bpy.data.filepath + ".cam.csv", "w") as f:
    f.write("\n".join(out))

scene.frame_set(start_frame)
