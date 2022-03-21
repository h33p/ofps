import bpy, os

scene = bpy.context.scene
camera = scene.camera

out = []

out.append("frame,fov_x,fov_y,rot_w,rot_i,rot_j,rot_k,pos_x,pos_y,pos_z")

aspect = float(scene.render.resolution_x) / float(scene.render.resolution_y)
aspect *= scene.render.pixel_aspect_x / scene.render.pixel_aspect_y

start_frame = scene.frame_current

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
    pos = camera.location
    out.append("{},{:.3f},{:.3f},{:.5f},{:.5f},{:.5f},{:.5f},{:.3f},{:.3f},{:.3f}".format(i, fov_x, fov_y, rot.w, rot.x, rot.y, rot.z, pos.x, pos.y, pos.z))

with open(bpy.data.filepath + ".cam.csv", "w") as f:
    f.write("\n".join(out))

scene.frame_set(start_frame)
