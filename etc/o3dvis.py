import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

mesh = o3d.io.read_triangle_mesh("frames/marching_mesh.ply")
mesh.compute_vertex_normals()

app = gui.Application.instance
app.initialize()

win = o3d.visualization.O3DVisualizer("Pick demo", 1024, 768)
mat = rendering.MaterialRecord()
mat.shader = "defaultLit"
win.add_geometry("mesh", mesh, mat)

def on_picked(info: o3d.visualization.O3DVisualizer.PickInfo):
    if not info.is_valid:
        return
    # info.world_position: 3D souřadnice zásahu (x,y,z)
    # info.primitive_id: index trojúhelníku v meshi
    print("hit @", info.world_position, "triangle id:", info.primitive_id)

win.set_on_picked(on_picked)
win.show_settings = True  # vpravo panel; v toolbaru je i režim Pick
app.add_window(win)
app.run()
