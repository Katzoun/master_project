import pyvista as pv
import numpy as np

mesh = pv.read("frames/marching_mesh.ply").triangulate()
mesh = mesh.compute_normals(point_normals=True, cell_normals=True,
                            auto_orient_normals=True, inplace=False)

picked = []  # (point, normal)

def on_pick(point):
    pid = mesh.find_closest_point(point)
    n = mesh["Normals"][pid] if "Normals" in mesh.point_data else (0,0,1)
    print("picked:", point, "normal:", n)
    picked.append((np.array(point), np.array(n)))

pl = pv.Plotter()
pl.add_mesh(mesh, show_edges=False)
pl.enable_point_picking(callback=on_pick, use_mesh=True,
                        show_message=True, show_point=True, color="red",
                        point_size=12, tolerance=0.025)
pl.show()

# po zavření okna máš v `picked` všechny kliky (souřadnice + normály)
