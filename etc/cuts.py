import numpy as np
import pyvista as pv
import pyvistaqt as pvqt

MESH_PATH = "frames/marching_mesh.ply"   # OBJ/STL/PLY…

# kolik řezů chceš (rovnoměrně mezi minZ–maxZ)
N_SLICES = 12

mesh = pv.read(MESH_PATH).triangulate()

# vygeneruj řezy rovnoběžné s rovinou XY (tedy podél osy Z)
slices = mesh.slice_along_axis(n=N_SLICES, axis='z')

# vykreslení: mesh poloprůhledný + křivky řezů jako čáry
pl = pvqt.BackgroundPlotter()
pl.add_mesh(mesh, color="lightgray", opacity=0.2, show_edges=False)
pl.show_axes()
pl.enable_trackball_style()
pl.show_bounds(grid=True, location='back')
# MultiBlock „slices“ obsahuje jednotlivé PolyData s čarami průniku
for i, s in enumerate(slices):
    pl.add_mesh(s, color="tomato", line_width=3, render_lines_as_tubes=True)

# pl.show()
pl.app.exec_()
 