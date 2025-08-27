import open3d as o3d
import numpy as np
import copy


class AdvancedCustomEditor:
    """
    Pokročilá vlastní implementace editing functionality s lepší polygon selection
    """
    def __init__(self):
        self.vis = None
        self.mesh = None
        self.original_mesh = None
        self.selected_vertices = set()
        self.crop_mode = 'inside'
        self.view_locked = False
        self.view_axis = 'z'
        
    def create_window(self, window_name="Advanced Custom Editor", width=1024, height=768):
        """Vytvoří okno"""
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name=window_name, width=width, height=height)
        self.setup_callbacks()
        return True
        
    def setup_callbacks(self):
        """Setup key callbacks"""
        # Nápověda a základní
        self.vis.register_key_callback(ord("H"), self.show_help)
        self.vis.register_key_callback(ord("S"), self.save_mesh)
        self.vis.register_key_callback(ord("U"), self.undo_all)
        self.vis.register_key_callback(ord("R"), self.reset_view)
        
        # Selection metody
        self.vis.register_key_callback(ord("B"), self.select_by_box)
        self.vis.register_key_callback(ord("P"), self.select_by_plane)
        self.vis.register_key_callback(ord("C"), self.clear_selection)
        self.vis.register_key_callback(ord("V"), self.visualize_selection)
        self.vis.register_key_callback(ord("I"), self.invert_selection)
        
        # Editing operace
        self.vis.register_key_callback(ord("X"), self.crop_by_selection)
        self.vis.register_key_callback(8, self.delete_selection)  # Backspace
        self.vis.register_key_callback(127, self.delete_selection)  # Delete
        
        # View controls
        self.vis.register_key_callback(ord("1"), self.set_top_view)
        self.vis.register_key_callback(ord("2"), self.set_front_view) 
        self.vis.register_key_callback(ord("3"), self.set_side_view)
        self.vis.register_key_callback(ord("L"), self.toggle_view_lock)
        
        # Modes
        self.vis.register_key_callback(ord("M"), self.toggle_crop_mode)
        
    def add_geometry(self, geometry):
        """Přidá geometrii"""
        self.mesh = copy.deepcopy(geometry)
        self.original_mesh = copy.deepcopy(geometry)
        if self.vis is not None:
            return self.vis.add_geometry(self.mesh)
        return False
    
    def run(self):
        """Spustí editor"""
        if self.vis is not None:
            self.show_help(None)
            return self.vis.run()
        return False
        
    def destroy_window(self):
        """Zničí okno"""
        if self.vis is not None:
            return self.vis.destroy_window()
        return False
    
    def get_edited_geometry(self):
        """Vrátí editovanou geometrii"""
        return [self.mesh] if self.mesh is not None else []
    
    # === SELECTION METODY ===
    
    def select_by_box(self, vis):
        """Výběr pomocí 3D bounding boxu"""
        if self.mesh is None:
            return False
            
        bbox = self.mesh.get_axis_aligned_bounding_box()
        center = bbox.get_center()
        extent = bbox.get_extent()
        
        # Vytvoříme menší selection box
        if self.view_axis == 'z':  # Top view - box v XY
            scale_x, scale_y, scale_z = 0.4, 0.4, 1.0
        elif self.view_axis == 'y':  # Front view - box v XZ
            scale_x, scale_y, scale_z = 0.4, 1.0, 0.4
        elif self.view_axis == 'x':  # Side view - box v YZ
            scale_x, scale_y, scale_z = 1.0, 0.4, 0.4
        else:
            scale_x, scale_y, scale_z = 0.5, 0.5, 0.5
            
        selection_extent = extent * np.array([scale_x, scale_y, scale_z])
        min_bound = center - selection_extent / 2
        max_bound = center + selection_extent / 2
        
        selection_box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
        
        # Najdeme vertices uvnitř boxu
        vertices = np.asarray(self.mesh.vertices)
        selected_indices = selection_box.get_point_indices_within_bounding_box(
            o3d.utility.Vector3dVector(vertices))
        
        self.selected_vertices = set(selected_indices)
        print(f"📦 Box selection: {len(self.selected_vertices)} vertices (view: {self.view_axis})")
        
        self.visualize_selection(vis)
        return False
    
    def select_by_plane(self, vis):
        """Výběr pomocí cutting plane"""
        if self.mesh is None:
            return False
            
        bbox = self.mesh.get_axis_aligned_bounding_box()
        center = bbox.get_center()
        vertices = np.asarray(self.mesh.vertices)
        
        # Definujeme cutting plane podle view axis
        if self.view_axis == 'z':  # Top view - horizontal plane
            # Vybereme vrchní polovinu
            selected_mask = vertices[:, 2] > center[2]
            plane_desc = "vrchní polovina (Z > center)"
        elif self.view_axis == 'y':  # Front view - depth plane
            selected_mask = vertices[:, 1] < center[1]
            plane_desc = "přední polovina (Y < center)"
        elif self.view_axis == 'x':  # Side view - lateral plane
            selected_mask = vertices[:, 0] > center[0]
            plane_desc = "pravá polovina (X > center)"
        else:
            # Default - Z plane
            selected_mask = vertices[:, 2] > center[2]
            plane_desc = "vrchní polovina"
            
        self.selected_vertices = set(np.where(selected_mask)[0])
        print(f"✂️  Plane selection: {len(self.selected_vertices)} vertices ({plane_desc})")
        
        self.visualize_selection(vis)
        return False
    
    def clear_selection(self, vis):
        """Vymaže selection"""
        self.selected_vertices.clear()
        if self.mesh is not None:
            self.mesh.vertex_colors = o3d.utility.Vector3dVector()
            vis.update_geometry(self.mesh)
        print("🧹 Selection vymazán")
        return False
    
    def visualize_selection(self, vis):
        """Zobrazí selection"""
        if not self.selected_vertices or self.mesh is None:
            return False
            
        # Vytvoříme color array
        colors = np.full((len(self.mesh.vertices), 3), [0.7, 0.7, 0.7])  # Šedá pro nevybrané
        
        # Obarvíme vybrané vertices
        for idx in self.selected_vertices:
            colors[idx] = [1.0, 0.2, 0.2]  # Červená pro vybrané
            
        self.mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
        vis.update_geometry(self.mesh)
        print(f"🎨 Zobrazeno: {len(self.selected_vertices)} vybraných vertices")
        return False
    
    def invert_selection(self, vis):
        """Invertuje selection"""
        if self.mesh is not None:
            all_indices = set(range(len(self.mesh.vertices)))
            self.selected_vertices = all_indices - self.selected_vertices
            print(f"🔄 Selection invertován: {len(self.selected_vertices)} vertices")
            self.visualize_selection(vis)
        return False
    
    # === EDITING OPERACE ===
    
    def crop_by_selection(self, vis):
        """Crop podle selection"""
        if not self.selected_vertices:
            print("⚠️  Žádný selection - provádím box selection")
            self.select_by_box(vis)
            
        if not self.selected_vertices:
            print("✗ Žádné vertices k crop")
            return False
            
        if self.crop_mode == 'inside':
            # Crop inside - ponecháme pouze vybrané
            selected_list = list(self.selected_vertices)
            cropped_mesh = self.mesh.select_by_index(selected_list)
            operation = f"CROP UVNITŘ ({len(selected_list)} vertices)"
        else:
            # Crop outside - ponecháme nevybrané
            all_indices = set(range(len(self.mesh.vertices)))
            outside_indices = list(all_indices - self.selected_vertices)
            
            if not outside_indices:
                print("✗ Crop outside by odstranil všechny vertices")
                return False
                
            cropped_mesh = self.mesh.select_by_index(outside_indices)
            operation = f"CROP MIMO ({len(outside_indices)} vertices)"
        
        if len(cropped_mesh.vertices) > 0:
            self.mesh = cropped_mesh
            vis.clear_geometries()
            vis.add_geometry(self.mesh)
            self.selected_vertices.clear()
            print(f"✓ {operation}")
            print(f"  Výsledek: {len(self.mesh.vertices)} vertices, {len(self.mesh.triangles)} triangles")
        else:
            print("✗ Crop by vytvořil prázdný mesh")
        return False
    
    def delete_selection(self, vis):
        """Smaže vybranou oblast"""
        if not self.selected_vertices:
            print("⚠️  Žádný selection - provádím box selection")
            self.select_by_box(vis)
            
        if not self.selected_vertices:
            print("✗ Žádné vertices k smazání")
            return False
        
        # Delete = ponechat nevybrané vertices
        all_indices = set(range(len(self.mesh.vertices)))
        remaining_indices = list(all_indices - self.selected_vertices)
        
        if not remaining_indices:
            print("✗ Delete by odstranil všechny vertices")
            return False
        
        deleted_count = len(self.selected_vertices)
        deleted_mesh = self.mesh.select_by_index(remaining_indices)
        
        if len(deleted_mesh.vertices) > 0:
            self.mesh = deleted_mesh
            vis.clear_geometries()
            vis.add_geometry(self.mesh)
            print(f"🗑️  Smazáno {deleted_count} vertices")
            print(f"   Zůstává: {len(self.mesh.vertices)} vertices, {len(self.mesh.triangles)} triangles")
            self.selected_vertices.clear()
        else:
            print("✗ Delete by vytvořil prázdný mesh")
        return False
    
    # === VIEW CONTROLS ===
    
    def set_top_view(self, vis):
        """Top view (XY rovina, Z osa)"""
        self.view_axis = 'z'
        ctr = vis.get_view_control()
        if self.mesh is not None:
            bbox = self.mesh.get_axis_aligned_bounding_box()
            center = bbox.get_center()
            ctr.set_front([0, 0, -1])
            ctr.set_up([0, 1, 0])
            ctr.set_lookat(center)
        print("🔝 TOP VIEW (XY rovina, Z selection)")
        return False
    
    def set_front_view(self, vis):
        """Front view (XZ rovina, Y osa)"""
        self.view_axis = 'y'
        ctr = vis.get_view_control()
        if self.mesh is not None:
            bbox = self.mesh.get_axis_aligned_bounding_box()
            center = bbox.get_center()
            ctr.set_front([0, 1, 0])
            ctr.set_up([0, 0, 1])
            ctr.set_lookat(center)
        print("👁️  FRONT VIEW (XZ rovina, Y selection)")
        return False
    
    def set_side_view(self, vis):
        """Side view (YZ rovina, X osa)"""
        self.view_axis = 'x'
        ctr = vis.get_view_control()
        if self.mesh is not None:
            bbox = self.mesh.get_axis_aligned_bounding_box()
            center = bbox.get_center()
            ctr.set_front([1, 0, 0])
            ctr.set_up([0, 0, 1])
            ctr.set_lookat(center)
        print("👀 SIDE VIEW (YZ rovina, X selection)")
        return False
    
    def toggle_view_lock(self, vis):
        """Přepne view lock"""
        self.view_locked = not self.view_locked
        status = "ZAMKNUT" if self.view_locked else "VOLNÝ"
        print(f"🔒 Pohled: {status}")
        return False
    
    def toggle_crop_mode(self, vis):
        """Přepne crop mode"""
        self.crop_mode = 'outside' if self.crop_mode == 'inside' else 'inside'
        mode_text = "UVNITŘ selection" if self.crop_mode == 'inside' else "MIMO selection"
        print(f"⚙️  Crop mód: {mode_text}")
        return False
    
    # === UTILITY ===
    
    def save_mesh(self, vis):
        """Uloží mesh"""
        if self.mesh is not None:
            filename = "advanced_custom_edited_mesh.ply"
            o3d.io.write_triangle_mesh(filename, self.mesh)
            print(f"💾 Mesh uložen: '{filename}'")
        return False
    
    def reset_view(self, vis):
        """Reset view"""
        vis.reset_view_point(True)
        self.view_locked = False
        print("🔄 Pohled resetován")
        return False
    
    def undo_all(self, vis):
        """Undo všech změn"""
        if self.original_mesh is not None:
            self.mesh = copy.deepcopy(self.original_mesh)
            vis.clear_geometries()
            vis.add_geometry(self.mesh)
            self.selected_vertices.clear()
            print("↶ UNDO: Obnovený původní mesh")
        return False
    
    def show_help(self, vis):
        """Zobrazí nápovědu"""
        print("\n" + "="*70)
        print("   ADVANCED CUSTOM EDITOR - NÁPOVĚDA")
        print("="*70)
        print("SELECTION METODY:")
        print("  B - Box selection (3D bounding box)")
        print("  P - Plane selection (cutting plane)")
        print("  C - Clear selection")
        print("  V - Visualize selection")
        print("  I - Invert selection")
        print(f"      Vybraných vertices: {len(self.selected_vertices)}")
        print()
        print("EDITING OPERACE:")
        print("  X - Crop podle selection")
        print("  Del/Backspace - Delete selected vertices")
        print("  M - Toggle crop mód")
        print(f"      Aktuální: {'UVNITŘ' if self.crop_mode == 'inside' else 'MIMO'}")
        print()
        print("VIEW CONTROLS:")
        print("  1 - Top view (XY rovina, Z selection)")
        print("  2 - Front view (XZ rovina, Y selection)")
        print("  3 - Side view (YZ rovina, X selection)")
        print("  L - Toggle view lock")
        print(f"      Aktuální view: {self.view_axis.upper()}")
        print()
        print("ZÁKLADNÍ:")
        print("  H - Nápověda")
        print("  S - Uložit mesh")
        print("  R - Reset view")
        print("  U - Undo všech změn")
        print()
        print("OPTIMÁLNÍ WORKFLOW:")
        print("  1. Nastavte view (1/2/3)")
        print("  2. Vyberte oblast (B/P)")
        print("  3. Nastavte crop mód (M)")
        print("  4. Proveďte operaci (X/Del)")
        print()
        print("  ESC - Ukončit editor")
        print("="*70)
        return False


def advanced_draw_geometries_with_editing(geometries, window_name="Advanced Custom Editor", width=1024, height=768):
    """
    Pokročilá vlastní implementace draw_geometries_with_editing
    """
    if not geometries:
        return None
    
    editor = AdvancedCustomEditor()
    
    try:
        editor.create_window(window_name=window_name, width=width, height=height)
        
        for geom in geometries:
            editor.add_geometry(geom)
            break  # Jedna geometrie pro jednoduchost
        
        print(f"🚀 Advanced Custom Editor spuštěn")
        print(f"   Stiskněte H pro nápovědu")
        
        editor.run()
        edited_geometries = editor.get_edited_geometry()
        editor.destroy_window()
        
        if edited_geometries:
            print(f"✓ Advanced editing dokončen")
            return edited_geometries
        else:
            return None
            
    except Exception as e:
        print(f"✗ Chyba: {e}")
        return None


# === DEMO ===

def demo_advanced_custom_editing():
    """Demo pokročilého custom editingu"""
    
    # Načtení meshe
    mesh_path = "frames/marching_mesh.ply"
    try:
        mesh = o3d.io.read_triangle_mesh(mesh_path)
        print(f"✓ Mesh načten: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")
    except Exception as e:
        print(f"✗ Chyba: {e}")
        return
    
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    
    print("\n" + "="*65)
    print("   ADVANCED CUSTOM EDITING DEMO")
    print("="*65)
    print("Pokročilá vlastní implementace s lepšími selection nástroji:")
    print()
    print("🔸 Box selection podle view axis")
    print("🔸 Plane selection (cutting plane)")
    print("🔸 Invert selection")
    print("🔸 Crop inside/outside")
    print("🔸 Delete vertices")
    print("🔸 Undo funkcionalita")
    print("🔸 View controls pro lepší workflow")
    print("="*65)
    
    input("Stiskněte Enter pro spuštění...")
    
    # Použití pokročilé implementace
    edited_geometry = advanced_draw_geometries_with_editing([mesh])
    
    if edited_geometry:
        edited_mesh = edited_geometry[0]
        print(f"\n✅ VÝSLEDEK ADVANCED EDITINGU:")
        print(f"   Původní: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")
        print(f"   Editovaný: {len(edited_mesh.vertices)} vertices, {len(edited_mesh.triangles)} triangles")
        
        # Uložení
        filename = "advanced_custom_edited.ply"
        o3d.io.write_triangle_mesh(filename, edited_mesh)
        print(f"   Uloženo: {filename}")
        
        # Zobrazení
        response = input("\nZobrazit výsledek? (y/n): ")
        if response.lower() == 'y':
            o3d.visualization.draw_geometries([edited_mesh],
                                            window_name="Advanced Custom Edited Mesh")
    else:
        print("✗ Editing zrušen")


if __name__ == '__main__':
    demo_advanced_custom_editing()
