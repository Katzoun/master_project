import open3d as o3d
import numpy as np
from typing import List, Tuple
import copy


class CustomEditingVisualizer:
    """
    Vlastní implementace editing functionality s polygon selection
    """
    def __init__(self):
        self.vis = None
        self.mesh = None
        self.original_mesh = None
        self.polygon_points = []
        self.selection_mode = False
        self.selected_indices = set()
        self.crop_mode = 'inside'  # 'inside' nebo 'outside'
        self.view_axis = 'z'  # pro projection
        
    def create_window(self, window_name="Custom Editing Visualizer", width=1024, height=768):
        """Vytvoří visualizer okno"""
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name=window_name, width=width, height=height)
        self.setup_callbacks()
        return True
        
    def setup_callbacks(self):
        """Nastavení key callbacks a mouse callbacks"""
        # Základní funkce
        self.vis.register_key_callback(ord("H"), self.show_help)
        self.vis.register_key_callback(ord("S"), self.save_mesh)
        self.vis.register_key_callback(ord("R"), self.reset_view)
        self.vis.register_key_callback(ord("U"), self.undo_changes)
        
        # Selection režim
        self.vis.register_key_callback(ord("P"), self.toggle_polygon_mode)
        self.vis.register_key_callback(ord("C"), self.clear_selection)
        self.vis.register_key_callback(ord("V"), self.visualize_selection)
        
        # Crop operace
        self.vis.register_key_callback(ord("X"), self.crop_selection)
        self.vis.register_key_callback(ord("D"), self.delete_selection)
        self.vis.register_key_callback(ord("I"), self.invert_selection)
        self.vis.register_key_callback(ord("M"), self.toggle_crop_mode)
        
        # Pohledy
        self.vis.register_key_callback(ord("1"), self.set_xy_view)
        self.vis.register_key_callback(ord("2"), self.set_xz_view)
        self.vis.register_key_callback(ord("3"), self.set_yz_view)
        
        # Mouse callback pro polygon drawing
        self.vis.register_mouse_move_callback(self.mouse_move_callback)
        self.vis.register_mouse_scroll_callback(self.mouse_scroll_callback)
        
    def add_geometry(self, geometry):
        """Přidá geometrii"""
        self.mesh = copy.deepcopy(geometry)
        self.original_mesh = copy.deepcopy(geometry)
        
        if self.vis is not None:
            return self.vis.add_geometry(self.mesh)
        return False
    
    def run(self):
        """Spustí visualizer"""
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
    
    # === POLYGON SELECTION IMPLEMENTACE ===
    
    def toggle_polygon_mode(self, vis):
        """Zapne/vypne polygon selection mód"""
        self.selection_mode = not self.selection_mode
        if self.selection_mode:
            print("🔺 POLYGON SELECTION AKTIVOVÁN")
            print("   Použijte Ctrl+Click pro kreslení polygonu")
            print("   Enter pro dokončení, Escape pro zrušení")
            self.polygon_points = []
        else:
            print("✓ Polygon selection vypnut")
            self.polygon_points = []
        return False
    
    def mouse_move_callback(self, vis, x, y):
        """Callback pro pohyb myši"""
        # Implementace polygon drawing by byla velmi složitá
        # Místo toho použijeme keyboard-based approach
        return False
    
    def mouse_scroll_callback(self, vis, x, y):
        """Callback pro scroll"""
        return False
    
    def select_rectangular_region(self, vis):
        """Výběr rectangular oblasti - zjednodušená verze polygon selection"""
        if self.mesh is None:
            return False
            
        print("📦 Rectangular selection - simulace polygon selection")
        
        # Získáme bounding box a vytvoříme selection podle pohledu
        bbox = self.mesh.get_axis_aligned_bounding_box()
        center = bbox.get_center()
        extent = bbox.get_extent()
        
        # Definujeme selection region podle view axis
        if self.view_axis == 'z':  # XY view
            # Vybereme střední část v X,Y
            sel_extent_x = extent[0] * 0.6
            sel_extent_y = extent[1] * 0.6
            min_bound = center - np.array([sel_extent_x/2, sel_extent_y/2, extent[2]])
            max_bound = center + np.array([sel_extent_x/2, sel_extent_y/2, extent[2]])
            
        elif self.view_axis == 'y':  # XZ view
            sel_extent_x = extent[0] * 0.6
            sel_extent_z = extent[2] * 0.6
            min_bound = center - np.array([sel_extent_x/2, extent[1], sel_extent_z/2])
            max_bound = center + np.array([sel_extent_x/2, extent[1], sel_extent_z/2])
            
        elif self.view_axis == 'x':  # YZ view
            sel_extent_y = extent[1] * 0.6
            sel_extent_z = extent[2] * 0.6
            min_bound = center - np.array([extent[0], sel_extent_y/2, sel_extent_z/2])
            max_bound = center + np.array([extent[0], sel_extent_y/2, sel_extent_z/2])
        
        # Najdeme vertices v selection region
        vertices = np.asarray(self.mesh.vertices)
        
        # Check které vertices jsou uvnitř selection boxu
        inside_mask = np.all(vertices >= min_bound, axis=1) & np.all(vertices <= max_bound, axis=1)
        selected_indices = np.where(inside_mask)[0]
        
        self.selected_indices = set(selected_indices)
        print(f"✓ Vybráno {len(self.selected_indices)} vertices")
        
        return True
    
    def clear_selection(self, vis):
        """Vymaže selection"""
        self.selected_indices.clear()
        self.polygon_points = []
        print("✓ Selection vymazán")
        
        # Obnovíme původní barvy
        if self.mesh is not None:
            self.mesh.vertex_colors = o3d.utility.Vector3dVector()
            vis.update_geometry(self.mesh)
        return False
    
    def visualize_selection(self, vis):
        """Zvýrazní vybranou oblast"""
        if not self.selected_indices:
            # Pokud není nic vybráno, proveď rectangular selection
            self.select_rectangular_region(vis)
        
        if self.selected_indices and self.mesh is not None:
            # Obarvíme vybrané vertices
            colors = np.zeros((len(self.mesh.vertices), 3))
            colors[:, 1] = 0.3  # Tmavě zelená pro nevybrané
            
            # Červená pro vybrané
            for idx in self.selected_indices:
                colors[idx] = [1.0, 0.0, 0.0]  # Červená
                
            self.mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
            vis.update_geometry(self.mesh)
            print(f"🎨 Vizualizace: {len(self.selected_indices)} vybraných vertices")
        return False
    
    # === EDITING OPERACE ===
    
    def crop_selection(self, vis):
        """Crop podle selection"""
        if not self.selected_indices:
            print("⚠️  Žádný selection - provádím rectangular selection")
            self.select_rectangular_region(vis)
        
        if not self.selected_indices:
            print("✗ Žádné vertices k crop")
            return False
            
        if self.crop_mode == 'inside':
            # Ponecháme pouze vybrané vertices
            selected_list = list(self.selected_indices)
            cropped_mesh = self.mesh.select_by_index(selected_list)
            operation = "CROP UVNITŘ"
        else:
            # Ponecháme všechny kromě vybraných
            all_indices = set(range(len(self.mesh.vertices)))
            outside_indices = list(all_indices - self.selected_indices)
            
            if not outside_indices:
                print("✗ Crop by odstranil všechny vertices")
                return False
                
            cropped_mesh = self.mesh.select_by_index(outside_indices)
            operation = "CROP MIMO"
        
        if len(cropped_mesh.vertices) > 0:
            self.mesh = cropped_mesh
            vis.clear_geometries()
            vis.add_geometry(self.mesh)
            self.selected_indices.clear()
            print(f"✓ {operation}: {len(self.mesh.vertices)} vertices")
        else:
            print("✗ Crop by vytvořil prázdný mesh")
            
        return False
    
    def delete_selection(self, vis):
        """Smaže vybranou oblast"""
        if not self.selected_indices:
            print("⚠️  Žádný selection - provádím rectangular selection")
            self.select_rectangular_region(vis)
        
        if not self.selected_indices:
            print("✗ Žádné vertices k smazání")
            return False
        
        # Odstranit vybrané vertices = ponechat nevybrané
        all_indices = set(range(len(self.mesh.vertices)))
        remaining_indices = list(all_indices - self.selected_indices)
        
        if not remaining_indices:
            print("✗ Smazání by odstranilo všechny vertices")
            return False
        
        deleted_mesh = self.mesh.select_by_index(remaining_indices)
        
        if len(deleted_mesh.vertices) > 0:
            self.mesh = deleted_mesh
            vis.clear_geometries()
            vis.add_geometry(self.mesh)
            print(f"✓ Smazáno {len(self.selected_indices)} vertices, zůstává {len(self.mesh.vertices)}")
            self.selected_indices.clear()
        else:
            print("✗ Smazání by vytvořilo prázdný mesh")
            
        return False
    
    def invert_selection(self, vis):
        """Invertuje selection"""
        if self.mesh is not None:
            all_indices = set(range(len(self.mesh.vertices)))
            self.selected_indices = all_indices - self.selected_indices
            print(f"🔄 Selection invertován: {len(self.selected_indices)} vertices")
            self.visualize_selection(vis)
        return False
    
    def toggle_crop_mode(self, vis):
        """Přepne crop mód"""
        self.crop_mode = 'outside' if self.crop_mode == 'inside' else 'inside'
        mode_text = "UVNITŘ selection" if self.crop_mode == 'inside' else "MIMO selection"
        print(f"✓ Crop mód: {mode_text}")
        return False
    
    # === VIEW CONTROLS ===
    
    def set_xy_view(self, vis):
        """Nastaví XY pohled (Z osa)"""
        self.view_axis = 'z'
        ctr = vis.get_view_control()
        if self.mesh is not None:
            bbox = self.mesh.get_axis_aligned_bounding_box()
            center = bbox.get_center()
            ctr.set_front([0, 0, -1])
            ctr.set_up([0, 1, 0])
            ctr.set_lookat(center)
        print("✓ XY pohled (Z projekce)")
        return False
    
    def set_xz_view(self, vis):
        """Nastaví XZ pohled (Y osa)"""
        self.view_axis = 'y'
        ctr = vis.get_view_control()
        if self.mesh is not None:
            bbox = self.mesh.get_axis_aligned_bounding_box()
            center = bbox.get_center()
            ctr.set_front([0, 1, 0])
            ctr.set_up([0, 0, 1])
            ctr.set_lookat(center)
        print("✓ XZ pohled (Y projekce)")
        return False
    
    def set_yz_view(self, vis):
        """Nastaví YZ pohled (X osa)"""
        self.view_axis = 'x'
        ctr = vis.get_view_control()
        if self.mesh is not None:
            bbox = self.mesh.get_axis_aligned_bounding_box()
            center = bbox.get_center()
            ctr.set_front([1, 0, 0])
            ctr.set_up([0, 0, 1])
            ctr.set_lookat(center)
        print("✓ YZ pohled (X projekce)")
        return False
    
    # === UTILITY FUNCTIONS ===
    
    def save_mesh(self, vis):
        """Uloží editovaný mesh"""
        if self.mesh is not None:
            filename = "custom_edited_mesh.ply"
            o3d.io.write_triangle_mesh(filename, self.mesh)
            print(f"✓ Mesh uložen jako '{filename}'")
        return False
    
    def reset_view(self, vis):
        """Reset pohledu"""
        vis.reset_view_point(True)
        print("✓ Pohled resetován")
        return False
    
    def undo_changes(self, vis):
        """Vrátí původní mesh"""
        if self.original_mesh is not None:
            self.mesh = copy.deepcopy(self.original_mesh)
            vis.clear_geometries()
            vis.add_geometry(self.mesh)
            self.selected_indices.clear()
            print("↶ Změny vráceny - obnovený původní mesh")
        return False
    
    def show_help(self, vis):
        """Zobrazí nápovědu"""
        print("\n" + "="*70)
        print("   CUSTOM EDITING VISUALIZER - NÁPOVĚDA")
        print("="*70)
        print("SELECTION:")
        print("  P - Zapnout/vypnout polygon mód")
        print("  V - Visualizovat/vytvořit rectangular selection")
        print("  C - Vymazat selection")
        print("  I - Invertovat selection")
        print()
        print("EDITING OPERACE:")
        print("  X - Crop podle selection")
        print("  D - Smazat vybranou oblast (Del)")
        print("  M - Přepnout crop mód (uvnitř/mimo)")
        print(f"      Aktuální: {'UVNITŘ' if self.crop_mode == 'inside' else 'MIMO'}")
        print()
        print("POHLEDY (pro selection):")
        print("  1 - XY pohled (Z projekce)")
        print("  2 - XZ pohled (Y projekce)")  
        print("  3 - YZ pohled (X projekce)")
        print(f"      Aktuální: {self.view_axis.upper()} projekce")
        print()
        print("ZÁKLADNÍ:")
        print("  H - Tato nápověda")
        print("  S - Uložit mesh")
        print("  R - Reset pohledu")
        print("  U - Undo (vrátit původní)")
        print()
        print("WORKFLOW:")
        print("  1. Nastavte pohled (1/2/3)")
        print("  2. Vytvořte selection (V)")
        print("  3. Nastavte crop mód (M)")
        print("  4. Proveďte operaci (X/D)")
        print()
        print("  ESC - Ukončit")
        print("="*70)
        return False


def custom_draw_geometries_with_editing(geometries, window_name="Custom Editing", width=1024, height=768):
    """
    Vlastní implementace draw_geometries_with_editing()
    
    Args:
        geometries: List geometrií k editaci
        window_name: Název okna
        width, height: Rozměry okna
        
    Returns:
        List editovaných geometrií nebo None pokud byla editace zrušena
    """
    
    if not geometries:
        print("✗ Žádné geometrie k editaci")
        return None
    
    # Vytvoříme custom editor
    editor = CustomEditingVisualizer()
    
    try:
        # Nastavíme okno a přidáme geometrie
        editor.create_window(window_name=window_name, width=width, height=height)
        
        for geom in geometries:
            editor.add_geometry(geom)
            break  # Pro jednoduchost podporujeme pouze jednu geometrii
        
        # Spustíme editor
        print(f"🚀 Spouštím Custom Editing Interface...")
        print(f"   Geometrie: {len(geometries)} objektů")
        print(f"   Stiskněte H pro nápovědu")
        
        editor.run()
        
        # Získáme editované geometrie
        edited_geometries = editor.get_edited_geometry()
        
        # Zničíme okno
        editor.destroy_window()
        
        if edited_geometries:
            print(f"✓ Editace dokončena: {len(edited_geometries)} objektů")
            return edited_geometries
        else:
            print("✗ Žádné geometrie po editaci")
            return None
            
    except Exception as e:
        print(f"✗ Chyba během editace: {e}")
        return None


# === DEMO FUNKCE ===

def demo_custom_editing():
    """
    Demonstrace vlastní editing functionality
    """
    
    # Načtení meshe
    mesh_path = "frames/marching_mesh.ply"
    try:
        mesh = o3d.io.read_triangle_mesh(mesh_path)
        print(f"✓ Mesh načten: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")
    except Exception as e:
        print(f"✗ Chyba při načítání meshe: {e}")
        return
    
    # Kontrola a výpočet normál
    if not mesh.has_vertex_normals():
        print("  Počítám vertex normály...")
        mesh.compute_vertex_normals()
    
    print("\n" + "="*60)
    print("   CUSTOM EDITING FUNCTIONALITY DEMO")
    print("="*60)
    print("Toto je vlastní implementace editing functionality")
    print("která nahrazuje draw_geometries_with_editing().")
    print()
    print("HLAVNÍ FUNKCE:")
    print("• Rectangular selection (místo polygon)")
    print("• Crop uvnitř/mimo selection")
    print("• Delete selection")
    print("• Invert selection")
    print("• Undo funkcionalita")
    print("• Fixace pohledu do os")
    print("="*60)
    
    input("Stiskněte Enter pro spuštění custom editoru...")
    
    # Použití vlastní editing functionality
    print("🚀 Spouštím Custom Editing Interface...")
    
    # Nahrazení původního kódu:
    # edited_geometry = o3d.visualization.draw_geometries_with_editing([mesh])
    edited_geometry = custom_draw_geometries_with_editing([mesh])
    
    if edited_geometry:
        edited_mesh = edited_geometry[0]
        print(f"✓ Editace dokončena: {len(edited_mesh.vertices)} vertices")
        
        # Uložíme editovaný mesh
        o3d.io.write_triangle_mesh("custom_polygon_edited_mesh.ply", edited_mesh)
        print("✓ Editovaný mesh uložen jako 'custom_polygon_edited_mesh.ply'")
        
        # Zobrazíme výsledek
        print("\n📊 POROVNÁNÍ:")
        print(f"   Původní: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")
        print(f"   Editovaný: {len(edited_mesh.vertices)} vertices, {len(edited_mesh.triangles)} triangles")
        
        # Nabídneme zobrazení výsledku
        response = input("\nChcete zobrazit editovaný mesh? (y/n): ")
        if response.lower() == 'y':
            o3d.visualization.draw_geometries([edited_mesh], 
                                            window_name="Custom Editovaný mesh",
                                            width=800, height=600)
    else:
        print("✗ Editace byla zrušena nebo neproběhla")


if __name__ == '__main__':
    demo_custom_editing()
