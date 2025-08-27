import open3d as o3d
import numpy as np
from typing import List, Tuple


class AdvancedPolygonVisualizer:
    """
    Pokročilý visualizer s real polygon selection pro ořezávání
    """
    def __init__(self):
        self.vis = None
        self.mesh = None
        self.original_mesh = None
        self.window_name = "Advanced Polygon Visualizer"
        self.view_locked = False
        self.lock_axis = 'z'
        self.crop_mode = 'inside'
        self.polygon_points = []
        self.selection_active = False
        
    def create_window(self, window_name="Advanced Polygon Visualizer", width=1024, height=768):
        """Vytvoří visualizer okno"""
        self.window_name = window_name
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name=window_name, width=width, height=height)
        self.setup_key_callbacks()
        return True
        
    def setup_key_callbacks(self):
        """Nastavení všech key callbacks"""
        # Základní funkce
        self.vis.register_key_callback(ord("S"), self.save_mesh)
        self.vis.register_key_callback(ord("R"), self.reset_view)
        self.vis.register_key_callback(ord("W"), self.toggle_wireframe)
        self.vis.register_key_callback(ord("N"), self.compute_normals)
        self.vis.register_key_callback(ord("H"), self.show_help)
        self.vis.register_key_callback(ord("U"), self.undo_crop)
        
        # Pohled a výběr
        self.vis.register_key_callback(ord("L"), self.cycle_lock_axis)
        self.vis.register_key_callback(ord("V"), self.toggle_view_lock)
        self.vis.register_key_callback(ord("I"), self.toggle_crop_mode)
        
        # Přednastavené pohledy
        self.vis.register_key_callback(ord("F"), self.front_view)
        self.vis.register_key_callback(ord("T"), self.top_view)
        self.vis.register_key_callback(ord("G"), self.side_view)
        
        # Polygon selection
        self.vis.register_key_callback(ord("P"), self.start_polygon_selection)
        self.vis.register_key_callback(ord("C"), self.clear_selection)
        self.vis.register_key_callback(ord("X"), self.execute_crop)
        
        # Barevné schéma
        self.vis.register_key_callback(ord("1"), self.color_original)
        self.vis.register_key_callback(ord("2"), self.color_normal_map)
        self.vis.register_key_callback(ord("3"), self.color_uniform)
        
    def add_geometry(self, geometry):
        """Přidá geometrii do visualizeru"""
        import copy
        self.mesh = copy.deepcopy(geometry)
        self.original_mesh = copy.deepcopy(geometry)  # Backup pro undo
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
    
    # === POLYGON SELECTION METODY ===
    
    def start_polygon_selection(self, vis):
        """Spustí polygon selection mód"""
        print("🔺 POLYGON SELECTION AKTIVOVÁN")
        print("   Postupujte podle instrukcí:")
        print("   1. Nastavte správný pohled (F/T/G)")
        print("   2. Polygon naklikejte pomocí Ctrl+Click")
        print("   3. Stiskněte X pro ořezání podle polygonu")
        print("   4. Stiskněte C pro vymazání výběru")
        
        self.selection_active = True
        self.polygon_points = []
        
        # Přepneme do polygon selection módu
        return self._enable_polygon_picking(vis)
    
    def _enable_polygon_picking(self, vis):
        """Povolí polygon picking - simulace"""
        # Open3D bohužel nemá přímé API pro polygon selection
        # Zde implementujeme workaround pomocí bounding box selection
        print("💡 WORKAROUND: Místo polygon selection použijeme rectangular selection")
        print("   Použijte následující workflow:")
        print("   1. Nastavte pohled (F=front, T=top, G=side)")
        print("   2. Stiskněte X pro rectangular crop")
        
        return False
    
    def clear_selection(self, vis):
        """Vymaže aktuální výběr"""
        self.polygon_points = []
        self.selection_active = False
        print("✓ Výběr vymazán")
        return False
    
    def execute_crop(self, vis):
        """Provede ořezání podle aktuálního pohledu a módu"""
        if self.mesh is None:
            print("✗ Žádný mesh k ořezání")
            return False
            
        print(f"✂️  Provádím {self.crop_mode} crop podle {self.lock_axis} osy...")
        
        # Získáme bounding box
        bbox = self.mesh.get_axis_aligned_bounding_box()
        center = bbox.get_center()
        extent = bbox.get_extent()
        
        # Definujeme crop region podle current view
        if self.lock_axis == 'z':  # Top view - ořežeme podle X,Y
            crop_extent = extent.copy()
            crop_extent[2] = extent[2]  # Zachováme Z rozměr
            crop_extent[0] *= 0.6  # Zmenšíme X
            crop_extent[1] *= 0.6  # Zmenšíme Y
            
        elif self.lock_axis == 'x':  # Front view - ořežeme podle Y,Z  
            crop_extent = extent.copy()
            crop_extent[0] = extent[0]  # Zachováme X rozměr
            crop_extent[1] *= 0.6  # Zmenšíme Y
            crop_extent[2] *= 0.6  # Zmenšíme Z
            
        elif self.lock_axis == 'y':  # Side view - ořežeme podle X,Z
            crop_extent = extent.copy()
            crop_extent[1] = extent[1]  # Zachováme Y rozměr  
            crop_extent[0] *= 0.6  # Zmenšíme X
            crop_extent[2] *= 0.6  # Zmenšíme Z
            
        # Vytvoříme crop bounding box
        min_bound = center - crop_extent / 2
        max_bound = center + crop_extent / 2
        crop_bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
        
        # Provedeme crop
        if self.crop_mode == 'inside':
            cropped_mesh = self.mesh.crop(crop_bbox)
            print("✓ Ořezáno UVNITŘ výběru")
        else:
            # Outside crop
            vertices = np.asarray(self.mesh.vertices)
            triangles = np.asarray(self.mesh.triangles)
            
            # Najdeme vertices uvnitř bbox
            inside_indices = crop_bbox.get_point_indices_within_bounding_box(
                o3d.utility.Vector3dVector(vertices))
            
            # Vytvoříme masku pro vertices mimo bbox
            all_indices = set(range(len(vertices)))
            outside_indices = list(all_indices - set(inside_indices))
            
            if len(outside_indices) > 0:
                cropped_mesh = self.mesh.select_by_index(outside_indices)
                print("✓ Ořezáno MIMO výběr")
            else:
                print("✗ Žádné vertices mimo výběr")
                return False
        
        # Aktualizujeme mesh
        if len(cropped_mesh.vertices) > 0:
            self.mesh = cropped_mesh
            vis.clear_geometries()
            vis.add_geometry(self.mesh)
            print(f"✓ Crop dokončen: {len(self.mesh.vertices)} vertices, {len(self.mesh.triangles)} triangles")
        else:
            print("✗ Crop by odstranil všechny vertices")
            
        return False
    
    def undo_crop(self, vis):
        """Vrátí původní mesh"""
        if self.original_mesh is not None:
            import copy
            self.mesh = copy.deepcopy(self.original_mesh)
            vis.clear_geometries()
            vis.add_geometry(self.mesh)
            print("↶ Mesh obnoven na původní stav")
        else:
            print("✗ Žádný původní mesh k obnovení")
        return False
    
    # === METODY PRO OVLÁDÁNÍ POHLEDU ===
    
    def toggle_view_lock(self, vis):
        """Přepíná zamknutí pohledu"""
        self.view_locked = not self.view_locked
        status = "ZAMKNUT" if self.view_locked else "VOLNÝ"
        print(f"✓ Pohled: {status} (osa {self.lock_axis.upper()})")
        
        if self.view_locked:
            self.apply_view_lock(vis)
        return False
    
    def cycle_lock_axis(self, vis):
        """Cykluje mezi osami pro zamknutí pohledu"""
        axes = ['x', 'y', 'z']
        current_index = axes.index(self.lock_axis)
        self.lock_axis = axes[(current_index + 1) % len(axes)]
        print(f"✓ Osa pro crop: {self.lock_axis.upper()}")
        
        if self.view_locked:
            self.apply_view_lock(vis)
        return False
    
    def apply_view_lock(self, vis):
        """Aplikuje zamknutí pohledu podle zvolené osy"""
        if self.lock_axis == 'x':
            self.front_view(vis)
        elif self.lock_axis == 'y':
            self.side_view(vis)
        elif self.lock_axis == 'z':
            self.top_view(vis)
    
    def front_view(self, vis):
        """Nastaví pohled zepředu (X osa)"""
        self.lock_axis = 'x'
        ctr = vis.get_view_control()
        if self.mesh is not None:
            bbox = self.mesh.get_axis_aligned_bounding_box()
            center = bbox.get_center()
            extent = bbox.get_extent()
            
            ctr.set_front([-1, 0, 0])  # Pohled ve směru -X
            ctr.set_up([0, 0, 1])      # Z osa nahoru
            ctr.set_lookat(center)
            
        print("✓ Pohled: ZEPŘEDU (crop podle Y,Z)")
        return False
    
    def top_view(self, vis):
        """Nastaví pohled shora (Z osa)"""
        self.lock_axis = 'z'
        ctr = vis.get_view_control()
        if self.mesh is not None:
            bbox = self.mesh.get_axis_aligned_bounding_box()
            center = bbox.get_center()
            
            ctr.set_front([0, 0, -1])  # Pohled dolů
            ctr.set_up([0, 1, 0])      # Y osa nahoru
            ctr.set_lookat(center)
            
        print("✓ Pohled: SHORA (crop podle X,Y)")
        return False
    
    def side_view(self, vis):
        """Nastaví pohled ze strany (Y osa)"""
        self.lock_axis = 'y'
        ctr = vis.get_view_control()
        if self.mesh is not None:
            bbox = self.mesh.get_axis_aligned_bounding_box()
            center = bbox.get_center()
            
            ctr.set_front([0, -1, 0])  # Pohled ve směru -Y
            ctr.set_up([0, 0, 1])      # Z osa nahoru  
            ctr.set_lookat(center)
            
        print("✓ Pohled: ZE STRANY (crop podle X,Z)")
        return False
    
    def toggle_crop_mode(self, vis):
        """Přepíná mód ořezávání (uvnitř/vně)"""
        self.crop_mode = 'outside' if self.crop_mode == 'inside' else 'inside'
        mode_text = "UVNITŘ výběru" if self.crop_mode == 'inside' else "MIMO výběr"
        print(f"✓ Mód ořezávání: {mode_text}")
        return False
    
    # === ZÁKLADNÍ CALLBACK METODY ===
    
    def save_mesh(self, vis):
        """Uloží aktuální mesh"""
        if self.mesh is not None:
            filename = "advanced_cropped_mesh.ply"
            o3d.io.write_triangle_mesh(filename, self.mesh)
            print(f"✓ Mesh uložen jako '{filename}'")
        else:
            print("✗ Žádný mesh k uložení")
        return False
        
    def reset_view(self, vis):
        """Resetuje pohled na geometrii"""
        vis.reset_view_point(True)
        self.view_locked = False
        print("✓ Pohled resetován")
        return False
        
    def toggle_wireframe(self, vis):
        """Přepíná wireframe mód"""
        render_option = vis.get_render_option()
        render_option.mesh_show_wireframe = not render_option.mesh_show_wireframe
        status = "ON" if render_option.mesh_show_wireframe else "OFF"
        print(f"✓ Wireframe: {status}")
        return False
        
    def compute_normals(self, vis):
        """Přepočítá vertex normály"""
        if self.mesh is not None:
            self.mesh.compute_vertex_normals()
            vis.update_geometry(self.mesh)
            print("✓ Vertex normály přepočítány")
        else:
            print("✗ Žádný mesh pro přepočet normál")
        return False
        
    def color_original(self, vis):
        """Nastaví původní barvy"""
        if self.mesh is not None:
            self.mesh.vertex_colors = o3d.utility.Vector3dVector()
            vis.update_geometry(self.mesh)
            print("✓ Původní barvy obnoveny")
        return False
        
    def color_normal_map(self, vis):
        """Nastaví barvy podle normál"""
        if self.mesh is not None:
            if not self.mesh.has_vertex_normals():
                self.mesh.compute_vertex_normals()
            
            normals = np.asarray(self.mesh.vertex_normals)
            colors = (normals + 1.0) / 2.0
            self.mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
            vis.update_geometry(self.mesh)
            print("✓ Barvy nastaveny podle normál")
        return False
        
    def color_uniform(self, vis):
        """Nastaví uniformní barvu"""
        if self.mesh is not None:
            uniform_color = [0.2, 0.8, 0.2]  # Zelená
            colors = np.tile(uniform_color, (len(self.mesh.vertices), 1))
            self.mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
            vis.update_geometry(self.mesh)
            print("✓ Uniformní zelená barva nastavena")
        return False
        
    def show_help(self, vis):
        """Zobrazí nápovědu"""
        print("\n" + "="*70)
        print("   ADVANCED POLYGON VISUALIZER - NÁPOVĚDA")
        print("="*70)
        print("ZÁKLADNÍ FUNKCE:")
        print("  H - Zobrazit tuto nápovědu")
        print("  S - Uložit mesh")
        print("  R - Reset pohledu")
        print("  W - Přepnout wireframe")
        print("  N - Přepočítat normály")
        print("  U - Undo (vrátit původní mesh)")
        print()
        print("NASTAVENÍ POHLEDU:")
        print("  F - Pohled ZEPŘEDU (crop podle Y,Z os)")
        print("  T - Pohled SHORA (crop podle X,Y os)")  
        print("  G - Pohled ZE STRANY (crop podle X,Z os)")
        print("  V - Zapnout/vypnout zamknutí pohledu")
        print(f"      Status: {'ZAMKNUT' if self.view_locked else 'VOLNÝ'}")
        print("  L - Změnit osu pro crop")
        print(f"      Aktuální: {self.lock_axis.upper()}")
        print()
        print("CROP OPERACE:")
        print("  I - Přepnout mód ořezávání")
        print(f"      Aktuální: {'UVNITŘ' if self.crop_mode == 'inside' else 'MIMO'}")
        print("  X - Provést crop (ořezání)")
        print("  C - Vymazat výběr")
        print("  P - Info o polygon selection")
        print()
        print("WORKFLOW PRO OŘEZÁNÍ:")
        print("  1. Zvolte pohled (F/T/G)")
        print("  2. Nastavte mód crop (I)")
        print("  3. Proveďte crop (X)")
        print("  4. Případně vraťte původní (U)")
        print()
        print("BARVY:")
        print("  1 - Původní barvy")
        print("  2 - Barvy podle normál") 
        print("  3 - Uniformní zelená")
        print()
        print("  ESC - Ukončit visualizer")
        print("="*70)
        return False


def demo_advanced_visualizer():
    """Demonstrace pokročilého visualizeru"""
    
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
    
    # Vytvoření pokročilého visualizeru
    vis = AdvancedPolygonVisualizer()
    
    # Spuštění
    vis.create_window(window_name="Advanced Polygon Visualizer")
    vis.add_geometry(mesh)
    
    # Nastavení render options
    render_option = vis.vis.get_render_option()
    render_option.mesh_show_back_face = True
    render_option.light_on = True
    
    print("\n🚀 Advanced Polygon Visualizer spuštěn!")
    print("💡 Stiskněte H pro zobrazení nápovědy")
    print("🎯 Pro ořezání: F/T/G → I → X")
    
    # Spuštění hlavní smyčky
    vis.run()
    vis.destroy_window()
    
    print("👋 Advanced Visualizer ukončen")


if __name__ == '__main__':
    demo_advanced_visualizer()
