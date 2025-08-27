import open3d as o3d
import numpy as np


class CompositeVisualizer:
    """
    Alternativní řešení pomocí kompozice - obsahuje obě visualizery
    """
    def __init__(self):
        self.editing_vis = o3d.visualization.VisualizerWithEditing()
        self.key_vis = o3d.visualization.VisualizerWithKeyCallback()
        self.current_vis = None
        self.mesh = None
        self.mode = "editing"  # "editing" nebo "viewing"
        
    def create_window_editing(self, **kwargs):
        """Vytvoří okno pro editační mód"""
        self.current_vis = self.editing_vis
        self.mode = "editing"
        return self.editing_vis.create_window(**kwargs)
        
    def create_window_viewing(self, **kwargs):
        """Vytvoří okno pro viewing mód s key callbacks"""
        self.current_vis = self.key_vis
        self.mode = "viewing"
        self.setup_key_callbacks()
        return self.key_vis.create_window(**kwargs)
        
    def switch_to_editing(self):
        """Přepne do editačního módu"""
        if self.mode != "editing":
            # Uložit aktuální stav
            if self.mesh is not None and self.current_vis is not None:
                self.current_vis.remove_geometry(self.mesh, reset_bounding_box=False)
                
            # Zničit aktuální okno
            self.current_vis.destroy_window()
            
            # Přepnout na editing visualizer
            self.current_vis = self.editing_vis
            self.mode = "editing"
            self.editing_vis.create_window(window_name="Editing Mode")
            
            # Přidat geometrii zpět
            if self.mesh is not None:
                self.editing_vis.add_geometry(self.mesh)
                
    def switch_to_viewing(self):
        """Přepne do viewing módu s key callbacks"""
        if self.mode != "viewing":
            # Uložit aktuální stav
            if self.mesh is not None and self.current_vis is not None:
                self.current_vis.remove_geometry(self.mesh, reset_bounding_box=False)
                
            # Zničit aktuální okno
            self.current_vis.destroy_window()
            
            # Přepnout na key callback visualizer
            self.current_vis = self.key_vis
            self.mode = "viewing"
            self.key_vis.create_window(window_name="Viewing Mode with Key Callbacks")
            self.setup_key_callbacks()
            
            # Přidat geometrii zpět
            if self.mesh is not None:
                self.key_vis.add_geometry(self.mesh)
        
    def setup_key_callbacks(self):
        """Nastavení klávesových zkratek pro viewing mód"""
        self.key_vis.register_key_callback(ord("E"), self.switch_to_editing_callback)
        self.key_vis.register_key_callback(ord("S"), self.save_mesh_callback)
        self.key_vis.register_key_callback(ord("R"), self.reset_view_callback)
        self.key_vis.register_key_callback(ord("W"), self.toggle_wireframe_callback)
        self.key_vis.register_key_callback(ord("H"), self.help_callback)
        
    def switch_to_editing_callback(self, vis):
        """Callback pro přepnutí do editačního módu"""
        print("Přepínám do editačního módu...")
        # Toto je trochu složitější, protože musíme ukončit aktuální run loop
        # Místo toho pouze zobrazíme zprávu
        print("Pro přepnutí do editačního módu ukončete aktuální okno a spusťte switch_to_editing()")
        return False
        
    def save_mesh_callback(self, vis):
        """Callback pro uložení meshe"""
        if self.mesh is not None:
            o3d.io.write_triangle_mesh("composite_edited_mesh.ply", self.mesh)
            print("Mesh uložen jako 'composite_edited_mesh.ply'")
        return False
        
    def reset_view_callback(self, vis):
        """Callback pro reset pohledu"""
        vis.reset_view_point(True)
        print("Pohled resetován")
        return False
        
    def toggle_wireframe_callback(self, vis):
        """Callback pro přepínání wireframe módu"""
        render_option = vis.get_render_option()
        render_option.mesh_show_wireframe = not render_option.mesh_show_wireframe
        print(f"Wireframe: {'ON' if render_option.mesh_show_wireframe else 'OFF'}")
        return False
        
    def help_callback(self, vis):
        """Callback pro zobrazení nápovědy"""
        print(f"\n=== NÁPOVĚDA ({self.mode.upper()} MÓD) ===")
        if self.mode == "viewing":
            print("E - Přepnout do editačního módu")
            print("S - Uložit mesh")
            print("R - Reset pohledu")
            print("W - Přepnout wireframe")
            print("H - Zobrazit nápovědu")
        print("ESC - Ukončit")
        print("========================\n")
        return False
        
    def add_geometry(self, geometry):
        """Přidá geometrii do aktuálního visualizeru"""
        if self.current_vis is not None:
            self.mesh = geometry
            return self.current_vis.add_geometry(geometry)
            
    def run(self):
        """Spustí aktuální visualizer"""
        if self.current_vis is not None:
            return self.current_vis.run()
            
    def destroy_window(self):
        """Zničí aktuální okno"""
        if self.current_vis is not None:
            return self.current_vis.destroy_window()


def demo_composite_visualizer():
    """Demonstrace composite visualizeru"""
    
    # Načtení meshe
    mesh = o3d.io.read_triangle_mesh("frames/marching_mesh.ply")
    
    if not mesh.has_vertex_normals():
        print("Počítám vertex normály...")
        mesh.compute_vertex_normals()
    
    # Vytvoření composite visualizeru
    vis = CompositeVisualizer()
    
    print("\n=== COMPOSITE VISUALIZER ===")
    print("1. Spustím viewing mód s key callbacks")
    print("2. Můžete přepnout do editačního módu")
    print("=============================\n")
    
    # Spuštění v viewing módu
    vis.create_window_viewing(window_name="Composite Visualizer - Viewing Mode")
    vis.add_geometry(mesh)
    vis.help_callback(None)  # Zobrazit nápovědu
    vis.run()
    vis.destroy_window()
    
    # Možnost spustit v editačním módu
    response = input("\nChcete spustit editační mód? (y/n): ")
    if response.lower() == 'y':
        vis.create_window_editing(window_name="Composite Visualizer - Editing Mode")
        vis.add_geometry(mesh)
        print("Editační mód spuštěn. Můžete editovat mesh.")
        vis.run()
        vis.destroy_window()


if __name__ == '__main__':
    demo_composite_visualizer()
