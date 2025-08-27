import open3d as o3d
import numpy as np


class CombinedVisualizer(o3d.visualization.VisualizerWithEditing, 
                        o3d.visualization.VisualizerWithKeyCallback):
    """
    Kombinuje funkcionalitu VisualizerWithEditing a VisualizerWithKeyCallback
    """
    def __init__(self):
        # Inicializace obou parent tříd
        super().__init__()
        self.mesh = None
        
    def setup_key_callbacks(self):
        """Nastavení klávesových zkratek"""
        # Registrace callback funkcí pro různé klávesy
        self.register_key_callback(ord("S"), self.save_mesh_callback)
        self.register_key_callback(ord("R"), self.reset_view_callback)
        self.register_key_callback(ord("W"), self.toggle_wireframe_callback)
        self.register_key_callback(ord("N"), self.compute_normals_callback)
        self.register_key_callback(ord("H"), self.help_callback)
        
    def save_mesh_callback(self, vis):
        """Callback pro uložení meshe (klávesa S)"""
        if self.mesh is not None:
            o3d.io.write_triangle_mesh("edited_mesh.ply", self.mesh)
            print("Mesh uložen jako 'edited_mesh.ply'")
        return False
        
    def reset_view_callback(self, vis):
        """Callback pro reset pohledu (klávesa R)"""
        vis.reset_view_point(True)
        print("Pohled resetován")
        return False
        
    def toggle_wireframe_callback(self, vis):
        """Callback pro přepínání wireframe módu (klávesa W)"""
        render_option = vis.get_render_option()
        render_option.mesh_show_wireframe = not render_option.mesh_show_wireframe
        print(f"Wireframe: {'ON' if render_option.mesh_show_wireframe else 'OFF'}")
        return False
        
    def compute_normals_callback(self, vis):
        """Callback pro přepočítání normál (klávesa N)"""
        if self.mesh is not None:
            self.mesh.compute_vertex_normals()
            vis.update_geometry(self.mesh)
            print("Normály přepočítány")
        return False
        
    def help_callback(self, vis):
        """Callback pro zobrazení nápovědy (klávesa H)"""
        print("\n=== NÁPOVĚDA ===")
        print("S - Uložit mesh")
        print("R - Reset pohledu")
        print("W - Přepnout wireframe")
        print("N - Přepočítat normály")
        print("H - Zobrazit nápovědu")
        print("ESC - Ukončit")
        print("================\n")
        return False


def demo_combined_visualizer():
    """Demonstrace kombinovaného visualizeru"""
    
    # Načtení meshe
    mesh = o3d.io.read_triangle_mesh("frames/marching_mesh.ply")
    
    # Kontrola a výpočet normál
    if not mesh.has_vertex_normals():
        print("Počítám vertex normály...")
        mesh.compute_vertex_normals()
    
    # Vytvoření kombinovaného visualizeru
    vis = CombinedVisualizer()
    vis.mesh = mesh  # Uložení reference na mesh
    
    # Vytvoření okna a přidání geometrie
    vis.create_window(window_name="Combined Visualizer - Editing + Key Callbacks", 
                      width=1024, height=768)
    vis.add_geometry(mesh)
    
    # Nastavení render options
    render_option = vis.get_render_option()
    render_option.mesh_show_back_face = True
    
    # Registrace key callbacks
    vis.setup_key_callbacks()
    
    # Zobrazení nápovědy
    print("\n=== KOMBINOVANÝ VISUALIZER ===")
    print("Tento visualizer kombinuje editační nástroje s key callbacks")
    print("Stiskněte H pro nápovědu")
    print("==============================\n")
    
    # Spuštění visualizeru
    vis.run()
    vis.destroy_window()


if __name__ == '__main__':
    demo_combined_visualizer()
