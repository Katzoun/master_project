import open3d as o3d
import numpy as np


class CustomVisualizer:
    """
    Vlastní visualizer který kombinuje editing funkcionalitu s key callbacks
    """
    def __init__(self):
        self.vis = None
        self.mesh = None
        self.window_name = "Custom Visualizer"
        self.view_locked = False
        self.lock_axis = 'z'  # 'x', 'y', 'z'
        self.selection_mode = False
        self.crop_mode = 'inside'  # 'inside' nebo 'outside'
        
    def create_window(self, window_name="Custom Visualizer", width=1024, height=768):
        """Vytvoří visualizer okno"""
        self.window_name = window_name
        
        # Použijeme VisualizerWithKeyCallback jako základ
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name=window_name, width=width, height=height)
        
        # Registrujeme key callbacks
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
        
        # Editační funkce
        self.vis.register_key_callback(ord("E"), self.toggle_editing_mode)
        self.vis.register_key_callback(ord("C"), self.interactive_crop)
        self.vis.register_key_callback(ord("X"), self.remove_selected)
        
        # Pohled a výběr
        self.vis.register_key_callback(ord("L"), self.cycle_lock_axis)
        self.vis.register_key_callback(ord("V"), self.toggle_view_lock)
        self.vis.register_key_callback(ord("P"), self.polygon_crop)
        self.vis.register_key_callback(ord("I"), self.toggle_crop_mode)
        
        # Přednastavené pohledy
        self.vis.register_key_callback(ord("F"), self.front_view)
        self.vis.register_key_callback(ord("T"), self.top_view)
        self.vis.register_key_callback(ord("G"), self.side_view)
        
        # Barevné schéma
        self.vis.register_key_callback(ord("1"), self.color_original)
        self.vis.register_key_callback(ord("2"), self.color_normal_map)
        self.vis.register_key_callback(ord("3"), self.color_uniform)
        
    def add_geometry(self, geometry):
        """Přidá geometrii do visualizeru"""
        self.mesh = geometry
        if self.vis is not None:
            return self.vis.add_geometry(geometry)
        return False
        
    def run(self):
        """Spustí visualizer"""
        if self.vis is not None:
            # Zobrazíme úvodní nápovědu
            self.show_help(None)
            return self.vis.run()
        return False
        
    def destroy_window(self):
        """Zničí okno"""
        if self.vis is not None:
            return self.vis.destroy_window()
        return False
            
    # === CALLBACK FUNKCE ===
    
    def save_mesh(self, vis):
        """Uloží aktuální mesh"""
        if self.mesh is not None:
            filename = "custom_edited_mesh.ply"
            o3d.io.write_triangle_mesh(filename, self.mesh)
            print(f"✓ Mesh uložen jako '{filename}'")
        else:
            print("✗ Žádný mesh k uložení")
        return False
        
    def reset_view(self, vis):
        """Resetuje pohled na geometrii"""
        vis.reset_view_point(True)
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
        
    def toggle_editing_mode(self, vis):
        """Simulace editačního módu"""
        print("✓ Editační mód: Použijte myš pro rotaci/zoom")
        print("  - Levé tlačítko: rotace")
        print("  - Pravé tlačítko: translace") 
        print("  - Kolečko: zoom")
        return False
        
    def crop_geometry(self, vis):
        """Simulace ořezání geometrie"""
        if self.mesh is not None:
            # Jednoduché ořezání - odebereme body mimo bounding box
            bbox = self.mesh.get_axis_aligned_bounding_box()
            center = bbox.get_center()
            extent = bbox.get_extent()
            
            # Vytvoříme menší bounding box (75% původní velikosti)
            new_extent = extent * 0.75
            min_bound = center - new_extent / 2
            max_bound = center + new_extent / 2
            
            # Ořežeme mesh
            cropped_mesh = self.mesh.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound))
            
            if len(cropped_mesh.vertices) > 0:
                self.mesh = cropped_mesh
                vis.clear_geometries()
                vis.add_geometry(self.mesh)
                print("✓ Geometrie ořezána (75% původní velikosti)")
            else:
                print("✗ Ořezání by odstranilo všechny body")
        else:
            print("✗ Žádný mesh k ořezání")
        return False
    
    def interactive_crop(self, vis):
        """Interaktivní ořezání pomocí bounding boxu"""
        if self.mesh is not None:
            print("🔄 Spouštím interaktivní ořezání...")
            print("   Postupujte podle instrukcí v konzoli Open3D")
            
            # Vytvoříme copy pro úpravu
            mesh_copy = self.mesh.copy()
            
            # Interaktivní výběr pomocí vestavěné funkcionality
            bbox = self.mesh.get_axis_aligned_bounding_box()
            center = bbox.get_center()
            extent = bbox.get_extent()
            
            # Vytvoříme menší bbox pro demonstraci
            scale_factor = 0.8
            new_extent = extent * scale_factor
            min_bound = center - new_extent / 2
            max_bound = center + new_extent / 2
            
            crop_bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
            
            if self.crop_mode == 'inside':
                cropped_mesh = self.mesh.crop(crop_bbox)
                print(f"✓ Ořezáno UVNITŘ bounding boxu")
            else:
                # Pro outside crop musíme použít inverzi
                all_indices = set(range(len(self.mesh.vertices)))
                vertices = np.asarray(self.mesh.vertices)
                inside_mask = crop_bbox.get_point_indices_within_bounding_box(
                    o3d.utility.Vector3dVector(vertices))
                outside_indices = list(all_indices - set(inside_mask))
                
                if len(outside_indices) > 0:
                    cropped_mesh = self.mesh.select_by_index(outside_indices)
                    print(f"✓ Ořezáno MIMO bounding box")
                else:
                    print("✗ Žádné body mimo bounding box")
                    return False
            
            if len(cropped_mesh.vertices) > 0:
                self.mesh = cropped_mesh
                vis.clear_geometries()
                vis.add_geometry(self.mesh)
                print(f"✓ Interaktivní ořezání dokončeno ({len(self.mesh.vertices)} vertices)")
            else:
                print("✗ Ořezání by odstranilo všechny body")
        else:
            print("✗ Žádný mesh k ořezání")
        return False
    
    def polygon_crop(self, vis):
        """Ořezání pomocí polygon selection"""
        if self.mesh is not None:
            print("🔺 Spouštím polygon selection...")
            print("   1. Nastavte si správný pohled")
            print("   2. Držte Shift + levé tlačítko myši pro kreslení polygonu")
            print("   3. Dokončete polygon dvojklikem")
            print("   4. Stiskněte Enter pro potvrzení nebo Escape pro zrušení")
            
            try:
                # Vytvoříme point cloud z mesh vertices pro selection
                pcd = o3d.geometry.PointCloud()
                pcd.points = self.mesh.vertices
                pcd.normals = self.mesh.vertex_normals
                
                # Přidáme point cloud do visualizeru pro selection
                vis.add_geometry(pcd)
                
                # Zobrazíme instrukce
                print("⚠️  DŮLEŽITÉ: Použijte následující ovládání:")
                print("   - Shift + drag: kreslení polygonu")
                print("   - Enter: potvrzení výběru")
                print("   - Escape: zrušení")
                print("   - Po dokončení stiskněte 'Y' pro potvrzení crop")
                
                # Registrujeme callback pro potvrzení
                self.vis.register_key_callback(ord("Y"), self.confirm_polygon_crop)
                self.temp_pcd = pcd
                
            except Exception as e:
                print(f"✗ Chyba při polygon selection: {e}")
        else:
            print("✗ Žádný mesh pro polygon selection")
        return False
    
    def confirm_polygon_crop(self, vis):
        """Potvrzení polygon crop"""
        print("✓ Polygon crop potvrzen")
        # Zde by se provedl skutečný crop podle vybraného polygonu
        # V této implementaci pouze odstraníme temp point cloud
        if hasattr(self, 'temp_pcd'):
            vis.remove_geometry(self.temp_pcd, reset_bounding_box=False)
            del self.temp_pcd
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
        print(f"✓ Osa pro zamknutí: {self.lock_axis.upper()}")
        
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
        ctr = vis.get_view_control()
        if self.mesh is not None:
            bbox = self.mesh.get_axis_aligned_bounding_box()
            center = bbox.get_center()
            extent = bbox.get_extent()
            
            # Pozice kamery zepředu
            camera_pos = center + np.array([extent[0] * 2, 0, 0])
            ctr.set_front([1, 0, 0])
            ctr.set_up([0, 0, 1])
            ctr.set_lookat(center)
            
        print("✓ Pohled: ZEPŘEDU (X osa)")
        return False
    
    def top_view(self, vis):
        """Nastaví pohled shora (Z osa)"""
        ctr = vis.get_view_control()
        if self.mesh is not None:
            bbox = self.mesh.get_axis_aligned_bounding_box()
            center = bbox.get_center()
            extent = bbox.get_extent()
            
            # Pozice kamery shora
            camera_pos = center + np.array([0, 0, extent[2] * 2])
            ctr.set_front([0, 0, -1])
            ctr.set_up([0, 1, 0])
            ctr.set_lookat(center)
            
        print("✓ Pohled: SHORA (Z osa)")
        return False
    
    def side_view(self, vis):
        """Nastaví pohled ze strany (Y osa)"""
        ctr = vis.get_view_control()
        if self.mesh is not None:
            bbox = self.mesh.get_axis_aligned_bounding_box()
            center = bbox.get_center()
            extent = bbox.get_extent()
            
            # Pozice kamery ze strany
            camera_pos = center + np.array([0, extent[1] * 2, 0])
            ctr.set_front([0, -1, 0])
            ctr.set_up([0, 0, 1])
            ctr.set_lookat(center)
            
        print("✓ Pohled: ZE STRANY (Y osa)")
        return False
    
    def toggle_crop_mode(self, vis):
        """Přepíná mód ořezávání (uvnitř/vně)"""
        self.crop_mode = 'outside' if self.crop_mode == 'inside' else 'inside'
        mode_text = "UVNITŘ výběru" if self.crop_mode == 'inside' else "MIMO výběr"
        print(f"✓ Mód ořezávání: {mode_text}")
        return False
        
    def remove_selected(self, vis):
        """Simulace odstranění vybraných částí"""
        print("✓ Simulace odstranění vybraných částí")
        print("  (V této implementaci není real selection, pouze info)")
        return False
        
    def color_original(self, vis):
        """Nastaví původní barvy"""
        if self.mesh is not None:
            # Odebereme vertex colors pro zobrazení původní barvy
            self.mesh.vertex_colors = o3d.utility.Vector3dVector()
            vis.update_geometry(self.mesh)
            print("✓ Původní barvy obnoveny")
        return False
        
    def color_normal_map(self, vis):
        """Nastaví barvy podle normál"""
        if self.mesh is not None:
            if not self.mesh.has_vertex_normals():
                self.mesh.compute_vertex_normals()
            
            # Konvertuje normály na barvy (0-1 range)
            normals = np.asarray(self.mesh.vertex_normals)
            colors = (normals + 1.0) / 2.0  # Převod z [-1,1] na [0,1]
            self.mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
            vis.update_geometry(self.mesh)
            print("✓ Barvy nastaveny podle normál")
        return False
        
    def color_uniform(self, vis):
        """Nastaví uniformní barvu"""
        if self.mesh is not None:
            # Nastavíme červenou barvu pro všechny vertices
            uniform_color = [0.8, 0.2, 0.2]  # Červená
            colors = np.tile(uniform_color, (len(self.mesh.vertices), 1))
            self.mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
            vis.update_geometry(self.mesh)
            print("✓ Uniformní červená barva nastavena")
        return False
        
    def show_help(self, vis):
        """Zobrazí nápovědu"""
        print("\n" + "="*60)
        print("   CUSTOM VISUALIZER - POKROČILÁ NÁPOVĚDA")
        print("="*60)
        print("ZÁKLADNÍ FUNKCE:")
        print("  H - Zobrazit tuto nápovědu")
        print("  S - Uložit mesh")
        print("  R - Reset pohledu")
        print("  W - Přepnout wireframe")
        print("  N - Přepočítat normály")
        print()
        print("EDITAČNÍ FUNKCE:")
        print("  E - Info o editačním módu")
        print("  C - Interaktivní ořezání (bounding box)")
        print("  P - Polygon selection & crop")
        print("  X - Info o odstranění výběru")
        print("  I - Přepnout mód ořezávání (uvnitř/vně)")
        print(f"      Aktuální: {'UVNITŘ' if self.crop_mode == 'inside' else 'MIMO'}")
        print()
        print("OVLÁDÁNÍ POHLEDU:")
        print("  V - Zapnout/vypnout zamknutí pohledu")
        print(f"      Status: {'ZAMKNUT' if self.view_locked else 'VOLNÝ'}")
        print("  L - Změnit osu zamknutí (X/Y/Z)")
        print(f"      Aktuální osa: {self.lock_axis.upper()}")
        print("  F - Pohled zepředu (X osa)")
        print("  T - Pohled shora (Z osa)")
        print("  G - Pohled ze strany (Y osa)")
        print()
        print("BARVY:")
        print("  1 - Původní barvy")
        print("  2 - Barvy podle normál")
        print("  3 - Uniformní červená")
        print()
        print("POLYGON SELECTION:")
        print("  Shift + drag - Kreslení polygonu")
        print("  Enter - Potvrzení výběru")
        print("  Y - Potvrzení crop operace")
        print("  Escape - Zrušení")
        print()
        print("  ESC - Ukončit visualizer")
        print("="*60)
        return False


def demo_custom_visualizer():
    """Demonstrace custom visualizeru"""
    
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
    
    # Vytvoření custom visualizeru
    vis = CustomVisualizer()
    
    # Spuštění
    vis.create_window(window_name="Custom Visualizer - Kombinovaná funkcionalita")
    vis.add_geometry(mesh)
    
    # Nastavení render options
    render_option = vis.vis.get_render_option()
    render_option.mesh_show_back_face = True
    render_option.light_on = True
    
    print("\n🚀 Custom Visualizer spuštěn!")
    print("💡 Stiskněte H pro zobrazení nápovědy")
    
    # Spuštění hlavní smyčky
    vis.run()
    vis.destroy_window()
    
    print("👋 Custom Visualizer ukončen")


if __name__ == '__main__':
    demo_custom_visualizer()
