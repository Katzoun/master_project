import open3d as o3d
import numpy as np


def demo_real_polygon_selection():
    """
    Demonstrace skutečné polygon selection v Open3D
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
    print("   REAL POLYGON SELECTION DEMO")
    print("="*60)
    print("Tento demo ukáže skutečnou polygon selection v Open3D")
    print("použitím draw_geometries_with_editing().")
    print()
    print("INSTRUKCE:")
    print("1. Otevře se okno s meshem")
    print("2. Nastavte si vhodný pohled")
    print("3. Držte Shift + levé tlačítko myši pro kreslení polygonu")
    print("4. Dokončete polygon dvojklikem")
    print("5. Stiskněte Del pro smazání vybraných částí")
    print("6. Nebo C pro crop (ponechání pouze vybraných částí)")
    print("7. Stiskněte Q nebo ESC pro ukončení")
    print("="*60)
    
    input("Stiskněte Enter pro pokračování...")
    
    # Použití vlastní editing functionality místo vestavěné
    print("🚀 Spouštím Custom Editing Interface...")
    
    # Import vlastní implementace
    from custom_editing_visualizer import custom_draw_geometries_with_editing
    
    # Nahrazení původního kódu vlastní implementací:
    # edited_geometry = o3d.visualization.draw_geometries_with_editing([mesh])
    edited_geometry = custom_draw_geometries_with_editing([mesh])
    
    if edited_geometry:
        edited_mesh = edited_geometry[0]
        print(f"✓ Editace dokončena: {len(edited_mesh.vertices)} vertices")
        
        # Uložíme editovaný mesh
        o3d.io.write_triangle_mesh("polygon_edited_mesh.ply", edited_mesh)
        print("✓ Editovaný mesh uložen jako 'polygon_edited_mesh.ply'")
        
        # Zobrazíme výsledek
        print("\n📊 POROVNÁNÍ:")
        print(f"   Původní: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")
        print(f"   Editovaný: {len(edited_mesh.vertices)} vertices, {len(edited_mesh.triangles)} triangles")
        
        # Nabídneme zobrazení výsledku
        response = input("\nChcete zobrazit editovaný mesh? (y/n): ")
        if response.lower() == 'y':
            o3d.visualization.draw_geometries([edited_mesh], 
                                            window_name="Editovaný mesh",
                                            width=800, height=600)
    else:
        print("✗ Editace byla zrušena nebo neproběhla")


def demo_manual_polygon_crop():
    """
    Demonstrace manuálního polygon crop workflow
    """
    
    # Načtení meshe
    mesh_path = "frames/marching_mesh.ply"
    try:
        mesh = o3d.io.read_triangle_mesh(mesh_path)
        print(f"✓ Mesh načten: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")
    except Exception as e:
        print(f"✗ Chyba při načítání meshe: {e}")
        return
    
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    
    print("\n" + "="*60)
    print("   MANUAL POLYGON CROP WORKFLOW")
    print("="*60)
    print("Tento workflow vám ukáže jak manuálně")
    print("implementovat polygon crop:")
    print()
    print("1. Konverze mesh na point cloud")
    print("2. Polygon selection na point cloud")
    print("3. Rekonstrukce mesh z vybraných bodů")
    print("="*60)
    
    input("Stiskněte Enter pro pokračování...")
    
    # Krok 1: Konverze na point cloud
    print("📍 Krok 1: Konverze mesh na point cloud...")
    pcd = o3d.geometry.PointCloud()
    pcd.points = mesh.vertices
    pcd.normals = mesh.vertex_normals
    
    # Přidáme barvy pro lepší vizualizaci
    colors = np.asarray(mesh.vertex_normals)
    colors = (colors + 1.0) / 2.0  # Normalizace do [0,1]
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    print(f"✓ Point cloud vytvořen: {len(pcd.points)} points")
    
    # Krok 2: Polygon selection
    print("\n📍 Krok 2: Polygon selection na point cloud...")
    print("INSTRUKCE pro polygon selection:")
    print("- Shift + drag: kreslení polygonu")
    print("- Enter: potvrzení výběru")
    print("- Del: smazání vybraných bodů")
    print("- C: crop (ponechání pouze vybraných bodů)")
    
    # Spustíme editing na point cloud
    edited_pcd_list = o3d.visualization.draw_geometries_with_editing([pcd])
    
    if edited_pcd_list and len(edited_pcd_list) > 0:
        edited_pcd = edited_pcd_list[0]
        print(f"✓ Point cloud editován: {len(edited_pcd.points)} points")
        
        # Krok 3: Rekonstrukce mesh
        print("\n📍 Krok 3: Rekonstrukce mesh...")
        
        if len(edited_pcd.points) > 100:  # Dostatek bodů pro rekonstrukci
            # Poisson surface reconstruction
            print("  Provádím Poisson surface reconstruction...")
            mesh_recon, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
                edited_pcd, depth=8, width=0, scale=1.1, linear_fit=False)
            
            # Vyčistíme mesh
            mesh_recon.remove_degenerate_triangles()
            mesh_recon.remove_duplicated_triangles()
            mesh_recon.remove_duplicated_vertices()
            mesh_recon.remove_non_manifold_edges()
            
            print(f"✓ Mesh rekonstruován: {len(mesh_recon.vertices)} vertices, {len(mesh_recon.triangles)} triangles")
            
            # Uložíme
            o3d.io.write_triangle_mesh("manual_polygon_crop.ply", mesh_recon)
            print("✓ Rekonstruovaný mesh uložen jako 'manual_polygon_crop.ply'")
            
            # Zobrazíme porovnání
            print("\n📊 VÝSLEDEK MANUAL POLYGON CROP:")
            print(f"   Původní mesh: {len(mesh.vertices)} vertices")
            print(f"   Po selection: {len(edited_pcd.points)} points")
            print(f"   Rekonstruovaný: {len(mesh_recon.vertices)} vertices")
            
            # Nabídneme zobrazení
            response = input("\nChcete zobrazit rekonstruovaný mesh? (y/n): ")
            if response.lower() == 'y':
                # Porovnání side-by-side
                mesh_recon.paint_uniform_color([0.8, 0.2, 0.2])  # Červený
                mesh.paint_uniform_color([0.2, 0.8, 0.2])       # Zelený
                
                print("🎨 Zobrazujem porovnání:")
                print("   Zelený = původní mesh")
                print("   Červený = rekonstruovaný mesh")
                
                o3d.visualization.draw_geometries([mesh, mesh_recon],
                                                window_name="Porovnání meshů",
                                                width=1200, height=800)
        else:
            print("✗ Příliš málo bodů pro rekonstrukci mesh")
    else:
        print("✗ Selection byla zrušena")


def main():
    """Hlavní menu"""
    while True:
        print("\n" + "="*50)
        print("   POLYGON SELECTION MENU")
        print("="*50)
        print("1. Real polygon selection (draw_geometries_with_editing)")
        print("2. Manual polygon crop workflow")
        print("3. Ukončit")
        print("="*50)
        
        choice = input("Vyberte možnost (1-3): ").strip()
        
        if choice == '1':
            demo_real_polygon_selection()
        elif choice == '2':
            demo_manual_polygon_crop()
        elif choice == '3':
            print("👋 Ukončuji...")
            break
        else:
            print("✗ Neplatná volba, zkuste znovu")


if __name__ == '__main__':
    main()
