# Image Processing
# import point cloud captured from photoneo camera in point cloud .ply
import sys
import os
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import struct
from plyfile import PlyData, PlyElement

print(os.getcwd())


if __name__ == "__main__":

    frames_dir = "/home/robolab2/master_project/frames"

    # List all .ply files in the directory
    ply_files = [f for f in os.listdir(frames_dir) if f.endswith('.ply')]
    ply_files.sort()  # Sort files to ensure correct order
    print(f"Found {len(ply_files)} .ply files.")

    #open scan_28.ply a visualize 2d image of point cloud from camera perspective
    ply_name = "scan_28.ply"
    ply_path = os.path.join(frames_dir, ply_name)
    print(f"Loading PLY: {ply_path}")

    plydata = PlyData.read(str(ply_path))
    print(plydata)
    vtx_texture_obj = plydata.elements[0].properties[9]
    print(f"Texture property name: {vtx_texture_obj.name}")
    vtx = plydata.elements[0].data['Texture32']
    print(f"Texture data shape: {vtx.shape}")
    vtx = np.array(vtx.tolist(), dtype=np.uint32)
    #Texture property name: Texture32
    # Texture data shape: (3186816,)
    #it is only gray scale image
    #plot this as 2d image
    img_width =  plydata.elements[2].data['frame_width'].item()
    img_height = plydata.elements[2].data['frame_height'].item()

    print(f"Image dimensions: {img_width}x{img_height}")
    reshaped_vtx = vtx.reshape((img_height, img_width))
    print(f"Reshaped texture data shape: {reshaped_vtx.shape}")
    # Plot the grayscale image
    # plt.imshow(reshaped_vtx, cmap='gray')
    # plt.axis('off')
    # plt.show()


   # Analyzujte rozsah hodnot
    print(f"Value range: min={np.min(vtx)}, max={np.max(vtx)}")
    print(f"Non-zero values: {np.count_nonzero(vtx)} / {len(vtx)}")
    print(f"Mean: {np.mean(vtx):.2f}, Std: {np.std(vtx):.2f}")
    
    # Vytvořte několik verzí s různými normalizacemi
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    
    # 1. Původní obraz
    axes[0,0].imshow(reshaped_vtx, cmap='gray')
    axes[0,0].set_title('Original')
    axes[0,0].axis('off')
    
    # 2. Logaritmická škála (pro velký dynamický rozsah)
    log_vtx = np.log1p(reshaped_vtx)  # log1p = log(1+x) aby se vyhnuli log(0)
    axes[0,1].imshow(log_vtx, cmap='gray')
    axes[0,1].set_title('Logarithmic Scale')
    axes[0,1].axis('off')
    
    # 3. Histogram equalization
    from scipy import ndimage
    # Jednoduchá histogram equalization pomocí percentilů
    p2, p98 = np.percentile(vtx[vtx > 0], (2, 98))  # Ignorujte nulové hodnoty
    hist_eq = np.clip((reshaped_vtx - p2) / (p98 - p2), 0, 1)
    axes[0,2].imshow(hist_eq, cmap='gray')
    axes[0,2].set_title('Histogram Equalization (2-98%)')
    axes[0,2].axis('off')
    
    # 4. Min-Max normalizace (pouze non-zero hodnoty)
    non_zero_mask = reshaped_vtx > 0
    normalized = reshaped_vtx.astype(float)
    if np.any(non_zero_mask):
        min_val = np.min(reshaped_vtx[non_zero_mask])
        max_val = np.max(reshaped_vtx[non_zero_mask])
        normalized[non_zero_mask] = (reshaped_vtx[non_zero_mask] - min_val) / (max_val - min_val)
    axes[1,0].imshow(normalized, cmap='gray')
    axes[1,0].set_title('Min-Max Normalized (non-zero)')
    axes[1,0].axis('off')
    
    # 5. Gamma korekce
    gamma = 0.5  # Experimentujte s hodnotami 0.3-2.0
    gamma_corrected = np.power(normalized, gamma)
    axes[1,1].imshow(gamma_corrected, cmap='gray')
    axes[1,1].set_title(f'Gamma Correction (γ={gamma})')
    axes[1,1].axis('off')
    
    # 6. Adaptivní kontrast (CLAHE-like)
    # Rozdělte obraz na bloky a normalizujte každý zvlášť
    block_size = 100
    adaptive = normalized.copy()
    for i in range(0, img_height, block_size):
        for j in range(0, img_width, block_size):
            block = normalized[i:i+block_size, j:j+block_size]
            if np.max(block) > np.min(block):
                adaptive[i:i+block_size, j:j+block_size] = (block - np.min(block)) / (np.max(block) - np.min(block))
    
    axes[1,2].imshow(adaptive, cmap='gray')
    axes[1,2].set_title('Adaptive Contrast')
    axes[1,2].axis('off')
    
    plt.tight_layout()
    plt.show()
    
    # Histogram hodnot
    plt.figure(figsize=(12, 4))
    
    plt.subplot(1, 2, 1)
    plt.hist(vtx[vtx > 0], bins=100, alpha=0.7)
    plt.title('Histogram of Non-Zero Values')
    plt.xlabel('Pixel Value')
    plt.ylabel('Frequency')
    plt.yscale('log')
    
    plt.subplot(1, 2, 2)
    plt.hist(np.log1p(vtx[vtx > 0]), bins=100, alpha=0.7)
    plt.title('Histogram of Log Values')
    plt.xlabel('Log(Pixel Value + 1)')
    plt.ylabel('Frequency')
    
    plt.tight_layout()
    plt.show()