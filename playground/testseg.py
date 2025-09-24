import cv2 as cv
import numpy as np
from skimage.morphology import skeletonize
import matplotlib.pyplot as plt
from segment_anything import sam_model_registry, SamAutomaticMaskGenerator, SamPredictor


# --- vyber masku „čáry“ (často nejdelší tenký komponent) ---
def pick_best_line_mask(masks):
    best = None
    best_score = -1
    for m in masks:
        mask = m['segmentation'].astype(np.uint8)
        # tenká a dlouhá: heuristika = vysoký obvod/plocha + velká „délka“ kostry
        skel = skeletonize(mask>0)
        length = skel.sum()
        area   = mask.sum()
        if area == 0: 
            continue
        score = length / (np.sqrt(area) + 1e-6)
        if score > best_score:
            best_score = score
            best = mask
    return best


# --- vizualizace masek ---
def visualize_masks(image, masks):
    plt.figure(figsize=(10, 10))
    plt.imshow(image)
    for i, mask in enumerate(masks):
        m = mask['segmentation']
        plt.contour(m, colors=[(1, 0, 0)], linewidths=0.5)  # červené obrysy
        # centroid pro číslo
        ys, xs = np.where(m)
        if len(xs) == 0 or len(ys) == 0:
            continue
        cx, cy = int(np.mean(xs)), int(np.mean(ys))
        plt.text(cx, cy, str(i), color='yellow', fontsize=12, ha='center', va='center')
    plt.axis('off')
    plt.show()

if __name__ == "__main__":
    import os
    print(os.getcwd())
    # --- načti mono snímek PhoXi ---
    img_mono = cv.imread("playground/gamma_corrected.png", cv.IMREAD_GRAYSCALE)           # HxW
    img_rgb  = cv.cvtColor(img_mono, cv.COLOR_GRAY2RGB)                    # HxWx3

    # --- SAM: model + auto masky ---
    sam = sam_model_registry["vit_h"](checkpoint="sam_vit_h.pth").to("cuda")
    amg = SamAutomaticMaskGenerator(
        model=sam,
        points_per_side=64,      # zvýší „granularitu“ (lepší pro tenké struktury)
        pred_iou_thresh=0.86,
        stability_score_thresh=0.92,
        box_nms_thresh=0.6,
        output_mode="binary_mask"
    )
    masks = amg.generate(img_rgb)   # list dictů s 'segmentation' (HxW bool)
    print(f"Generated {len(masks)} masks.")

    visualize_masks(img_rgb, masks)
    mask_line = pick_best_line_mask(masks)
    assert mask_line is not None, "Auto generátor nenašel kandidátní masku – zkus interaktivní variantu níž."

    # --- skeleton (1 px) ---
    skel = skeletonize(mask_line > 0).astype(np.uint8)

    # --- extrahuj (y,x) body skeletonu v pořadí po křivce (jednoduchý greedy řetězení) ---
    ys, xs = np.where(skel > 0)
    pts = list(zip(xs, ys))  # pozor: (x,y)
    # (volitelně: post-processing – seřazení podél křivky, MST apod.)
