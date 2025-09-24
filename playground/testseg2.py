import torch
from PIL import Image
import requests
from transformers import SamModel, SamProcessor, infer_device
import numpy as np
import matplotlib.pyplot as plt

device = infer_device()
model = SamModel.from_pretrained("facebook/sam-vit-huge").to(device)
processor = SamProcessor.from_pretrained("facebook/sam-vit-huge")

img_url = "https://huggingface.co/ybelkada/segment-anything/resolve/main/assets/car.png"
raw_image = Image.open(requests.get(img_url, stream=True).raw).convert("RGB")
input_points = [[[1120, 600]]]  # 2D location of a window in the image

inputs = processor(raw_image, input_points=input_points, return_tensors="pt").to(device)
with torch.no_grad():
    outputs = model(**inputs)

masks = processor.image_processor.post_process_masks(
    outputs.pred_masks.cpu(), inputs["original_sizes"].cpu(), inputs["reshaped_input_sizes"].cpu()
)
scores = outputs.iou_scores
#visualize the mask
plt.figure(figsize=(10, 10))
plt.imshow(raw_image)
for i, mask in enumerate(masks):
    m = np.squeeze(mask.numpy())  # Ensure mask is 2D
    plt.contour(m[0], colors=[(1, 0, 0)], linewidths=0.5)  # červené obrysy
    # centroid pro číslo
    ys, xs = np.where(m[0])
    if len(xs) == 0 or len(ys) == 0:
        continue
    cx, cy = int(np.mean(xs)), int(np.mean(ys))
    plt.text(cx, cy, str(i), color='yellow', fontsize=12, ha='center', va='center')
plt.axis('off')
plt.show()
