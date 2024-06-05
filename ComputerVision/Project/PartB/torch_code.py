from torchvision.transforms import v2
from PIL import Image
import matplotlib.pyplot as plt

image_path = 'images/3_colour.jpeg'
img = Image.open(image_path)

transform = v2.ColorJitter(brightness=0.3, contrast=0.3, saturation=0.1, hue=0.3)

out = transform(img)

plt.figure(figsize=(12, 6))

plt.subplot(1, 2, 1)
plt.title('Original Image')
plt.imshow(img)

plt.subplot(1, 2, 2)
plt.title('Transformed Image')
plt.imshow(out)

plt.show()
