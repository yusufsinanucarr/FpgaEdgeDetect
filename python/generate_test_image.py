# generate_test_image.py
# Generates a 128x128 test image with distinct geometric edges

from PIL import Image, ImageDraw

def create_test_image(filename="test_image.png"):
    img = Image.new("L", (128, 128), color=30) # Dark gray background
    draw = ImageDraw.Draw(img)

    # White rectangle
    draw.rectangle([15, 15, 55, 55], outline=255, fill=200, width=2)

    # White circle
    draw.ellipse([70, 15, 115, 60], outline=255, fill=180, width=2)

    # White triangle
    draw.polygon([(64, 70), (20, 115), (108, 115)], outline=255, fill=160)

    # Diagonal line
    draw.line([10, 120, 118, 10], fill=255, width=2)

    img.save(filename)
    print(f"[+] 128x128 test image created: {filename}")

if __name__ == "__main__":
    create_test_image()
