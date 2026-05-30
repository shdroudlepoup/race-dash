from PIL import Image

img = Image.open(r"C:\Users\remig\Downloads\chara_stand_1068_106801.png").convert("RGBA")

target_h = 90
ratio = target_h / img.height
target_w = int(img.width * ratio)
img = img.resize((target_w, target_h), Image.LANCZOS)

BG_COLOR = 0xFFFF  # noir sur cet écran

print(f"// Image: {target_w}x{target_h} pixels")
print(f"#define IMG_W {target_w}")
print(f"#define IMG_H {target_h}")
print(f"const uint16_t chara_img[{target_w * target_h}] PROGMEM = {{")

for y in range(target_h):
    row = []
    for x in range(target_w):
        r, g, b, a = img.getpixel((x, y))
        if a < 128:
            row.append(f"0x{BG_COLOR:04X}")
        else:
            r5 = r >> 3
            g6 = g >> 2
            b5 = b >> 3
            rgb565 = (r5 << 11) | (g6 << 5) | b5
            inverted = (~rgb565) & 0xFFFF  # juste inverser, pas BGR
            row.append(f"0x{inverted:04X}")
    print(f"  {','.join(row)}{',' if y < target_h - 1 else ''}")

print("};")
