#!/usr/bin/env python3
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path("/mnt/c/Users/22240/rc2026_snapshot/other")
SRC = ROOT / "main_atlas_v3_field_layout_page04.png"
OUT = ROOT / "zone3_corner_concept_on_atlas.png"
ZOOM_OUT = ROOT / "zone3_corner_concept_zoom.png"
FONT = "/mnt/c/Windows/Fonts/simhei.ttf"


def label(draw, font, text, xy, fill, bg=(255, 255, 255)):
    x, y = xy
    bbox = draw.textbbox((x, y), text, font=font)
    draw.rounded_rectangle(
        (bbox[0] - 8, bbox[1] - 5, bbox[2] + 8, bbox[3] + 5),
        radius=6,
        fill=bg,
        outline=fill,
        width=2,
    )
    draw.text((x, y), text, font=font, fill=fill)


def draw_dash_line(draw, p0, p1, fill, width, dash=16):
    x0, y0 = p0
    x1, y1 = p1
    if x0 == x1:
        step = dash * 2 if y1 >= y0 else -dash * 2
        for y in range(y0, y1, step):
            y2 = y + (dash if step > 0 else -dash)
            draw.line((x0, y, x1, max(min(y2, max(y0, y1)), min(y0, y1))), fill=fill, width=width)
    elif y0 == y1:
        step = dash * 2 if x1 >= x0 else -dash * 2
        for x in range(x0, x1, step):
            x2 = x + (dash if step > 0 else -dash)
            draw.line((x, y0, max(min(x2, max(x0, x1)), min(x0, x1)), y1), fill=fill, width=width)


def main():
    img = Image.open(SRC).convert("RGB")
    font_small = ImageFont.truetype(FONT, 24)
    font_big = ImageFont.truetype(FONT, 42)

    # 蓝区第三区在导出图上的近似像素坐标，用于概念标注。
    top_left = (1118, 430)
    top_right = (1410, 430)
    bottom_right = (1410, 890)
    bottom_left = (1118, 890)
    corner = top_right

    overlay = Image.new("RGBA", img.size, (255, 255, 255, 0))
    od = ImageDraw.Draw(overlay)
    od.rectangle((0, 0, 1015, img.height), fill=(255, 255, 255, 135))
    img = Image.alpha_composite(img.convert("RGBA"), overlay).convert("RGB")
    draw = ImageDraw.Draw(img)

    # 灰虚线是用图册尺寸外推，不是当前点云拟合出来的边。
    draw_dash_line(draw, top_left, top_right, (50, 120, 255), 8)
    draw_dash_line(draw, top_right, bottom_right, (255, 60, 60), 8)
    draw_dash_line(draw, bottom_left, bottom_right, (120, 120, 120), 5)
    draw_dash_line(draw, top_left, bottom_left, (120, 120, 120), 5)

    # 只拟合两条可靠边：外侧长边、端部边。
    draw.line((top_left[0] - 12, top_left[1], top_right[0] + 8, top_right[1]), fill=(0, 92, 255), width=14)
    draw.line((top_right[0], top_right[1] - 10, bottom_right[0], bottom_right[1] + 12), fill=(255, 30, 30), width=14)

    r = 18
    draw.ellipse((corner[0] - r, corner[1] - r, corner[0] + r, corner[1] + r), fill=(255, 0, 0), outline=(255, 255, 255), width=5)
    draw.line((corner[0] + 36, corner[1] - 78, corner[0] + 36, corner[1] + 78), fill=(160, 0, 255), width=12)
    draw.polygon(
        [(corner[0] + 36, corner[1] - 108), (corner[0] + 18, corner[1] - 72), (corner[0] + 54, corner[1] - 72)],
        fill=(160, 0, 255),
    )

    draw.text((1040, 285), "蓝区第三区角点定位设想", font=font_big, fill=(0, 0, 0))
    label(draw, font_small, "蓝线：拟合外侧长边", (1040, 365), (0, 92, 255))
    label(draw, font_small, "红线：拟合端部边", (1435, 600), (255, 30, 30))
    label(draw, font_small, "红点：目标角点 = 蓝线 ∩ 红线", (1210, 360), (255, 0, 0))
    label(draw, font_small, "紫线：角点竖向边，俯视图只能符号表示", (1245, 470), (130, 0, 220))
    label(draw, font_small, "灰虚线：用图册尺寸外推，不靠噪声绿线", (1130, 930), (80, 80, 80))

    draw.line((1240, 405, corner[0] - 8, corner[1] - 8), fill=(255, 0, 0), width=5)
    draw.polygon([(corner[0] - 8, corner[1] - 8), (corner[0] - 38, corner[1] - 10), (corner[0] - 15, corner[1] - 36)], fill=(255, 0, 0))
    draw.line((1320, 505, corner[0] + 32, corner[1] - 50), fill=(130, 0, 220), width=4)

    img.save(OUT)
    crop = img.crop((980, 240, 1535, 1010))
    crop = crop.resize((int(crop.width * 1.55), int(crop.height * 1.55)), Image.LANCZOS)
    crop.save(ZOOM_OUT)
    print(OUT)
    print(ZOOM_OUT)


if __name__ == "__main__":
    main()
