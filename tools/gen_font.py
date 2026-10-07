#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_font.py - 汉字点阵字库生成器 (12x12)

作用: 扫描工程里所有 .c / .h 文件的字符串字面量中出现的汉字(含中文标点), 用系统里的中文字体
      渲染成 12x12 点阵, 生成 oled_font_cn.c。
用法: python3 tools/gen_font.py            (在工程根目录执行)
      python3 tools/gen_font.py --font /path/to/font.ttc --index 1
      python3 tools/gen_font.py --preview preview.png     (同时输出预览图, 用来肉眼检查)
想额外收录一些现在代码里还没用到的字: 写到 tools/extra_chars.txt 里即可。
依赖: pip install pillow ; 系统里要有中文字体 (默认自动查找文泉驿/Noto CJK)。
"""
import argparse, glob, os, re, sys
from PIL import Image, ImageDraw, ImageFont

SIZE = 12
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

FONT_CANDIDATES = [  # (路径, ttc 序号) 优先使用点阵优化好的字体
    ("/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc", 1),   # 文泉驿点阵正黑 (12px 手工优化点阵)
    ("/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc", 0),
    ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 2),
    ("C:/Windows/Fonts/simsun.ttc", 0),                    # Windows 宋体 (自带 12px 点阵)
    ("C:/Windows/Fonts/msyh.ttc", 0),
    ("/System/Library/Fonts/PingFang.ttc", 0),
]

def is_cjk(ch):
    c = ord(ch)
    return (0x2E80 <= c <= 0x9FFF) or (0x3000 <= c <= 0x303F) or (0xFF00 <= c <= 0xFFEF) or c in (0x2026, 0x2014, 0x00B7)

def collect_chars():
    chars = set()
    files = glob.glob(os.path.join(ROOT, "*.c")) + glob.glob(os.path.join(ROOT, "*.h"))
    for f in files:
        if os.path.basename(f) == "oled_font_cn.c":
            continue
        with open(f, encoding="utf-8") as fp:
            text = fp.read()
        # 只收录 "字符串字面量" 里的汉字, 注释里的汉字不会显示在屏幕上, 不需要字模
        for lit in re.findall(r'"((?:[^"\\\n]|\\.)*)"', text):
            for ch in lit:
                if is_cjk(ch):
                    chars.add(ch)
    extra = os.path.join(ROOT, "tools", "extra_chars.txt")
    if os.path.exists(extra):
        with open(extra, encoding="utf-8") as fp:
            for ch in fp.read():
                if is_cjk(ch):
                    chars.add(ch)
    return sorted(chars, key=ord)

def load_font(path, index):
    if path:
        return ImageFont.truetype(path, SIZE, index=index), path
    for p, i in FONT_CANDIDATES:
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, SIZE, index=i), f"{p}#{i}"
            except Exception:
                pass
    sys.exit("找不到中文字体, 请用 --font 指定一个包含汉字的 .ttf/.ttc 文件")

def render(font, ch):
    img = Image.new("1", (SIZE, SIZE), 0)
    d = ImageDraw.Draw(img)
    d.fontmode = "1"                       # 关闭抗锯齿, 直接得到黑白点阵
    bbox = d.textbbox((0, 0), ch, font=font)
    w, h = bbox[2] - bbox[0], bbox[3] - bbox[1]
    x = (SIZE - w) // 2 - bbox[0]          # 水平居中
    y = (SIZE - h) // 2 - bbox[1]          # 垂直居中
    d.text((x, y), ch, font=font, fill=1)
    return img

def to_bytes(img):
    out = []
    for row in range(SIZE):
        bits = 0
        for col in range(SIZE):
            if img.getpixel((col, row)):
                bits |= 0x8000 >> col      # 每行 2 字节, 高位在前, 只用高 12 位
        out += [bits >> 8, bits & 0xFF]
    return out

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--font"); ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--preview")
    a = ap.parse_args()

    chars = collect_chars()
    font, used = load_font(a.font, a.index)
    print(f"字体: {used}\n收录汉字 {len(chars)} 个")

    lines = [
        "// oled_font_cn.c - 自动生成, 请勿手工修改!",
        "// 生成命令: python3 tools/gen_font.py   (字体: %s)" % os.path.basename(used.split('#')[0]),
        "// 格式: 12x12 点阵, 每行 2 字节, 高位在前, 仅使用每行高 12 位; 按 Unicode 升序排列",
        '#include "oled_display.h"',
        "",
        "typedef struct { uint16_t code; uint8_t bits[24]; } cn_glyph_t;",
        "",
        "static const cn_glyph_t CN_FONT[] = {",
    ]
    glyphs = []
    for ch in chars:
        img = render(font, ch)
        glyphs.append((ch, img))
        data = ", ".join("0x%02X" % b for b in to_bytes(img))
        lines.append("    {0x%04X, {%s}},  // %s" % (ord(ch), data, ch))
    lines += [
        "};",
        "",
        "#define CN_FONT_COUNT (sizeof(CN_FONT) / sizeof(CN_FONT[0]))",
        "",
        "// 二分查找",
        "const uint8_t *oled_font_cn_find(uint32_t unicode) {",
        "    if (unicode > 0xFFFF) return 0;",
        "    int lo = 0, hi = (int)CN_FONT_COUNT - 1;",
        "    while (lo <= hi) {",
        "        int mid = (lo + hi) / 2;",
        "        if (CN_FONT[mid].code == unicode) return CN_FONT[mid].bits;",
        "        if (CN_FONT[mid].code < unicode) lo = mid + 1; else hi = mid - 1;",
        "    }",
        "    return 0;",
        "}",
        "",
    ]
    with open(os.path.join(ROOT, "oled_font_cn.c"), "w", encoding="utf-8") as fp:
        fp.write("\n".join(lines))
    print("已生成 oled_font_cn.c")

    if a.preview:
        cols = 16
        rows = (len(glyphs) + cols - 1) // cols
        scale = 6
        sheet = Image.new("RGB", (cols * (SIZE + 2) * scale, rows * (SIZE + 2) * scale), (20, 20, 20))
        for n, (ch, img) in enumerate(glyphs):
            ox, oy = (n % cols) * (SIZE + 2) * scale, (n // cols) * (SIZE + 2) * scale
            for y in range(SIZE):
                for x in range(SIZE):
                    if img.getpixel((x, y)):
                        for dy in range(scale - 1):
                            for dx in range(scale - 1):
                                sheet.putpixel((ox + x * scale + dx, oy + y * scale + dy), (120, 220, 255))
        sheet.save(a.preview)
        print("预览图:", a.preview)

if __name__ == "__main__":
    main()
