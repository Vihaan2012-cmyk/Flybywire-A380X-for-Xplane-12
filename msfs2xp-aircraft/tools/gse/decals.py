import math
import os
import sys

from PIL import Image, ImageDraw, ImageFont

FONTS = 'C:/Windows/Fonts'
BLUE = (16, 40, 140, 255)
GREEN = (0, 140, 70, 255)
RED = (200, 20, 25, 255)
WHITE = (250, 250, 248, 255)
BLACK = (20, 20, 20, 255)
YELLOW = (250, 205, 0, 255)
SIGN_BLUE = (0, 90, 170, 255)


def font(name, size):
    return ImageFont.truetype(os.path.join(FONTS, name), size)


def canvas(w, h, bg=(0, 0, 0, 0)):
    im = Image.new('RGBA', (w, h), bg)
    return im, ImageDraw.Draw(im)


def centered(d, box, text, f, fill):
    l, t, r, b = d.textbbox((0, 0), text, font=f)
    x = box[0] + (box[2] - box[0] - (r - l)) / 2 - l
    y = box[1] + (box[3] - box[1] - (b - t)) / 2 - t
    d.text((x, y), text, font=f, fill=fill)


def tld_logo(w=1024, h=512):
    im, d = canvas(w, h)
    f = font('ariblk.ttf', int(h * 0.9))
    centered(d, (0, 0, w, h), 'TLD', f, BLUE)
    return im


def tld_green(w=1024, h=256):
    im, d = canvas(w, h)
    f = font('arialbd.ttf', int(h * 0.55))
    d.text((int(w * 0.07), int(h * 0.17)), 'TLD', font=f, fill=BLUE)
    d.text((int(w * 0.37), int(h * 0.17)), 'Green', font=f, fill=GREEN)
    cx, cy, r = int(w * 0.9), int(h * 0.35), int(h * 0.16)
    d.pieslice((cx - r, cy - r, cx + r, cy + r), 200, 340, fill=GREEN)
    d.polygon([(cx - r, cy), (cx + r, cy), (cx, cy + int(r * 1.5))], fill=GREEN)
    return im


def warning_triangle(size=256, glyph='!'):
    im, d = canvas(size, size, YELLOW)
    m = size * 0.1
    d.polygon([(size / 2, m), (size - m, size - m), (m, size - m)], fill=BLACK)
    i = size * 0.2
    d.polygon([(size / 2, m + i * 0.9), (size - m - i * 0.75, size - m - i * 0.45), (m + i * 0.75, size - m - i * 0.45)], fill=YELLOW)
    if glyph == 'bolt':
        cx, cy = size / 2, size * 0.62
        s = size * 0.2
        d.polygon([(cx + s * 0.2, cy - s), (cx - s * 0.5, cy + s * 0.1), (cx, cy + s * 0.1), (cx - s * 0.2, cy + s), (cx + s * 0.5, cy - s * 0.1), (cx, cy - s * 0.1)], fill=BLACK)
    else:
        centered(d, (0, size * 0.32, size, size * 0.92), '!', font('arialbd.ttf', int(size * 0.42)), BLACK)
    return im


def mandatory_sign(size=256):
    im, d = canvas(size, size, WHITE)
    m = size * 0.06
    d.ellipse((m, m, size - m, size - m), fill=SIGN_BLUE)
    cx, cy = size / 2, size / 2
    d.ellipse((cx - size * 0.13, cy - size * 0.3, cx + size * 0.13, cy - size * 0.05), fill=WHITE)
    d.rounded_rectangle((cx - size * 0.24, cy - size * 0.02, cx + size * 0.24, cy + size * 0.3), radius=int(size * 0.08), fill=WHITE)
    for k in (-1, 1):
        d.ellipse((cx + k * size * 0.17 - size * 0.06, cy - size * 0.22, cx + k * size * 0.17 + size * 0.06, cy - size * 0.1), fill=WHITE)
    return im


def sign_stack():
    a, b, c = mandatory_sign(256), warning_triangle(256), warning_triangle(256, 'bolt')
    im, d = canvas(256, 256 * 3 + 32)
    for k, s in enumerate((a, b, c)):
        im.paste(s, (0, k * (256 + 16)))
    return im


def text_label(text, w, h, fg=BLACK, bg=(0, 0, 0, 0), name='arialbd.ttf', scale=0.7):
    im, d = canvas(w, h, bg)
    centered(d, (0, 0, w, h), text, font(name, int(h * scale)), fg)
    return im


def tyre_pressure():
    im, d = canvas(1536, 256)
    f = font('arialbd.ttf', 150)
    d.text((10, 40), '5,86 BAR', font=f, fill=BLACK)
    d.ellipse((760, 70, 880, 190), outline=BLACK, width=14)
    d.text((960, 40), '85 PSI', font=f, fill=BLACK)
    for x0, x1 in ((700, 750), (900, 950)):
        d.line([(x0, 130), (x1, 130)], fill=BLACK, width=12)
    return im


def max_speed():
    im, d = canvas(768, 384)
    f = font('arialbd.ttf', 120)
    centered(d, (0, 0, 768, 190), 'MAX 16 KPH', f, BLACK)
    centered(d, (0, 190, 768, 384), '10 MPH', f, BLACK)
    return im


def chevrons(w=256, h=1024, band=90):
    im, d = canvas(w, h, WHITE)
    for k in range(-w, h + w, band * 2):
        d.polygon([(0, k), (w, k + w * 0.8), (w, k + w * 0.8 + band), (0, k + band)], fill=RED)
    return im


def embossed_tld(w=1024, h=512):
    im, d = canvas(w, h)
    f = font('ariblk.ttf', int(h * 0.9))
    centered(d, (6, 6, w + 6, h + 6), 'TLD', f, (150, 150, 150, 110))
    centered(d, (0, 0, w, h), 'TLD', f, (255, 255, 255, 90))
    return im


def tray_stamp(w=512, h=256):
    im, d = canvas(w, h)
    centered(d, (0, 0, w, h), 'TLD', font('ariblk.ttf', int(h * 0.8)), (60, 62, 64, 230))
    return im


def screen(w=512, h=384):
    im, d = canvas(w, h, (225, 232, 236, 255))
    d.rectangle((0, 0, w - 1, h - 1), outline=(60, 60, 60, 255), width=6)
    small = font('arial.ttf', 26)
    d.text((24, 18), 'TLD GPU 90kVA', font=small, fill=BLACK)
    d.text((w - 140, 18), 'T36297', font=small, fill=BLACK)
    d.rounded_rectangle((40, 80, 120, 160), radius=16, outline=BLACK, width=5)
    d.text((w - 170, 110), 'Engine', font=small, fill=BLACK)
    d.text((w - 170, 140), 'Hours', font=small, fill=BLACK)
    d.line([(40, 230), (w - 40, 230)], fill=(80, 80, 80, 255), width=3)
    d.text((70, 250), 'Press Engine Start/Stop to Start Unit', font=font('arial.ttf', 22), fill=BLACK)
    for i, col in enumerate(((40, 150, 140, 255), (200, 40, 40, 255), (40, 140, 90, 255))):
        x = 40 + i * 170
        d.rounded_rectangle((x, 300, x + 90, 370), radius=10, fill=col)
    return im


def tld_white(w=512, h=256):
    im, d = canvas(w, h)
    centered(d, (0, 0, w, h), 'TLD', font('ariblk.ttf', int(h * 0.85)), WHITE)
    return im


def fuel_label(w=384, h=320):
    im, d = canvas(w, h, GREEN)
    d.rectangle((8, 8, w - 9, h - 9), outline=WHITE, width=6)
    small = font('arialbd.ttf', 34)
    for i, line in enumerate(('ULTRA LOW', 'SULFUR', 'DIESEL', 'FUEL ONLY')):
        centered(d, (0, 30 + i * 66, w, 30 + (i + 1) * 66), line, small, WHITE)
    return im


ALL = {
    'tld_white': tld_white,
    'fuel_label': fuel_label,
    'tld_logo': tld_logo,
    'tld_green': tld_green,
    'sign_stack': sign_stack,
    'sign_mandatory': lambda: mandatory_sign(256),
    'sign_warning': lambda: warning_triangle(256),
    'sign_electric': lambda: warning_triangle(256, 'bolt'),
    'diesel': lambda: text_label('DIESEL', 512, 160, fg=WHITE, bg=GREEN),
    'tyre_pressure': tyre_pressure,
    'max_speed': max_speed,
    'chevrons': chevrons,
    'embossed_tld': embossed_tld,
    'tray_stamp': tray_stamp,
    'screen': screen,
}


def main(out):
    os.makedirs(out, exist_ok=True)
    for name, fn in ALL.items():
        fn().save(os.path.join(out, f'{name}.png'))
    print(f'{len(ALL)} decals in {out}')


if __name__ == '__main__':
    main(sys.argv[1])
