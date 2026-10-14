import json
import os
import sys
import textwrap

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

shots_dir, fonts_dir, tips_path, out_dir = sys.argv[1:5]
os.makedirs(out_dir, exist_ok=True)

DESIGN_W = 2560
BAND_H = 330
FADE_H = 160
LAYER_H = BAND_H + FADE_H
CYAN = (0, 200, 255, 255)
WHITE = (240, 244, 248, 255)
GREY = (150, 160, 175, 255)
GREEN = (120, 220, 140, 255)


def font(name, size):
    return ImageFont.truetype(os.path.join(fonts_dir, name), size)


mono = lambda size: font('EFB/JetBrainsMono-Regular.ttf', size)
fcu = lambda size: font('A380X_FCU.ttf', size)
manifest = {'design_width': DESIGN_W, 'layer_height': LAYER_H, 'band_height': BAND_H, 'sprites': {}, 'backgrounds': [], 'tips': []}


def save_sprite(name, im, x, y):
    box = im.getbbox()
    if box is None:
        return
    im = im.crop(box)
    im.save(os.path.join(out_dir, f'{name}.png'))
    manifest['sprites'][name] = [x + box[0], y + box[1], im.width, im.height]


def canvas():
    return Image.new('RGBA', (DESIGN_W, LAYER_H), (0, 0, 0, 0))


def crop_menu_bar(im):
    rows = np.asarray(im, dtype=np.float32).mean(axis=2).mean(axis=1)
    w, h = im.size
    if rows[:20].mean() > 80:
        return im
    for i in range(4, min(120, h)):
        if rows[i] - rows[:i].max() > 60:
            return im.crop((0, i + round(h * 0.02), w, h))
    return im


def grade(im):
    a = np.asarray(im, dtype=np.float32) / 255
    w3 = np.array([0.2126, 0.7152, 0.0722])
    lum = a @ w3
    bright = np.clip((lum - 0.8) / 0.2, 0, 1)[..., None] * a
    bloom = np.asarray(Image.fromarray((bright * 255).astype('uint8')).filter(ImageFilter.GaussianBlur(24)), dtype=np.float32) / 255
    a = a + bloom * 0.18
    a = np.clip((a - 0.5) * 1.06 + 0.5, 0, 1)
    a = a * a * (3 - 2 * a) * 0.35 + a * 0.65
    lum = (a @ w3)[..., None]
    shadows = np.clip(1 - lum * 2, 0, 1)
    highlights = np.clip(lum * 2 - 1, 0, 1)
    a = a + shadows * np.array([-0.02, 0.01, 0.04]) + highlights * np.array([0.04, 0.015, -0.03])
    a = lum + (a - lum) * 1.12
    a = np.where(a > 0.85, 0.85 + (1 - np.exp(-(a - 0.85) / 0.15)) * 0.15, a)
    h, w = lum.shape[:2]
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    r = np.sqrt(((xx - w / 2) / (w / 2)) ** 2 + ((yy - h / 2) / (h / 2)) ** 2)
    a = a * (1 - 0.35 * np.clip(r - 0.55, 0, 1) ** 1.5)[..., None]
    out = Image.fromarray((np.clip(a, 0, 1) * 255).astype('uint8'))
    return out


shots = sorted(f for f in os.listdir(shots_dir) if f.lower().endswith(('.webp', '.png', '.jpg', '.jpeg')))
for i, name in enumerate(shots):
    im = crop_menu_bar(Image.open(os.path.join(shots_dir, name)).convert('RGB'))
    target = (DESIGN_W, DESIGN_W * 9 // 16)
    scale = max(target[0] / im.width, target[1] / im.height)
    cw, ch = round(target[0] / scale), round(target[1] / scale)
    left, top = (im.width - cw) // 2, (im.height - ch) // 2
    im = im.crop((left, top, left + cw, top + ch)).resize(target, Image.LANCZOS)
    im = grade(im)
    out = f'bg_{i + 1:02d}.png'
    im.save(os.path.join(out_dir, out), optimize=True)
    manifest['backgrounds'].append(out)

grad = Image.new('L', (1, LAYER_H))
for y in range(LAYER_H):
    t = y / LAYER_H
    grad.putpixel((0, y), int(235 * min(1, max(0, (t - 0.05) / 0.55)) ** 1.6))
fade = Image.new('RGBA', (1, LAYER_H), (6, 9, 14, 255))
fade.putalpha(grad)
fade.save(os.path.join(out_dir, 'band.png'))
manifest['sprites']['band'] = [0, 0, DESIGN_W, LAYER_H]

top = LAYER_H - BAND_H
base = LAYER_H - 70
left = 90
c = canvas()
d = ImageDraw.Draw(c)
d.text((left - 6, base - 150), 'A380', font=font('EFB/Manrope-Bold.ttf', 168), fill=(255, 255, 255, 255), anchor='ls')
d.text((left, base - 40), 'F O R   X - P L A N E   1 2', font=font('EFB/Manrope-Medium.ttf', 30), fill=(200, 210, 222, 255), anchor='ls')
d.line([(left, base - 112), (left + 120, base - 112)], fill=CYAN, width=4)
save_sprite('wordmark', c, 0, 0)

tips = [t.strip() for t in open(tips_path, encoding='utf-8') if t.strip()]
cx = 900
for i, tip in enumerate(tips):
    c = canvas()
    d = ImageDraw.Draw(c)
    d.line([(cx - 40, top + 40), (cx - 40, LAYER_H - 60)], fill=(255, 255, 255, 50), width=2)
    d.text((cx, top + 72), 'Did you know?', font=mono(30), fill=CYAN, anchor='ls')
    y = top + 116
    for line in textwrap.wrap(tip, 78):
        d.text((cx, y), line, font=mono(24), fill=WHITE, anchor='ls')
        y += 36
    d.text((cx, y + 12), f'Tip {i + 1} of {len(tips)}', font=mono(20), fill=GREY, anchor='ls')
    name = f'tip_{i + 1:02d}'
    save_sprite(name, c, 0, 0)
    manifest['tips'].append(name)

rx = DESIGN_W - 90
c = canvas()
ImageDraw.Draw(c).text((rx, top + 70), 'Loading', font=mono(24), fill=GREY, anchor='rs')
save_sprite('loading', c, 0, 0)

digit_font = fcu(92)
pct_font = mono(40)
c = canvas()
ImageDraw.Draw(c).text((rx, top + 172), '%', font=pct_font, fill=CYAN, anchor='rs')
save_sprite('percent', c, 0, 0)
pct_w = ImageDraw.Draw(c).textlength('%', font=pct_font)
advance = max(ImageDraw.Draw(c).textlength(str(n), font=digit_font) for n in range(10))
manifest['digits'] = {'right': rx - pct_w - 10, 'advance': advance}
for n in range(10):
    c = canvas()
    ImageDraw.Draw(c).text((rx - pct_w - 10, top + 172), str(n), font=digit_font, fill=(255, 255, 255, 255), anchor='rs')
    save_sprite(f'digit_{n}', c, 0, 0)

check = font('EFB/Inter-SemiBold.ttf', 26)
for row, label in enumerate(['Aircraft systems', 'Cockpit displays', 'Sound']):
    y = top + 210 + row * 29
    for state, colour, mark in [('done', GREEN, '✓'), ('active', CYAN, '•'), ('waiting', (110, 118, 130, 255), '•')]:
        c = canvas()
        d = ImageDraw.Draw(c)
        d.text((rx, y), mark, font=check, fill=colour, anchor='rs')
        d.text((rx - 34, y), label, font=mono(22), fill=colour, anchor='rs')
        save_sprite(f'item_{row}_{state}', c, 0, 0)

white = Image.new('RGBA', (4, 4), (255, 255, 255, 255))
white.save(os.path.join(out_dir, 'white.png'))
glow = Image.new('L', (1, 32))
for y in range(32):
    glow.putpixel((0, y), int(255 * np.exp(-((y - 26) / 6.0) ** 2)))
g = Image.new('RGBA', (1, 32), CYAN[:3] + (255,))
g.putalpha(glow)
g.save(os.path.join(out_dir, 'bar_glow.png'))

with open(os.path.join(out_dir, 'manifest.json'), 'w', encoding='utf-8') as f:
    json.dump(manifest, f, indent=1)


def lua(v):
    if isinstance(v, dict):
        return '{' + ', '.join(f'[{lua(k)}] = {lua(x)}' for k, x in v.items()) + '}'
    if isinstance(v, (list, tuple)):
        return '{' + ', '.join(lua(x) for x in v) + '}'
    if isinstance(v, str):
        return json.dumps(v)
    return repr(float(v))


with open(os.path.join(out_dir, 'manifest.lua'), 'w', encoding='utf-8') as f:
    f.write('return ' + lua(manifest) + chr(10))
print(len(manifest['backgrounds']), 'backgrounds,', len(tips), 'tips,', len(manifest['sprites']), 'sprites')
