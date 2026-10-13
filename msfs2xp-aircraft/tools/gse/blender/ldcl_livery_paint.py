import sys

from PIL import Image, ImageDraw, ImageFont

out = sys.argv[1]
schemes = {
    'dnata': ((0, 74, 140), (255, 255, 255), 'dnata'),
    'swissport': ((200, 20, 30), (255, 255, 255), 'Swissport'),
    'menzies': ((20, 70, 120), (255, 205, 0), 'MENZIES'),
}
for operator, (background, foreground, text) in schemes.items():
    image = Image.new('RGBA', (1000, 100), background + (255,))
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 84, 1000, 100), fill=foreground + (255,))
    try:
        font = ImageFont.truetype('arialbd.ttf', 64)
    except OSError:
        font = ImageFont.load_default()
    draw.text((40, 8), text + '  GROUND HANDLING', fill=foreground + (255,), font=font)
    image.save(f'{out}/{operator}.png')
