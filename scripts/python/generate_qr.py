#!/usr/bin/env python

import qrcode
import random
from PIL import ImageOps
from PIL import Image

# If background is false, it will be black
# If true, will be white
def generate_qr(text, filename, bg=False):
    qr = qrcode.QRCode(
            version=1,
            error_correction=qrcode.constants.ERROR_CORRECT_L,
            box_size=10,
            border=1,
            )
    qr.add_data(text)
    qr.make(fit=True)

    back = "black"
    fill = "white"
    if bg:
        back = "white"
        fill = "black"

    img = qr.make_image(fill_color=fill, back_color=back)
    img.save(filename + ".png")
    return img

def seg_rotate(img, filename):
    w, h = img.size
    half_w = w / 2
    half_h = h / 2

    top_l = img.crop((0, 0, half_w, half_h))
    top_l = ImageOps.expand(top_l, border=(0, 0, half_w, half_h))
    top_l.rotate(random.randint(-30, 30), expand=True).save(filename + "_1.png")

    top_r = img.crop((half_w, 0, w, half_h))
    top_r = ImageOps.expand(top_r, border=(half_w, 0, 0, half_h))
    top_r.rotate(random.randint(-30, 30), expand=True).save(filename + "_2.png")

    bot_l = img.crop((0, half_h, half_w, h))
    bot_l = ImageOps.expand(bot_l, border=(0, half_h, half_w, 0))
    bot_l.rotate(random.randint(-30, 30), expand=True).save(filename + "_3.png")

    bot_r = img.crop((half_w, half_h, w, h))
    bot_r = ImageOps.expand(bot_r, border=(half_w, half_h, 0, 0))
    bot_r.rotate(random.randint(-30, 30), expand=True).save(filename + "_4.png")

def seg(img, filename):
    w, h = img.size
    half_w = w / 2
    half_h = h / 2

    top_l = img.crop((0, 0, half_w, half_h))
    top_l = ImageOps.expand(top_l, border=(0, 0, half_w, half_h))
    top_l.save(filename + "_1s.png")

    top_r = img.crop((half_w, 0, w, half_h))
    top_r = ImageOps.expand(top_r, border=(half_w, 0, 0, half_h))
    top_r.save(filename + "_2s.png")

    bot_l = img.crop((0, half_h, half_w, h))
    bot_l = ImageOps.expand(bot_l, border=(0, half_h, half_w, 0))
    bot_l.save(filename + "_3s.png")

    bot_r = img.crop((half_w, half_h, w, h))
    bot_r = ImageOps.expand(bot_r, border=(half_w, half_h, 0, 0))
    bot_r.save(filename + "_4s.png")

def aligned(img, filename):
    w, h = img.size
    half_w = w / 2
    half_h = h / 2

    top_l = img.crop((0, 0, half_w, half_h))
    top_r = img.crop((half_w, 0, w, half_h))
    bot_l = img.crop((0, half_h, half_w, h))
    bot_r = img.crop((half_w, half_h, w, h))

    new = Image.new("L", (w + 20, h + 20))
    new.paste(top_l)
    new.paste(top_r, (half_w + 20, 0))
    new.paste(bot_l, (0, half_h + 20))
    new.paste(bot_r, (half_w + 20, half_h + 20))
    new.save(filename + "_a.png")

def main():
    base = "/tmp/test"
    for i in range(10):
        num = random.randint(1000,9999)
        filename = base + str(num)
        qr = generate_qr(num, filename, bg=True)
        seg_rotate(qr, filename)
        seg(qr, filename)
        aligned(qr, filename)

main()
