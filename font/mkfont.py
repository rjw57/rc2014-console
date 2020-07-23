#!/usr/bin/env python3
"""
Convert font image into compiled font.

Usage:
    mkfont.py (-h | --help)
    mkfont.py <image> <output>
"""
import docopt
from PIL import Image

PREFIX = '''#include "font.h"

PROGMEM __attribute__ ((aligned(256))) const unsigned char font_data[] = {
'''

SUFFIX = '''};
'''


def main():
    opts = docopt.docopt(__doc__)
    image = Image.open(opts['<image>']).convert("L")
    outf = open(opts['<output>'], 'w')
    threshold = sum(image.getextrema()) >> 1

    im_horiz_chars = image.width >> 3
    im_vert_chars = image.height >> 3

    # Array to hold each character's rows.
    chars = [list() for _ in range(im_horiz_chars * im_vert_chars)]

    char_idx = 0
    for y in range(0, image.height, 8):
        for x in range(0, image.width, 8):
            char_im = image.crop((x, y, x+8, y+8))

            char_rows = chars[char_idx]
            char_idx += 1

            for r in range(8):
                row_byte = 0xff
                for b in range(8):
                    if char_im.getpixel((b, r)) >= threshold:
                        row_byte ^= 1 << (7-b)
                char_rows.append(row_byte)

    # We output one row for each character.
    outf.write(PREFIX)
    for r_idx in range(8):
        outf.write('  ')
        outf.write(', '.join(
            f'0x{rs[r_idx]:02x}' for rs in chars
        ))
        outf.write(',\n')
    outf.write(SUFFIX)


if __name__ == '__main__':
    main()
