#!/usr/bin/env python3
"""Convert an X11 `xwd -root` dump to PNG using only the Python stdlib.

Usage: python3 xwd2png.py in.xwd out.png

Host has no ImageMagick / PIL, so we parse the big-endian XWD header, skip the
colormap, read the TrueColor pixel data and emit a PNG via zlib+struct.
"""
import sys, struct, zlib

def main(src, dst):
    d = open(src, "rb").read()
    # XWD header is big-endian, 32-bit fields.
    hdr = struct.unpack(">25I", d[:100])
    (hsize, ver, pixfmt, depth, width, height, xoff, byte_order,
     bmap_unit, bmap_bit_order, bmap_pad, bpp, bytes_per_line, vclass,
     rmask, gmask, bmask, bits_per_rgb, cmap_entries, ncolors,
     win_w, win_h, win_x, win_y, win_bdr) = hdr
    if ver != 7:
        raise SystemExit(f"unexpected XWD version {ver}")
    # pixel data starts after the (variable-length) header + colormap.
    off = hsize + ncolors * 12

    def shift_of(mask):
        s = 0
        while mask and not (mask & 1):
            mask >>= 1; s += 1
        return s
    rs, gs, bs = shift_of(rmask), shift_of(gmask), shift_of(bmask)
    nbytes = bpp // 8

    rows = bytearray()
    for y in range(height):
        line = d[off + y * bytes_per_line: off + y * bytes_per_line + width * nbytes]
        rows.append(0)  # PNG filter type 0
        for x in range(width):
            px = line[x * nbytes: x * nbytes + nbytes]
            if byte_order == 0:          # LSBFirst
                val = int.from_bytes(px, "little")
            else:                        # MSBFirst
                val = int.from_bytes(px, "big")
            rows.append((val & rmask) >> rs)
            rows.append((val & gmask) >> gs)
            rows.append((val & bmask) >> bs)

    def chunk(tag, data):
        c = tag + data
        return struct.pack(">I", len(data)) + c + struct.pack(">I", zlib.crc32(c) & 0xffffffff)

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(bytes(rows), 9))
    png += chunk(b"IEND", b"")
    open(dst, "wb").write(png)
    print(f"wrote {dst} {width}x{height} (bpp={bpp})")

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
