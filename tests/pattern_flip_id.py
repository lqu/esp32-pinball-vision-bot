#!/usr/bin/env python3
"""
pattern_flip_id.py
Full-screen black/white flip + small self-identifying ID code block
for camera/processor responsiveness + latency testing.

Requirements:
  pip install pygame

Run examples:
  python3 pattern_flip_id.py
  python3 pattern_flip_id.py --hz 2
  python3 pattern_flip_id.py --hz 10 --code-px 320 --corner tr
  python3 pattern_flip_id.py --windowed --width 1280 --height 720

Keys:
  ESC or Q  -> quit
"""

import argparse
import time
import pygame

def popcount(x: int) -> int:
    return bin(x & 0xFFFFFFFF).count("1")

def parity8(x: int) -> int:
    return popcount(x) & 1

def make_grid_bits(frame_id: int, grid_n: int = 8):
    """
    Returns an grid_n x grid_n list of 0/1 bits.
    Includes an asymmetric finder pattern + 16-bit payload + parity + whitening tail.
    """
    g = [[0 for _ in range(grid_n)] for _ in range(grid_n)]

    # Finder pattern: row0=1s, col0=1s, (1,1)=0 to break symmetry
    for c in range(grid_n):
        g[0][c] = 1
    for r in range(grid_n):
        g[r][0] = 1
    g[1][1] = 0

    reserved = set()
    for c in range(grid_n):
        reserved.add((0, c))
    for r in range(grid_n):
        reserved.add((r, 0))
    reserved.add((1, 1))

    cells = [(r, c) for r in range(grid_n) for c in range(grid_n) if (r, c) not in reserved]

    payload = [(frame_id >> i) & 1 for i in range(16)]  # LSB first
    p_lo = parity8(frame_id & 0xFF)
    p_hi = parity8((frame_id >> 8) & 0xFF)

    bits = payload + payload + [p_lo, p_hi]  # redundancy

    # Whitening tail so nearby IDs still create lots of toggling
    seed = (frame_id * 1103515245 + 12345) & 0x7fffffff
    while len(bits) < len(cells):
        seed ^= (seed << 13) & 0x7fffffff
        seed ^= (seed >> 17)
        seed ^= (seed << 5) & 0x7fffffff
        bits.append(seed & 1)

    for (r, c), b in zip(cells, bits):
        g[r][c] = b

    return g

def corner_xy(screen_w: int, screen_h: int, block_w: int, block_h: int, corner: str, margin: int):
    corner = corner.lower()
    if corner == "tl":
        return margin, margin
    if corner == "tr":
        return screen_w - margin - block_w, margin
    if corner == "bl":
        return margin, screen_h - margin - block_h
    if corner == "br":
        return screen_w - margin - block_w, screen_h - margin - block_h
    raise ValueError("corner must be one of: tl,tr,bl,br")

def draw_code(surface: pygame.Surface, x0: int, y0: int, size_px: int, frame_id: int,
              grid_n: int = 8, invert: bool = False):
    """
    Draw an NxN tile code snapped to whole-tile pixels.
    Returns actual drawn size (snapped).
    """
    cell = max(1, size_px // grid_n)
    size_px = cell * grid_n

    g = make_grid_bits(frame_id, grid_n=grid_n)
    if invert:
        g = [[1 - b for b in row] for row in g]

    # border for thresholding
    border = max(2, cell // 6)
    pygame.draw.rect(surface, (0, 0, 0), (x0 - border, y0 - border, size_px + 2 * border, size_px + 2 * border), 0)

    for r in range(grid_n):
        for c in range(grid_n):
            v = g[r][c]
            color = (255, 255, 255) if v else (0, 0, 0)
            pygame.draw.rect(surface, color, (x0 + c * cell, y0 + r * cell, cell, cell), 0)

    return size_px

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=float, default=10.0,
                    help="flip frequency (full-screen toggles) in Hz, e.g. 2, 5, 10")
    ap.add_argument("--code-px", type=int, default=240,
                    help="size of the ID code square in pixels (snapped to tile size)")
    ap.add_argument("--corner", type=str, default="tr", choices=["tl", "tr", "bl", "br"],
                    help="which corner to place the ID code block")
    ap.add_argument("--margin", type=int, default=24,
                    help="margin from screen edge (pixels)")
    ap.add_argument("--grid", type=int, default=8,
                    help="grid dimension (NxN tiles), default 8")
    ap.add_argument("--no-invert", action="store_true",
                    help="do NOT invert the code on odd ticks (by default it inverts to maximize diff)")
    ap.add_argument("--no-text", action="store_true",
                    help="do NOT draw text label next to the code block")
    ap.add_argument("--windowed", action="store_true",
                    help="run windowed (useful if fullscreen is annoying)")
    ap.add_argument("--width", type=int, default=1280, help="windowed mode width")
    ap.add_argument("--height", type=int, default=720, help="windowed mode height")
    ap.add_argument("--fps-cap", type=int, default=240,
                    help="cap the render loop to this FPS (0 = uncapped)")
    args = ap.parse_args()

    if args.hz <= 0:
        raise SystemExit("--hz must be > 0")

    pygame.init()
    flags = pygame.DOUBLEBUF

    # Try vsync if pygame supports it (2.0+). Some backends ignore it.
    vsync_arg = {}
    try:
        if args.windowed:
            screen = pygame.display.set_mode((args.width, args.height), flags, vsync=1)
        else:
            screen = pygame.display.set_mode((0, 0), flags | pygame.FULLSCREEN, vsync=1)
    except TypeError:
        # Older pygame without vsync kwarg
        if args.windowed:
            screen = pygame.display.set_mode((args.width, args.height), flags)
        else:
            screen = pygame.display.set_mode((0, 0), flags | pygame.FULLSCREEN)

    info = pygame.display.Info()
    W, H = info.current_w, info.current_h

    font = pygame.font.SysFont("monospace", 28, bold=True)

    period = 1.0 / args.hz
    t0 = time.perf_counter()
    tick = 0

    clock = pygame.time.Clock()

    print(f"# pattern_flip_id.py hz={args.hz} code_px={args.code_px} corner={args.corner} grid={args.grid}")
    print("# t_perf_s, frame_id, bg(0=black,1=white), invert_code(0/1)", flush=True)

    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN and e.key in (pygame.K_ESCAPE, pygame.K_q):
                running = False

        # Advance stimulus tick based on perf_counter (independent of render FPS)
        t = time.perf_counter()
        new_tick = int((t - t0) / period)
        if new_tick != tick:
            tick = new_tick
            bg = tick & 1
            inv = 0 if args.no_invert else (tick & 1)
            print(f"{t:.6f}, {tick & 0xFFFF}, {bg}, {inv}", flush=True)

        bg = tick & 1
        invert = False if args.no_invert else bool(tick & 1)

        if bg:
            screen.fill((255, 255, 255))
            fg_text = (0, 0, 0)
        else:
            screen.fill((0, 0, 0))
            fg_text = (255, 255, 255)

        # Draw code
        code_size = max(32, args.code_px)
        cx, cy = corner_xy(W, H, code_size, code_size, args.corner, args.margin)
        actual = draw_code(screen, cx, cy, code_size, frame_id=(tick & 0xFFFF),
                           grid_n=args.grid, invert=invert)

        # Optional text label
        if not args.no_text:
            label = f"ID={tick & 0xFFFF:05d}  Hz={args.hz:g}  (ESC/Q quit)"
            surf = font.render(label, True, fg_text)
            tx, ty = cx, cy + actual + 10
            # keep visible
            if ty + surf.get_height() + 4 > H:
                ty = max(args.margin, cy - surf.get_height() - 10)
            screen.blit(surf, (tx, ty))

        pygame.display.flip()

        if args.fps_cap > 0:
            clock.tick(args.fps_cap)

    pygame.quit()

if __name__ == "__main__":
    main()
