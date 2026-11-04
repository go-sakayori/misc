#!/usr/bin/env python3
"""Download one representative figure per paper from the arXiv HTML version.

Saves <day_dir>/figs/<id>.jpg and <id>.txt (short caption). Papers without an
HTML version (or without a usable figure) are skipped silently; build.py then
falls back to the text-only layout. Runs without Claude reading anything.
"""
import html
import io
import os
import re
import sys
import time
import urllib.parse
import urllib.request

from PIL import Image

UA = {"User-Agent": "arxiv-daily/1.0"}
PREFER = re.compile(r"overview|pipeline|framework|architecture|approach|method", re.I)
MAX_SIDE = 1600
MIN_WIDTH = 300


def get(url):
    with urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=30) as r:
        return r.read()


def pick_figure(page):
    """Return (img_src, caption) of the preferred figure among the first three."""
    figs = []
    for m in re.finditer(r'<figure[^>]*class="ltx_figure[" ]', page):
        chunk = page[m.start(): m.start() + 20000]
        img = re.search(r'<img[^>]*src="([^"]+\.(?:png|jpe?g|gif))"', chunk, re.I)
        cap = re.search(r"<figcaption[^>]*>(.*?)</figcaption>", chunk, re.S)
        if img:
            text = html.unescape(re.sub(r"<[^>]+>", "", cap.group(1))) if cap else ""
            figs.append((img.group(1), " ".join(text.split())))
        if len(figs) == 3:
            break
    if not figs:
        return None
    return next((f for f in figs if PREFER.search(f[1])), figs[0])


def fetch_one(aid, out_dir):
    jpg = os.path.join(out_dir, f"{aid}.jpg")
    if os.path.exists(jpg):
        return True
    base = f"https://arxiv.org/html/{aid}"
    try:
        found = pick_figure(get(base).decode("utf-8", "replace"))
        if not found:
            return False
        src, caption = found
        im = Image.open(io.BytesIO(get(urllib.parse.urljoin(base, src))))
    except Exception:
        return False
    if im.width < MIN_WIDTH:
        return False
    if im.mode in ("RGBA", "LA", "P"):
        im = im.convert("RGBA")
        bg = Image.new("RGB", im.size, "white")
        bg.paste(im, mask=im.split()[-1])
        im = bg
    im = im.convert("RGB")
    im.thumbnail((MAX_SIDE, MAX_SIDE))
    im.save(jpg, "JPEG", quality=85)
    with open(os.path.join(out_dir, f"{aid}.txt"), "w") as f:
        f.write(caption[:160] + ("…" if len(caption) > 160 else ""))
    return True


def fetch_all(day_dir, ids):
    out_dir = os.path.join(day_dir, "figs")
    os.makedirs(out_dir, exist_ok=True)
    ok = 0
    for aid in ids:
        ok += fetch_one(aid, out_dir)
        time.sleep(1)  # be polite to arxiv.org
    print(f"figures: {ok}/{len(ids)}")


if __name__ == "__main__":
    import json
    day = sys.argv[1]
    fetch_all(day, [p["id"] for p in json.load(open(os.path.join(day, "summaries.json")))["papers"]])
