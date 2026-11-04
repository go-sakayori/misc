#!/usr/bin/env python3
"""Render <date>/summaries.json into a fixed-format 16:9 deck.

summaries.json (written by Claude) only holds the id and the summary text;
titles, authors and dates are joined from candidates.json.

  {"papers": [{"id": "2609.01234", "tag": "Planning",
               "oneliner": "...", "problem": "...", "method": "...",
               "result": "...", "why": "..."}]}

Fetches one figure per paper (figures.py), outputs
<date>/arxiv-daily-<date>.pptx and records
the ids in state/seen.json.
"""
import io
import json
import os
import re
import sys
import zipfile

import figures

from PIL import Image
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.util import Emu, Pt

SKILL_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FONT = "Noto Sans JP"
INK = RGBColor(0x1F, 0x23, 0x28)
MUTED = RGBColor(0x5F, 0x66, 0x6D)
ACCENT = RGBColor(0x1A, 0x73, 0xE8)
NEW_C = RGBColor(0x18, 0x80, 0x38)
REL_C = RGBColor(0xB0, 0x60, 0x00)
W, H = Emu(12192000), Emu(6858000)  # 16:9
M = Emu(457200)  # 0.5in margin


def text(slide, x, y, w, h, runs, size=14, color=INK, bold=False):
    """runs: str, or list of paragraphs; a paragraph is str or list of (text, opts)."""
    tb = slide.shapes.add_textbox(x, y, w, h)
    tf = tb.text_frame
    tf.word_wrap = True
    paras = [runs] if isinstance(runs, str) else runs
    for i, para in enumerate(paras):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        for seg in ([(para, {})] if isinstance(para, str) else para):
            t, o = seg
            r = p.add_run()
            r.text = t
            f = r.font
            f.name = FONT
            f.size = Pt(o.get("size", size))
            f.bold = o.get("bold", bold)
            f.color.rgb = o.get("color", color)
            if o.get("link"):
                r.hyperlink.address = o["link"]
    return tb


def paper_slide(prs, n, p, fig_dir):
    s = prs.slides.add_slide(prs.slide_layouts[6])
    kind_c = NEW_C if p["kind"] == "new" else REL_C
    kind_t = "NEW" if p["kind"] == "new" else "関連"
    text(s, M, Emu(300000), W - 2 * M, Emu(300000),
         [[(f"#{n}  ", {"color": MUTED, "bold": True}), (kind_t, {"color": kind_c, "bold": True}),
           (f"  ·  {p.get('tag', '')}", {"color": MUTED})]], size=12)
    text(s, M, Emu(600000), W - 2 * M, Emu(900000), p["title"], size=22, bold=True)
    url = f"https://arxiv.org/abs/{p['id']}"
    text(s, M, Emu(1500000), W - 2 * M, Emu(300000),
         [[(", ".join(p["authors"]) + f"  ·  {p['published']}  ·  {' '.join(p['cats'])}  ·  ", {}),
           (url, {"color": ACCENT, "link": url})]], size=11, color=MUTED)

    fig = os.path.join(fig_dir, f"{p['id']}.jpg")
    ratio = None
    if os.path.exists(fig):
        with Image.open(fig) as im:
            ratio = im.width / im.height
    # layouts: "side" = summary left 57% + figure right, "wide" = summary on top +
    # full-width figure at the bottom (for figures >= 2:1), "text" = no figure
    layout = "text" if ratio is None else ("wide" if ratio >= 2.0 else "side")
    text_w = Emu(6800000) if layout == "side" else W - 2 * M
    size = 13 if layout == "text" else 12
    text(s, M, Emu(1950000), text_w, Emu(500000),
         [[("一言で  ", {"color": ACCENT, "bold": True}), (p["oneliner"], {"bold": True})]], size=size + 3)
    rows = [("課題", p["problem"]), ("手法", p["method"]), ("結果", p["result"]), ("なぜ重要", p["why"])]
    body = []
    for label, t in rows:
        if layout == "wide":  # label and body on one line to save height
            body.append([(label + "  ", {"color": ACCENT, "bold": True}), (t, {})])
        else:
            body += [[(label, {"color": ACCENT, "bold": True})], [(t, {})]]
    body_h = Emu(1900000) if layout == "wide" else H - Emu(2800000)
    text(s, M, Emu(2450000), text_w, body_h, body, size=size)
    if layout == "text":
        return

    if layout == "side":
        box_x, box_y = M + text_w + Emu(250000), Emu(2000000)
        box_w, box_h = W - M - box_x, Emu(3900000)
    else:
        box_x, box_y = M, Emu(4450000)
        box_w, box_h = W - 2 * M, H - box_y - Emu(350000)
    w, h = (box_w, int(box_w / ratio)) if ratio >= box_w / box_h else (int(box_h * ratio), box_h)
    s.shapes.add_picture(fig, box_x + (box_w - w) // 2, box_y, Emu(w), Emu(h))
    if layout == "side":  # the wide layout has no room left for the caption
        cap_path = os.path.join(fig_dir, f"{p['id']}.txt")
        cap = open(cap_path).read() if os.path.exists(cap_path) else ""
        text(s, box_x, box_y + Emu(h) + Emu(80000), box_w, Emu(600000), cap, size=9, color=MUTED)


def shrink(pptx_bytes):
    """Drop the thumbnail, printer settings and unused layouts, recompress.

    Keeps the file small. Only slideLayout7 (Blank) is used.
    """
    src = zipfile.ZipFile(io.BytesIO(pptx_bytes))
    drop = re.compile(r"docProps/thumbnail\.jpeg|ppt/printerSettings/.*|"
                      r"ppt/slideLayouts/(_rels/)?slideLayout(?!7\.)\d+\.xml(\.rels)?")
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as dst:
        for item in src.infolist():
            if drop.fullmatch(item.filename):
                continue
            data = src.read(item.filename)
            if not item.filename.endswith((".rels", ".xml")):
                dst.writestr(item, data)  # media etc. stay binary
                continue
            data = data.decode()
            data = re.sub(r'<Relationship [^>]*Target="[^"]*(thumbnail\.jpeg|printerSettings\d*\.bin|'
                          r'slideLayout(?!7\.)\d+\.xml)"/>', "", data)
            data = re.sub(r'<Override PartName="/(docProps/thumbnail\.jpeg|ppt/printerSettings/[^"]*|'
                          r'ppt/slideLayouts/slideLayout(?!7\.)\d+\.xml)"[^>]*/>', "", data)
            if item.filename == "ppt/slideMasters/slideMaster1.xml":
                keep = re.search(r'Id="(rId\d+)"[^>]*Target="../slideLayouts/slideLayout7.xml"',
                                 src.read("ppt/slideMasters/_rels/slideMaster1.xml.rels").decode()).group(1)
                data = re.sub(r'<p:sldLayoutId [^>]*r:id="(?!%s")rId\d+"/>' % keep, "", data)
            dst.writestr(item, data.encode())
    return buf.getvalue()

def main(day_dir):
    cands = json.load(open(os.path.join(day_dir, "candidates.json")))
    meta = {p["id"]: dict(p, kind="new") for p in cands["new"]}
    meta.update({p["id"]: dict(p, kind="related") for p in cands["related"]})
    papers = []
    for s in json.load(open(os.path.join(day_dir, "summaries.json")))["papers"]:
        if s["id"] not in meta:
            sys.exit(f"unknown id in summaries.json: {s['id']}")
        papers.append({**meta[s["id"]], **s})
    date = cands["date"]

    prs = Presentation()
    prs.slide_width, prs.slide_height = W, H
    cover = prs.slides.add_slide(prs.slide_layouts[6])
    n_new = sum(p["kind"] == "new" for p in papers)
    text(cover, M, Emu(2200000), W - 2 * M, Emu(900000), "arXiv Daily — 自動運転", size=36, bold=True)
    text(cover, M, Emu(3100000), W - 2 * M, Emu(1200000),
         [date, f"{len(papers)}本(NEW {n_new} / 関連 {len(papers) - n_new})  ·  関連テーマ: {', '.join(cands['related_topics'])}"],
         size=16, color=MUTED)

    for start in range(0, len(papers), 10):
        idx = prs.slides.add_slide(prs.slide_layouts[6])
        text(idx, M, Emu(300000), W - 2 * M, Emu(500000), f"今日の一覧 ({start + 1}–{min(start + 10, len(papers))})",
             size=20, bold=True)
        lines = [[(f"{i}. ", {"color": MUTED, "bold": True}),
                  ("NEW " if p["kind"] == "new" else "関連 ", {"color": NEW_C if p["kind"] == "new" else REL_C, "bold": True}),
                  (p["oneliner"], {})]
                 for i, p in enumerate(papers[start:start + 10], start + 1)]
        text(idx, M, Emu(950000), W - 2 * M, H - Emu(1300000), lines, size=13)

    fig_dir = os.path.join(day_dir, "figs")
    figures.fetch_all(day_dir, [p["id"] for p in papers])
    for i, p in enumerate(papers, 1):
        paper_slide(prs, i, p, fig_dir)

    buf = io.BytesIO()
    prs.save(buf)
    data = shrink(buf.getvalue())
    pptx_path = os.path.join(day_dir, f"arxiv-daily-{date}.pptx")
    with open(pptx_path, "wb") as f:
        f.write(data)

    seen_path = os.path.join(SKILL_DIR, "state", "seen.json")
    seen = json.load(open(seen_path)) if os.path.exists(seen_path) else []
    seen += [p["id"] for p in papers if p["id"] not in seen]
    json.dump(seen, open(seen_path, "w"))

    print(f"{pptx_path}\nslides={len(prs.slides)} pptx={len(data)}B")


if __name__ == "__main__":
    main(sys.argv[1])
