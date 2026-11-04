#!/usr/bin/env python3
"""Fetch arXiv candidates for the daily autonomous-driving digest.

Writes <output_dir>/<date>/candidates.json with compact records so Claude
reads as few tokens as possible. Already-published papers (state/seen.json)
are excluded.
"""
import datetime as dt
import json
import os
import sys
import time
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET

SKILL_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
API = "https://export.arxiv.org/api/query"
NS = {"a": "http://www.w3.org/2005/Atom", "arxiv": "http://arxiv.org/schemas/atom"}


def load_json(path, default):
    try:
        with open(path) as f:
            return json.load(f)
    except FileNotFoundError:
        return default


def query(search, sort, max_results):
    params = urllib.parse.urlencode({
        "search_query": search,
        "sortBy": sort,
        "sortOrder": "descending",
        "max_results": max_results,
    })
    req = urllib.request.Request(f"{API}?{params}", headers={"User-Agent": "arxiv-daily/1.0"})
    with urllib.request.urlopen(req, timeout=60) as r:
        root = ET.fromstring(r.read())
    out = []
    for e in root.findall("a:entry", NS):
        aid = e.find("a:id", NS).text.rsplit("/abs/", 1)[-1]
        base_id = aid.split("v")[0] if "v" in aid else aid
        authors = [a.find("a:name", NS).text for a in e.findall("a:author", NS)]
        out.append({
            "id": base_id,
            "title": " ".join(e.find("a:title", NS).text.split()),
            "authors": authors[:3] + (["et al."] if len(authors) > 3 else []),
            "published": e.find("a:published", NS).text[:10],
            "cats": [c.get("term") for c in e.findall("a:category", NS)][:3],
            "abstract": " ".join(e.find("a:summary", NS).text.split()),
        })
    return out


def main():
    cfg = load_json(os.path.join(SKILL_DIR, "config.json"), {})
    seen = set(load_json(os.path.join(SKILL_DIR, "state", "seen.json"), []))
    today = dt.date.today()
    out_dir = os.path.join(os.path.expanduser(cfg["output_dir"]), today.isoformat())
    os.makedirs(out_dir, exist_ok=True)

    cats = "(" + " OR ".join(f"cat:{c}" for c in cfg["categories"]) + ")"
    core = "(" + " OR ".join(f'abs:"{t}"' for t in cfg["core_terms"]) + ")"
    focus = "(" + " OR ".join(f'abs:"{t}"' for t in cfg["focus_terms"]) + ")"

    # 1) newest submissions within the focus areas
    new = [p for p in query(f"{core} AND {focus} AND {cats}", "submittedDate", cfg["new_candidates"] + 20)
           if p["id"] not in seen][: cfg["new_candidates"]]
    time.sleep(3)  # arXiv API etiquette

    # 2) related (older, by relevance) - rotate two sub-topics per day
    topics = cfg["related_topics"]
    idx = today.toordinal() * 2 % len(topics)
    todays = [topics[idx], topics[(idx + 1) % len(topics)]]
    new_ids = {p["id"] for p in new}
    related = []
    for t in todays:
        words = " AND ".join(f"abs:{w}" for w in t.replace("-", " ").split())
        q = f"{core} AND ({words}) AND {cats}"
        related += [p for p in query(q, "relevance", cfg["related_candidates"] + 20)
                    if p["id"] not in seen and p["id"] not in new_ids
                    and p["id"] not in {r["id"] for r in related}][: cfg["related_candidates"]]
        time.sleep(3)

    n = cfg["abstract_chars"]
    for p in new + related:
        if len(p["abstract"]) > n:
            p["abstract"] = p["abstract"][:n].rsplit(" ", 1)[0] + " …"

    result = {"date": today.isoformat(), "related_topics": todays, "total": cfg["total"],
              "new": new, "related": related}
    path = os.path.join(out_dir, "candidates.json")
    with open(path, "w") as f:
        json.dump(result, f, ensure_ascii=False, separators=(",", ":"))
    print(f"{path}\nnew={len(new)} related={len(related)} topics={todays} size={os.path.getsize(path)}B")


if __name__ == "__main__":
    sys.exit(main())
