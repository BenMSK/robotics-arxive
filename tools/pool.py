"""Merge venue lists + arXiv-tagged lists into one pool and tag topic relevance.
usage: python -I tools/pool.py   (reads _raw/*.jsonl, writes _raw/pool.jsonl)
"""
import json, re, glob

load = lambda p: [json.loads(l) for l in open(p, encoding='utf-8')]
norm = lambda t: re.sub(r'[^a-z0-9]', '', t.lower())

arx = {}
for p in glob.glob('_raw/*_arxiv.jsonl'):
    for r in load(p):
        # "submitted to X" / "under review" is not an acceptance
        c = r['comments'].lower()
        r['accepted'] = not re.search(r'submit|under review|in review', c) or bool(re.search(r'accept|to appear|camera', c))
        arx.setdefault(norm(r['title']), r)

pool = []
for p, venue in [('_raw/icra2026.jsonl', 'ICRA 2026'), ('_raw/rss2026.jsonl', 'RSS 2026'), ('_raw/corl2026.jsonl', 'CoRL 2026')]:
    for r in load(p):
        a = arx.get(norm(r['title']))
        if a:
            r['arxiv'] = a['arxiv']
            r['abstract'] = r.get('abstract') or a['abstract']
        r.setdefault('abstract', '')
        pool.append(r)
# IROS: no official list reachable (Cloudflare); use accepted arXiv-tagged papers
pool += [dict(r, venue='IROS 2026') for r in arx.values() if r['venue'] == 'IROS 2026' and r['accepted']]

SLAM = r'\bslam\b|odometr|locali[sz]ation|state estimation|place recognition|loop clos|registration|factor graph|pose graph|lidar.inertial|visual.inertial|\bicp\b|mapping'
ROBUST = r'robust|degenera|outlier|dynamic (environment|scene|object)|adverse|weather|resilien|failure|uncertain|certifi|drift|degraded|featureless|geometrically|corrupt|noise|long-term|lifelong'
PLAN = r'motion planning|trajectory (planning|generation|optimi[sz]ation)|path planning|planner|navigation'
AI = r'learn|diffusion|neural|transformer|reinforcement|imitation|foundation model|language model|\bllm|\bvlm|vision-language|data-driven|generative|flow matching|world model'
DRIVE = r'parking|valet|autonomous driving|autonomous vehicle|self-driving|nonholonomic|car-like|vehicle|end-to-end driving'
for r in pool:
    t = (r['title'] + ' ' + r['abstract'] + ' ' + ' '.join(r.get('keywords', []))).lower()
    tags = []
    if re.search(SLAM, t): tags.append('slam' + ('+robust' if re.search(ROBUST, t) else ''))
    if re.search(PLAN, t) and re.search(AI, t): tags.append('aiplan')
    if re.search(DRIVE, t): tags.append('drive')
    if re.search(r'parking|valet', t): tags.append('parking')
    r['tags'] = tags
with open('_raw/pool.jsonl', 'w', encoding='utf-8') as f:
    for r in pool:
        f.write(json.dumps(r, ensure_ascii=False) + '\n')

from collections import Counter
for v in ['ICRA 2026', 'IROS 2026', 'RSS 2026', 'CoRL 2026']:
    rs = [r for r in pool if r['venue'] == v]
    c = Counter(t for r in rs for t in r['tags'])
    print(v, len(rs), 'papers, arxiv:', sum('arxiv' in r for r in rs), dict(c))
