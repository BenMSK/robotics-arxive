"""Find arXiv ids for pool entries by exact(ish) title search.
usage: python -I tools/resolve_arxiv.py idx idx ...   (updates _raw/pool.jsonl in place)
"""
import difflib, html, json, re, sys, time, urllib.parse, urllib.request

UA = 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/130.0 Safari/537.36'
norm = lambda t: re.sub(r'[^a-z0-9]', '', html.unescape(re.sub(r'<[^>]+>', '', t)).lower())
strip = lambda t: re.sub(r'\s+', ' ', html.unescape(re.sub(r'<[^>]+>', '', t))).strip()
pool = [json.loads(l) for l in open('_raw/pool.jsonl', encoding='utf-8')]
for i in map(int, sys.argv[1:]):
    r = pool[i]
    if r.get('arxiv'):
        continue
    q = re.sub(r'[*:]|\(I\)$', ' ', r['title']).strip()
    url = 'https://arxiv.org/search/?' + urllib.parse.urlencode(dict(query=q, searchtype='all', abstracts='show', size=25))
    try:
        s = urllib.request.urlopen(urllib.request.Request(url, headers={'User-Agent': UA}), timeout=60).read().decode('utf-8', 'replace')
    except Exception as e:
        print(i, 'ERR', e); continue
    hit = None
    for it in s.split('<li class="arxiv-result">')[1:]:
        t = re.search(r'<p class="title is-5 mathjax">(.*?)</p>', it, re.S)
        if t and difflib.SequenceMatcher(None, norm(t.group(1)), norm(q)).ratio() > 0.85:
            hit = it
            break
    if hit:
        r['arxiv'] = re.search(r'arxiv\.org/abs/([0-9.]+)', hit).group(1)
        ab = re.search(r'<span class="abstract-full[^"]*"[^>]*>(.*?)<a', hit, re.S)
        if not r.get('abstract') and ab:
            r['abstract'] = strip(ab.group(1))
    print(i, r.get('arxiv', '-'), r['title'][:80], flush=True)
    time.sleep(3)
with open('_raw/pool.jsonl', 'w', encoding='utf-8') as f:
    for r in pool:
        f.write(json.dumps(r, ensure_ascii=False) + '\n')
