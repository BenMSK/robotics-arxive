"""Scrape arXiv HTML search results (all pages) into JSONL.
usage: python -I tools/arxiv_search.py VENUE out.jsonl searchtype "query" ["query" ...]
  searchtype: comments | title | all
"""
import html, json, re, sys, time, urllib.parse, urllib.request

venue, out, stype, queries = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4:]
UA = 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/130.0 Safari/537.36'
strip = lambda t: re.sub(r'\s+', ' ', html.unescape(re.sub(r'<[^>]+>', '', t))).strip()


def page(q, start):
    url = 'https://arxiv.org/search/?' + urllib.parse.urlencode(dict(
        query=q, searchtype=stype, abstracts='show', size=200, order='-announced_date_first', start=start))
    req = urllib.request.Request(url, headers={'User-Agent': UA})
    return urllib.request.urlopen(req, timeout=60).read().decode('utf-8', 'replace')


rows = {}
for q in queries:
    start = 0
    while True:
        s = page(q, start)
        items = s.split('<li class="arxiv-result">')[1:]
        for it in items:
            aid = re.search(r'arxiv\.org/abs/([0-9.]+)', it)
            if not aid or aid.group(1) in rows:
                continue
            t = re.search(r'<p class="title is-5 mathjax">(.*?)</p>', it, re.S)
            ab = re.search(r'<span class="abstract-full[^"]*"[^>]*>(.*?)<a', it, re.S)
            au = re.findall(r'searchtype=author[^>]*>(.*?)</a>', it)
            co = re.search(r'<span class="has-text-black-bis has-text-weight-semibold">Comments:</span>\s*<span[^>]*>(.*?)</span>', it, re.S)
            sub = re.search(r'Submitted</span>\s*(.*?);', it, re.S)
            rows[aid.group(1)] = dict(venue=venue, arxiv=aid.group(1), title=strip(t.group(1)) if t else '',
                                      authors=[strip(a) for a in au], abstract=strip(ab.group(1)) if ab else '',
                                      comments=strip(co.group(1)) if co else '', submitted=strip(sub.group(1)) if sub else '')
        total = re.search(r'of ([\d,]+) results', s)
        total = int(total.group(1).replace(',', '')) if total else 0
        start += 200
        print(q, start, '/', total, file=sys.stderr)
        if start >= total or not items:
            break
        time.sleep(3)  # be polite to arXiv
    time.sleep(3)
with open(out, 'w', encoding='utf-8') as f:
    for r in rows.values():
        f.write(json.dumps(r, ensure_ascii=False) + '\n')
print(len(rows), '->', out)
