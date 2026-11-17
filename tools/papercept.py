"""Parse PaperCept ContentListWeb pages (ICRA/IROS programs) into JSONL.
usage: python -I tools/papercept.py VENUE out.jsonl page1.html page2.html ...
"""
import html, json, re, sys

venue, out, pages = sys.argv[1], sys.argv[2], sys.argv[3:]
strip = lambda t: html.unescape(re.sub(r'<[^>]+>', '', t)).strip()
seen, rows = set(), []
for p in pages:
    s = open(p, encoding='cp1252', errors='replace').read()
    # each paper starts at a pHdr row; split on it
    for blk in s.split('<tr class="pHdr">')[1:]:
        t = re.search(r'class="pTtl">.*?<a [^>]*>(.*?)</a>', blk, re.S)
        a = re.search(r'<strong>Abstract:</strong>(.*?)</div>', blk, re.S)
        if not t or not a:
            continue
        title = strip(t.group(1))
        if title.lower() in seen:
            continue
        seen.add(title.lower())
        authors = [strip(x) for x in re.findall(r'AuthorIndexWeb\.html#\d+"[^>]*>(.*?)</a>', blk)]
        kw = [strip(x) for x in re.findall(r'KeywordIndexWeb\.html#[^"]*"[^>]*>(.*?)</a>', blk)]
        rows.append(dict(venue=venue, title=title, authors=authors, keywords=kw, abstract=strip(a.group(1))))
with open(out, 'w', encoding='utf-8') as f:
    for r in rows:
        f.write(json.dumps(r, ensure_ascii=False) + '\n')
print(len(rows), 'papers ->', out)
