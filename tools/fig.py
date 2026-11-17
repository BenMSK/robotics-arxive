"""PDF helper for writing a paper post.
  python -P tools/fig.py text PDF            body text up to the references
  python -P tools/fig.py find PDF            list figure captions: N, page, caption start
  python -P tools/fig.py auto PDF N OUT.jpg  crop Figure N (graphics above its caption)
  python -P tools/fig.py clip PDF PAGE x0 y0 x1 y1 OUT.jpg   crop a rect (PDF points, page 1-based)
  python -P tools/fig.py page PDF PAGE OUT.jpg               whole page preview (to pick a rect)
Always look at the output image before using it.
"""
import re, sys
import pymupdf

CAP = re.compile(r'^\s*(Fig\.|Figure|FIGURE|FIG\.)\s*(\d+)\s*[:.|]', re.S)


def save(page, rect, out, zoom=2.5):
    pix = page.get_pixmap(matrix=pymupdf.Matrix(zoom, zoom), clip=rect & page.rect)
    if pix.width > 1400:  # keep files small for phones
        pix = page.get_pixmap(matrix=pymupdf.Matrix(zoom * 1400 / pix.width, zoom * 1400 / pix.width), clip=rect & page.rect)
    pix.save(out, jpg_quality=85) if out.lower().endswith(('.jpg', '.jpeg')) else pix.save(out)
    print(out, pix.width, 'x', pix.height)


def captions(doc):
    for pno, page in enumerate(doc):
        for b in page.get_text('blocks'):
            m = CAP.match(b[4])
            if m:
                yield int(m.group(2)), pno, pymupdf.Rect(b[:4]), b[4].replace('\n', ' ')[:160]


def auto(doc, n, out):
    caps = list(captions(doc))
    hit = next((c for c in caps if c[0] == n), None)
    if not hit:
        sys.exit(f'Figure {n} caption not found; use page/clip')
    _, pno, cap, _ = hit
    page = doc[pno]
    W = page.rect.width
    if cap.x0 < W * 0.4 and cap.x1 > W * 0.6:
        x0, x1 = 0, W                                   # full-width figure
    elif cap.x1 <= W * 0.55:
        x0, x1 = 0, W / 2                               # left column
    else:
        x0, x1 = W / 2, W                               # right column
    # stop at the previous caption in the same column above this one
    top = max([c[2].y1 for c in caps if c[1] == pno and c[2].y1 <= cap.y0 and c[2].x0 < x1 and c[2].x1 > x0], default=0)
    rects = [pymupdf.Rect(d['rect']) for d in page.get_drawings()] + [pymupdf.Rect(i['bbox']) for i in page.get_image_info()]
    rects = [r for r in rects if r.y1 <= cap.y0 + 2 and r.y0 >= top - 2 and r.x0 >= x0 - 5 and r.x1 <= x1 + 5 and r.width * r.height > 4]
    if not rects:
        sys.exit(f'no graphics found above Figure {n} caption on page {pno + 1}; use page/clip')
    u = rects[0]
    for r in rects[1:]:
        u |= r
    u = pymupdf.Rect(u.x0 - 4, u.y0 - 4, u.x1 + 4, min(u.y1 + 4, cap.y0 - 2))
    print('page', pno + 1, 'rect', [round(v) for v in u])
    save(page, u, out)


if __name__ == '__main__':
    cmd, pdf = sys.argv[1], sys.argv[2]
    doc = pymupdf.open(pdf)
    sys.stdout.reconfigure(encoding='utf-8')
    if cmd == 'text':
        t = '\n'.join(p.get_text() for p in doc)
        m = re.search(r'\n\s*(References|REFERENCES|Bibliography)\s*\n', t)
        print(t[:m.start()] if m else t)
    elif cmd == 'find':
        for n, pno, r, txt in captions(doc):
            print(f'Fig {n} | page {pno + 1} | {txt}')
    elif cmd == 'auto':
        auto(doc, int(sys.argv[3]), sys.argv[4])
    elif cmd == 'clip':
        p, x0, y0, x1, y1 = map(float, sys.argv[3:8])
        save(doc[int(p) - 1], pymupdf.Rect(x0, y0, x1, y1), sys.argv[8])
    elif cmd == 'page':
        page = doc[int(sys.argv[3]) - 1]
        print('page size', page.rect)
        save(page, page.rect, sys.argv[4], zoom=1.2)
