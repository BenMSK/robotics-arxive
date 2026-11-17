# Robotics arXive

A phone-first reading list of robotics papers for robust SLAM, AI-applied motion planning and autonomous parking,
explained as short blog posts in English and Korean from the DYROS (SNU) point of view.

| What | Where |
|---|---|
| Live site | https://benmsk.github.io/robotics-arxive/ (GitHub Pages, branch `main`, root) |
| Repository | https://github.com/BenMSK/robotics-arxive |
| The app | `index.html` (one file; loads `papers.json`, then `papers/<id>/<lang>.md` on demand; Markdown via marked 12.0.2 from jsDelivr) |
| Entries | `papers/<id>/meta.json`, `en.md`, `ko.md`, `fig*.jpg` |
| Index the app reads (generated) | `papers.json`, built by `tools/build.py` |
| How to write an entry | `CONTRACT.md` (format, headings, Korean term rules, DYROS lens). `papers/seg-parking/` is the reference entry. |
| Not published (git-ignored) | `pdf/` (downloaded papers), `_raw/` (scraped programs, candidate pool, batches), `_shots/` |

Language, read marks and nothing else are kept in the browser (`localStorage`, keys `arxive.*`).

## Tabs

- **2026 papers**: ICRA, IROS, RSS and CoRL 2026 papers. `Write-up` entries have a full EN/KR post; `Abstract only`
  entries have no open full text (no arXiv), so they show the abstract, its Korean translation and an official link.
- **Bible**: classics in SLAM, planning and AI + robotics, with a "why it is a classic" line.
- **Venues**: the conferences and journals worth following, with typical months and deadlines.
- **Log**: dated notes on each update (`log.en.md`, `log.ko.md`: what was scanned, added and left out) followed by
  every paper grouped by its `added` date.

## Add a paper

1. Make `papers/<id>/` following `CONTRACT.md` (download the PDF into `pdf/`, read it with `python -P tools/fig.py text`,
   crop figures with `python -P tools/fig.py auto|clip`, write `meta.json`, `en.md`, `ko.md`).
2. `python -P tools/build.py` (checks every entry, writes `papers.json`; must print 0 errors).
3. Preview: `python -m http.server` in this folder, open http://localhost:8000.
4. Add a dated section to `log.en.md` and `log.ko.md` (what was added, anything left out).
5. Commit and push; Pages redeploys in about a minute.

## What is inside (2026-11-17)

71 entries: 39 full write-ups from 2026 venues (ICRA 24, IROS 8, RSS 5, CoRL 2), 15 ICRA 2026 abstract-only entries
(including the owner's T-ASE 2025 paper presented at ICRA 2026, marked "Mine"), and 17 bible papers
(SLAM 9, planning and AI + robotics 8).

## How the 2026 list was made (2026-11-17)

| Venue | Source | Papers scanned |
|---|---|---|
| ICRA 2026 | PaperCept program pages (titles, authors, keywords, abstracts): `tools/papercept.py` | 2,761 |
| RSS 2026 | roboticsconference.org paper pages; open PDFs on roboticsproceedings.org | 210 |
| CoRL 2026 | 2026.corl.org accepted-papers list (titles + OpenReview ids); abstracts from arXiv | 687 |
| IROS 2026 | The official program is behind a Cloudflare bot check, so: arXiv papers whose comments say "IROS 2026" and are not marked as submitted/under review (`tools/arxiv_search.py`) | 533 |

`tools/pool.py` merges these and tags candidates by keyword; the final selection was made by reading titles and
abstracts. `tools/resolve_arxiv.py` finds arXiv versions by title. OpenReview blocks automated access, so CoRL
papers without an arXiv version could not be read and were left out.
