"""Check every papers/<id>/ entry against CONTRACT.md and write papers.json (the app's index).
usage: python -P tools/build.py
"""
import glob, json, os, re, sys

TAGS = set('''RobustSLAM LiDAR VisualSLAM VIO Radar Thermal WheelOdometry GNSS Degeneracy Outliers Certifiable
Uncertainty DynamicSLAM PlaceRecognition FactorGraph GaussianSplatting FoundationModel MotionPlanning SamplingBased
LearnedSampling Diffusion FlowMatching ReinforcementLearning ImitationLearning InverseRL EndToEndDriving
AutonomousDriving Parking Prediction POMDP GPU Safety Benchmark Dataset Survey MultiRobot VLA WorldModel Nonholonomic
TrajectoryOptimization Manipulation'''.split())
HEAD = {'en': ['Why this paper', 'The problem', 'Key idea', 'How it works', 'Results', 'Limitations', 'DYROS lens', 'Key terms'],
        'ko': ['왜 이 논문인가', '문제', '핵심 아이디어', '작동 방식', '결과', '한계', 'DYROS 관점', '용어 정리']}

errors, warns, out = [], [], []
for mp in sorted(glob.glob('papers/*/meta.json')):
    d = os.path.dirname(mp)
    pid = os.path.basename(d)
    err = lambda m: errors.append(f'{pid}: {m}')
    try:
        m = json.load(open(mp, encoding='utf-8'))
    except Exception as e:
        err(f'bad json: {e}'); continue
    if m.get('id') != pid: err('id must equal folder name')
    for k in ['title', 'venue', 'abstract']:
        if not str(m.get(k, '')).strip(): err(f'missing {k}')
    if not m.get('authors'): err('missing authors')
    if not isinstance(m.get('year'), int): err('year must be an int')
    if m.get('track') not in ('2026', 'bible'): err('track must be "2026" or "bible"')
    if m.get('kind') not in ('full', 'abstract'): err('kind must be "full" or "abstract"')
    for lang in ('en', 'ko'):
        if not m.get('tldr', {}).get(lang, '').strip(): err(f'missing tldr.{lang}')
    links = m.get('links', {})
    if not any(links.get(k) for k in ('arxiv', 'pdf', 'official')): err('needs at least one of links.arxiv/pdf/official')
    tags = m.get('tags', [])
    if not 2 <= len(tags) <= 7: err('3-6 tags')
    warns += [f'{pid}: new tag {t}' for t in tags if t not in TAGS]
    if m.get('kind') == 'full':
        for lang in ('en', 'ko'):
            p = f'{d}/{lang}.md'
            if not os.path.exists(p):
                err(f'missing {lang}.md'); continue
            s = open(p, encoding='utf-8').read()
            if not s.lstrip().startswith('> **TL;DR**'): err(f'{lang}.md must start with "> **TL;DR**"')
            got = re.findall(r'^## (.+?)\s*$', s, re.M)
            if got != HEAD[lang]: err(f'{lang}.md headings {got} != {HEAD[lang]}')
            for img in re.findall(r'!\[[^\]]*\]\(([^)]+)\)', s):
                if not os.path.exists(f'{d}/{img}'): err(f'{lang}.md uses missing image {img}')
            m.setdefault('words', {})[lang] = len(s.split()) if lang == 'en' else len(s.replace(' ', ''))
    elif m.get('kind') == 'abstract' and not m.get('abstract_ko', '').strip():
        err('abstract entry needs abstract_ko')
    out.append(m)

# newest first: within 2026, CoRL (Nov) > IROS (Oct) > RSS (Jul) > ICRA (Jun); own papers on top, write-ups before abstracts
RANK = {'CoRL 2026': 0, 'IROS 2026': 1, 'RSS 2026': 2, 'ICRA 2026': 3}
out.sort(key=lambda m: (m.get('track') != '2026', not m.get('mine'), -m.get('year', 0), RANK.get(m.get('venue'), 9),
                        m.get('kind') == 'abstract', m.get('title', '').lower()))
json.dump(out, open('papers.json', 'w', encoding='utf-8'), ensure_ascii=False, separators=(',', ':'))
sys.stdout.reconfigure(encoding='utf-8')
print('\n'.join(['WARN ' + w for w in warns] + ['ERROR ' + e for e in errors]))
print(f'{len(out)} entries ({sum(m.get("kind") == "full" for m in out)} full) -> papers.json, {len(errors)} errors')
sys.exit(1 if errors else 0)
