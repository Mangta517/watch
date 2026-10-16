import hashlib
import json
import pathlib
import re

project = pathlib.Path(__file__).resolve().parents[1]
repo = project.parent
content = json.loads((project / 'content.json').read_text(encoding='utf-8'))
chunks = []
evidence = []
for scene_index, scene in enumerate(content):
    source_path, needle = scene['source'].split('|', 1)
    source = (repo / source_path).read_text(encoding='utf-8-sig')
    assert needle in source, (scene['title'], needle)
    source_lines = source.splitlines()
    line_index = next(i for i, line in enumerate(source_lines) if needle in line)
    evidence.append({'scene': scene_index + 1, 'file': source_path, 'line': line_index + 1,
                     'sha256': hashlib.sha256((repo / source_path).read_bytes()).hexdigest(),
                     'excerpt': '\n'.join(source_lines[line_index:line_index + 6])})
    for beat, paragraph in enumerate(scene['lines']):
        clauses = re.findall(r'[^，。；！？]+[，。；！？]?', paragraph)
        for clause in clauses:
            while len(clause) > 28:
                cut = clause.rfind('、', 0, 27) + 1
                if cut < 12:
                    cut = 26
                chunks.append({'id': len(chunks), 'scene': scene_index, 'beat': beat, 'text': clause[:cut]})
                clause = clause[cut:]
            if clause:
                chunks.append({'id': len(chunks), 'scene': scene_index, 'beat': beat, 'text': clause})
(project / 'public' / 'voice').mkdir(parents=True, exist_ok=True)
(project / '.voice-cache').mkdir(exist_ok=True)
(project / 'qa').mkdir(exist_ok=True)
(project / 'tts-input.json').write_text(json.dumps(chunks, ensure_ascii=False, indent=2), encoding='utf-8')
(project / 'evidence.json').write_text(json.dumps(evidence, ensure_ascii=False, indent=2), encoding='utf-8')
reference = (project / 'basic-captions-reference.md').read_text(encoding='utf-8')
basic = re.search(r'```tsx[^\n]*\n(.*?)\n```', reference, re.S).group(1)
(project / 'src' / 'basic-captions.tsx').write_text(basic, encoding='utf-8')
print(f'Prepared {len(content)} scenes, {len(chunks)} speech clips; all source anchors verified.')
