import asyncio
import hashlib
import json
import pathlib
import sys

project = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(project / '.tts-deps'))
import edge_tts

voice = 'zh-CN-YunxiNeural'
rate = '+14%'
cache = project / '.neural-cache'
cache.mkdir(exist_ok=True)


async def synthesize(text, stem, speech_rate=rate):
    key = hashlib.sha256((voice + speech_rate + text).encode()).hexdigest()
    audio_path = cache / f'{stem}.mp3'
    metadata_path = cache / f'{stem}.json'
    if audio_path.exists() and metadata_path.exists():
        previous = json.loads(metadata_path.read_text(encoding='utf-8'))
        if previous['key'] == key:
            return previous
    boundaries = []
    # Each paragraph remains one synthesis request, preserving contextual prosody.
    communicate = edge_tts.Communicate(text, voice, rate=speech_rate, boundary='WordBoundary')
    with audio_path.open('wb') as audio:
        async for event in communicate.stream():
            if event['type'] == 'audio':
                audio.write(event['data'])
            elif event['type'] == 'WordBoundary':
                boundaries.append({'text': event['text'], 'startMs': event['offset'] / 10000,
                                   'endMs': (event['offset'] + event['duration']) / 10000})
    assert audio_path.stat().st_size > 1000 and boundaries
    metadata = {'key': key, 'voice': voice, 'rate': speech_rate, 'text': text,
                'audio': str(audio_path), 'boundaries': boundaries}
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
    return metadata


async def main():
    if '--sample' in sys.argv:
        await synthesize('先看这条数据链路。传感器读得快，并不代表界面看到的数据就新。中间还有任务调度、快照发布，以及屏幕刷新的等待。我们要优化的，是从一次采样到真正显示出来的整段延迟。', 'sample', '+0%')
        print('Sample ready:', cache / 'sample.mp3', flush=True)
        return
    scenes = json.loads((project / 'content.json').read_text(encoding='utf-8'))
    speech = [block.splitlines() for block in (project / '口语旁白.txt').read_text(encoding='utf-8').strip().split('\n\n')]
    assert len(speech) == len(scenes) and all(len(lines) == 4 for lines in speech)
    semaphore = asyncio.Semaphore(3)
    completed = 0

    async def paragraph(index, beat, text):
        nonlocal completed
        async with semaphore:
            result = await synthesize(text, f'scene-{index + 1:02}-beat-{beat + 1}')
            completed += 1
            if completed % 8 == 0:
                print(f'Neural narration: {completed}/152 paragraphs', flush=True)
            return result

    tasks = [paragraph(index, beat, text) for index, lines in enumerate(speech) for beat, text in enumerate(lines)]
    results = await asyncio.gather(*tasks)
    (cache / 'manifest.json').write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding='utf-8')
    print('All neural narration ready.', flush=True)


asyncio.run(main())
