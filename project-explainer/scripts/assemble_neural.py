import array
import json
import io
import pathlib
import subprocess
import wave

project = pathlib.Path(__file__).resolve().parents[1]
content = json.loads((project / 'content.json').read_text(encoding='utf-8'))
manifest = json.loads((project / '.neural-cache' / 'manifest.json').read_text(encoding='utf-8'))
speech = [block.splitlines() for block in (project / '口语旁白.txt').read_text(encoding='utf-8').strip().split('\n\n')]
assert len(manifest) == len(content) * 4
ffmpeg = project / 'node_modules' / '@remotion' / 'compositor-win32-x64-msvc' / 'ffmpeg.exe'
sample_rate = 24000
scenes = []

for scene_index, scene in enumerate(content):
    samples = array.array('h', [0] * int(sample_rate * 0.35))
    captions = []
    beats = []
    for beat, paragraph in enumerate(speech[scene_index]):
        caption_begin = len(captions)
        record = manifest[scene_index * 4 + beat]
        assert record['text'] == paragraph
        decoded = subprocess.run([str(ffmpeg), '-v', 'error', '-i', record['audio'], '-f', 'wav', '-acodec', 'pcm_s16le',
                                  '-ar', str(sample_rate), '-ac', '1', '-'], capture_output=True, check=True)
        with wave.open(io.BytesIO(decoded.stdout)) as decoded_wave:
            raw = array.array('h', decoded_wave.readframes(decoded_wave.getnframes()))
        boundaries = record['boundaries']
        first_ms = max(0, boundaries[0]['startMs'] - 70)
        last_ms = min(len(raw) / sample_rate * 1000, boundaries[-1]['endMs'] + 100)
        offset_ms = len(samples) / sample_rate * 1000 - first_ms
        beats.append(len(samples) / sample_rate)
        samples.extend(raw[round(first_ms / 1000 * sample_rate):round(last_ms / 1000 * sample_rate)])
        cursor = 0
        caption_text = ''
        caption_start = boundaries[0]['startMs']
        for index, boundary in enumerate(boundaries):
            word = boundary['text']
            location = paragraph.find(word, cursor)
            if location < 0:
                location = cursor
            next_cursor = min(len(paragraph), location + len(word))
            while next_cursor < len(paragraph) and paragraph[next_cursor] in '，。；！？：、 ':
                next_cursor += 1
            caption_text += paragraph[cursor:next_cursor]
            cursor = next_cursor
            if len(caption_text) >= 18 or caption_text.endswith(('。', '；', '！', '？')) or index == len(boundaries) - 1:
                if index == len(boundaries) - 1:
                    caption_text += paragraph[cursor:]
                captions.append({'text': caption_text, 'startMs': round(offset_ms + caption_start),
                                 'endMs': round(offset_ms + boundary['endMs'] + 60),
                                 'timestampMs': None, 'confidence': None, 'pageBreakAfter': True})
                caption_text = ''
                if index + 1 < len(boundaries):
                    caption_start = boundaries[index + 1]['startMs']
        samples.extend([0] * int(sample_rate * 0.08))
        assert ''.join(caption['text'] for caption in captions[caption_begin:]) == paragraph
    samples.extend([0] * int(sample_rate * 0.35))
    audio_name = f'voice/neural-scene-{scene_index + 1:02}.wav'
    with wave.open(str(project / 'public' / audio_name), 'wb') as output:
        output.setnchannels(1)
        output.setsampwidth(2)
        output.setframerate(sample_rate)
        output.writeframes(samples.tobytes())
    scenes.append({'audio': audio_name, 'seconds': len(samples) / sample_rate, 'beats': beats, 'captions': captions})
(project / '.neural-cache' / 'timeline.json').write_text(json.dumps(scenes, ensure_ascii=False, indent=2), encoding='utf-8')
print(f'Neural audio assembled: {sum(scene["seconds"] for scene in scenes) / 60:.2f} minutes. No playback acceleration.')
