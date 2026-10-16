import array
import json
import pathlib
import re
import sys
import wave

project = pathlib.Path(__file__).resolve().parents[1]
content = json.loads((project / 'content.json').read_text(encoding='utf-8'))
chunks = json.loads((project / 'tts-input.json').read_text(encoding='utf-8'))
evidence = json.loads((project / 'evidence.json').read_text(encoding='utf-8'))
fps = 30
sample_rate = 22050
scenes = []
total_frames = 0
neural_mode = '--neural' in sys.argv
if neural_mode:
    neural_scenes = json.loads((project / '.neural-cache' / 'timeline.json').read_text(encoding='utf-8'))
for i, scene in enumerate(content):
    if neural_mode:
        scenes.append({**scene, **neural_scenes[i], 'id': i + 1, 'evidence': evidence[i]})
        continue
    samples = array.array('h', [0] * int(sample_rate * 0.45))
    captions = []
    beats = []
    for chunk in [c for c in chunks if c['scene'] == i]:
        with wave.open(str(project / '.voice-cache' / f"clip-{chunk['id']:04}.wav")) as audio:
            assert audio.getnchannels() == 1 and audio.getsampwidth() == 2 and audio.getframerate() == sample_rate
            raw = array.array('h', audio.readframes(audio.getnframes()))
        active = [j for j, value in enumerate(raw) if abs(value) > 100]
        assert active, chunk
        left = max(0, active[0] - int(0.035 * sample_rate))
        right = min(len(raw), active[-1] + int(0.065 * sample_rate))
        start = len(samples) / sample_rate
        if len(beats) <= chunk['beat']:
            beats.append(start)
        samples.extend(raw[left:right])
        end = len(samples) / sample_rate
        captions.append({'text': chunk['text'], 'startMs': round(start * 1000), 'endMs': round(end * 1000),
                         'timestampMs': None, 'confidence': None, 'pageBreakAfter': True})
        samples.extend([0] * int(sample_rate * 0.08))
    samples.extend([0] * int(sample_rate * 0.6))
    seconds = len(samples) / sample_rate
    with wave.open(str(project / 'public' / 'voice' / f'scene-{i + 1:02}.wav'), 'wb') as audio:
        audio.setnchannels(1)
        audio.setsampwidth(2)
        audio.setframerate(sample_rate)
        audio.writeframes(samples.tobytes())
    scenes.append({**scene, 'id': i + 1, 'seconds': seconds, 'captions': captions, 'beats': beats,
                   'audio': f'voice/scene-{i + 1:02}.wav', 'evidence': evidence[i]})

natural_seconds = sum(s['seconds'] for s in scenes)
# Keep the complete narration within the user's 15–20 minute choice.
playback_rate = max(1.0, natural_seconds / (19 * 60))
if neural_mode:
    playback_rate = 1.0
assert playback_rate <= 1.35, f'Voice too fast: {playback_rate}; regenerate with a higher SAPI rate.'
for scene in scenes:
    scene['seconds'] /= playback_rate
    scene['frames'] = int(scene['seconds'] * fps + 0.99999)
    scene['startFrame'] = total_frames
    scene['playbackRate'] = playback_rate
    scene['beats'] = [round(t / playback_rate, 3) for t in scene['beats']]
    for caption in scene['captions']:
        caption['startMs'] = round(caption['startMs'] / playback_rate)
        caption['endMs'] = round(caption['endMs'] / playback_rate)
    total_frames += scene['frames']
assert 15 * 60 * fps <= total_frames <= 20 * 60 * fps

(project / 'src' / 'scenes').mkdir(exist_ok=True)
for scene in scenes:
    index = scene['id']
    data = {k: v for k, v in scene.items() if k != 'captions'}
    code = f'''import {{SceneFrame}} from '../SceneFrame';
import {{BasicCaptions}} from '../basic-captions';
import {{useVideoConfig}} from 'remotion';

export const Scene{index:02} = () => {{
    const {{fps}} = useVideoConfig();
    return (
        <SceneFrame scene={{{json.dumps(data, ensure_ascii=False)}}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={{fps}}
                width={{2125}}
                combineTokensWithinMilliseconds={{1}}
                style={{{{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}}}
                captions={{{json.dumps(scene['captions'], ensure_ascii=False)}}}
            />
        </SceneFrame>
    );
}};
'''
    (project / 'src' / 'scenes' / f'Scene{index:02}.tsx').write_text(code, encoding='utf-8')

imports = '\n'.join(f"import {{Scene{s['id']:02}}} from './scenes/Scene{s['id']:02}';" for s in scenes)
sequence_nodes = '\n'.join(f'''            <Series.Sequence name="{s['id']:02} · {s['title']}" durationInFrames={{{s['frames']}}} premountFor={{fps}}>
                <Scene{s['id']:02} />
            </Series.Sequence>''' for s in scenes)
registrations = '\n'.join(f'''                <Composition id="Chapter-{s['id']:02}" component={{Scene{s['id']:02}}} durationInFrames={{{s['frames']}}} fps={{30}} width={{1920}} height={{1080}} />''' for s in scenes)
root = f'''import {{Composition, Folder, Series, useVideoConfig}} from 'remotion';
{imports}

export const FullVideo = () => {{
    const {{fps}} = useVideoConfig();
    return (
        <Series>
{sequence_nodes}
        </Series>
    );
}};

export const RemotionRoot = () => (
    <>
        <Composition id="F411-Deep-Dive" component={{FullVideo}} durationInFrames={{{total_frames}}} fps={{30}} width={{1920}} height={{1080}} />
        <Folder name="Chapters">
{registrations}
        </Folder>
    </>
);
'''
(project / 'src' / 'Root.tsx').write_text(root, encoding='utf-8')
(project / 'timeline.json').write_text(json.dumps(scenes, ensure_ascii=False, indent=2), encoding='utf-8')

def stamp(seconds):
    milliseconds = round(seconds * 1000)
    hours, milliseconds = divmod(milliseconds, 3600000)
    minutes, milliseconds = divmod(milliseconds, 60000)
    secs, milliseconds = divmod(milliseconds, 1000)
    return f'{hours:02}:{minutes:02}:{secs:02},{milliseconds:03}'

srt = []
voice_description = '语音：本机 Microsoft Huihui Desktop；字幕按每个合成语音片段的真实音频长度对齐。'
if neural_mode:
    voice_description = '语音：Microsoft 云希神经男声；整段合成，按词级时间戳对齐字幕，播放速度 1 倍。'
script = ['# STM32F411 智能手表 · 数据链路、性能与并发进阶', '',
          '38 段中文合成配音动画。波形、运动与调度图为机制示意，不是实机测量。', '',
          voice_description, '',
          f'总时长：{total_frames / fps / 60:.2f} 分钟；1920×1080；30 fps。', '']
for scene in scenes:
    offset = scene['startFrame'] / fps
    chapter_time = f'{int(offset // 60):02}:{int(offset % 60):02}'
    script.extend([f"## {chapter_time} · {scene['id']:02} {scene['title']}", '',
                   f"源码：`{scene['evidence']['file']}:{scene['evidence']['line']}`", '',
                   f"关键关系：{scene['formula']}", ''])
    script.extend('\n\n'.join(scene['lines']).splitlines())
    script.append('')
    for caption in scene['captions']:
        srt.extend([str(len(srt) // 4 + 1),
                    f"{stamp(offset + caption['startMs'] / 1000)} --> {stamp(offset + caption['endMs'] / 1000)}",
                    caption['text'], ''])
script.extend(['## 外部基础资料', '',
               '- [ST 官方 STM32F411CE 产品资料](https://www.st.com/en/microcontrollers-microprocessors/stm32f411ce.html)',
               '- [ST RM0383 参考手册](https://www.st.com/resource/en/reference_manual/dm00119316-stm32f411xce-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)',
               '- [Remotion Composition](https://www.remotion.dev/docs/composition)', '',
               '当前项目具体配置以 evidence.json 对应的源码快照为准；旧文档中的 BLE-only 描述未作为当前行为。'])
(project / '讲解稿.md').write_text('\n'.join(script), encoding='utf-8')
(project / '中文字幕.srt').write_text('\n'.join(srt), encoding='utf-8')
print(f'Natural audio {natural_seconds:.1f}s, playback {playback_rate:.3f}x, total {total_frames} frames / {total_frames / fps / 60:.2f} min')
