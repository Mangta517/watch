import React from 'react';
import {AbsoluteFill, interpolate, useCurrentFrame, useVideoConfig, staticFile} from 'remotion';
import {Audio} from '@remotion/media';
import {Diagram} from './Visuals';

type SceneData = {
    id: number;
    title: string;
    chapter: string;
    kind: string;
    nodes: string[];
    formula: string;
    lines: string[];
    beats: number[];
    frames: number;
    seconds: number;
    startFrame: number;
    source: string;
    audio: string;
    playbackRate: number;
    evidence: {file: string; line: number; excerpt: string; scene: number; sha256: string};
};

const accents = ['#63e5d0', '#82baff', '#a8a0ff', '#ffbf77'];

export const SceneFrame = ({scene, children}: {scene: SceneData; children: React.ReactNode}) => {
    const frame = useCurrentFrame();
    const {fps} = useVideoConfig();
    let beat = 0;
    scene.beats.forEach((start, index) => {
        if (frame / fps >= start)
        {
            beat = index;
        }
    });
    const accent = accents[Math.floor((scene.id - 1) / 10)];
    const source_parts = scene.evidence.file.split('/');
    const source_name = source_parts[source_parts.length - 1];
    return (
        <AbsoluteFill data-scene={scene.id} style={{backgroundColor: '#08111d', color: '#eef5ff', fontFamily: 'Microsoft YaHei, Segoe UI, sans-serif', overflow: 'hidden'}}>
            <Audio src={staticFile(scene.audio)} playbackRate={scene.playbackRate} premountFor={fps} />
            <AbsoluteFill style={{backgroundImage: 'radial-gradient(ellipse at 10% 10%, #19344d 0%, transparent 58%), linear-gradient(135deg, #12253555, transparent 60%)'}} />
            <div style={{position: 'absolute', left: 0, right: 0, top: 0, height: 7, backgroundColor: '#263445'}}>
                <div style={{height: '100%', backgroundColor: accent, width: `${100 * frame / Math.max(1, scene.frames - 1)}%`}} />
            </div>
            <div style={{position: 'absolute', left: 100, top: 58, color: accent, fontSize: 28, letterSpacing: 4}}>{scene.chapter}</div>
            <div style={{position: 'absolute', right: 100, top: 58, fontSize: 26, color: '#8fa9bf', letterSpacing: 2}}>F411 / ENGINEERING NOTES</div>
            <div data-role="title" style={{position: 'absolute', left: 96, top: 110, fontSize: 82, fontWeight: 700, letterSpacing: -2,
                opacity: interpolate(frame, [0, 0.5 * fps], [0.75, 1], {extrapolateRight: 'clamp'}),
                translate: interpolate(frame, [0, 0.6 * fps], ['0px 18px', '0px 0px'], {extrapolateRight: 'clamp'})}}>{scene.title}</div>
            <div style={{position: 'absolute', left: 100, top: 225, height: 2, width: 1720, background: '#2b4257'}} />
            <div style={{position: 'absolute', left: 100, top: 265, width: 1100, height: 455, background: '#0d1c2b', border: '1px solid #294258', borderRadius: 24, overflow: 'hidden'}}>
                <Diagram kind={scene.kind} nodes={scene.nodes} beat={beat} accent={accent} sceneId={scene.id} />
            </div>
            <div data-role="explanation" style={{position: 'absolute', left: 1240, top: 270, width: 580, height: 450}}>
                <div style={{color: accent, fontSize: 26, letterSpacing: 3}}>关键步骤 · {String(beat + 1).padStart(2, '0')} / 04</div>
                <div style={{position: 'absolute', top: 52, fontSize: 37, lineHeight: 1.3, fontWeight: 650}}>{scene.nodes[beat]}</div>
                <div data-role="body" style={{position: 'absolute', top: 160, fontSize: 30, lineHeight: 1.55, color: '#c3d3e2'}}>{scene.lines[beat]}</div>
                <div data-role="source" style={{position: 'absolute', top: 418, width: '100%', borderTop: '1px solid #294258', paddingTop: 12, color: '#829eb7', fontSize: 21}}>源码锚点 · {source_name}:{scene.evidence.line}</div>
            </div>
            <div data-role="formula" style={{position: 'absolute', left: 100, right: 100, top: 755, padding: '18px 28px', borderLeft: `5px solid ${accent}`, backgroundColor: '#162c3e', fontFamily: 'Consolas, Microsoft YaHei, monospace', fontSize: 37, lineHeight: 1.45, color: '#e0edfa'}}>{scene.formula}</div>
            {children}
            <div style={{position: 'absolute', left: 100, bottom: 24, fontSize: 21, color: '#637e96'}}>源码讲解 · 动画为机制示意</div>
            <div style={{position: 'absolute', right: 100, bottom: 24, fontSize: 22, color: '#8fa9bf'}}>{String(scene.id).padStart(2, '0')} / 38</div>
        </AbsoluteFill>
    );
};
