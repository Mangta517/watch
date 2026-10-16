import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene23 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "滤波器时间常数与重建策略", "chapter": "06 · 信号与候选状态", "kind": "wave", "nodes": ["dc：1500 ms", "低通：70 ms", "幅值：300 ms", "预热 / 缺采样 / 阶跃"], "formula": "α = Δt / (τ + Δt)；异常段重建状态，不继续盲目追踪", "source": "application/app_heart.c|detector.dc += (float)elapsed", "lines": ["这里的三个一阶滤波使用实际经过时间计算系数，分别控制慢背景、交流低通和幅值估计，避免把调用次数直接当作时间。", "接触阶跃的门限同时考虑绝对计数和原始值比例。首帧或重建阶段跟随当前背景，避免让慢速基线花很久追赶一个错误初值。", "长缺采样会重新建立滤波与周期；原始样本仍可用于诊断，但重新建立的状态不应立即被解释为有效心率。", "评价滤波参数需要同时观察噪声抑制、峰位置延迟和恢复时间。曲线看起来更平滑，不自动代表峰间隔更准确，也可能只是把瞬态隐藏了。"], "audio": "voice/neural-scene-23.wav", "seconds": 33.074041666666666, "beats": [0.35, 8.889, 17.247, 24.706], "id": 23, "evidence": {"scene": 23, "file": "application/app_heart.c", "line": 275, "sha256": "ee38b6fb8f98350ae3610814d13c4be0465819ff8be2e6babbe3c9ecfea2b096", "excerpt": "    detector.dc += (float)elapsed / (1500.0f + (float)elapsed) * ((float)raw - detector.dc);\n    detector.filtered += (float)elapsed / (70.0f + (float)elapsed) * ((float)raw - detector.dc - detector.filtered);\n    detector.amplitude += (float)elapsed / (300.0f + (float)elapsed) * (fabsf(detector.filtered) - detector.amplitude);\n    data->signal_x10 = (uint32_t)(detector.amplitude * 10.0f + 0.5f);\n    for (i = 0; i < APP_HEART_GATE_COUNT; ++i)\n    {"}, "frames": 993, "startFrame": 20638, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "背景、交流低通和幅值估计，用实际间隔", "startMs": 420, "endMs": 3945, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "算滤波系数，时间常数分别是一千五百、", "startMs": 3907, "endMs": 7103, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "七十和三百毫秒。", "startMs": 7240, "endMs": 8769, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "接触阶跃同时检查绝对变化和相对比例。", "startMs": 8959, "endMs": 12386, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "重建时先跟随背景，避免慢滤波器长时间", "startMs": 12824, "endMs": 16064, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "追赶错误初值。", "startMs": 16015, "endMs": 17127, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "长缺采样也会重建滤波和周期。", "startMs": 17317, "endMs": 19844, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "原始值可以继续诊断，但新状态不能立刻", "startMs": 20283, "endMs": 23555, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "被当成有效心率。", "startMs": 23506, "endMs": 24586, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "滤波效果要同时看降噪、峰位延迟和恢复", "startMs": 24776, "endMs": 28334, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "时间。", "startMs": 28285, "endMs": 28761, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "曲线更平滑，也可能只是把有用的瞬态磨掉", "startMs": 29200, "endMs": 32461, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "了。", "startMs": 32401, "endMs": 32604, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
