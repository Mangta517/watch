import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene24 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "候选峰与有效心跳要分开", "chapter": "06 · 信号与候选状态", "kind": "wave", "nodes": ["candidate_tick", "beat_tick", "三次 RR 方差", "输出连续性 25%"], "formula": "短次峰不能覆盖有效心跳；高频假峰不能隔拍拼成正常心率", "source": "application/app_heart.c|static void heart_track_peak", "lines": ["当前检测器把相邻候选峰时间与最后接受心跳时间分开。短次峰可以被拒绝，同时保留有效心跳基准，避免污染下一次正常间隔。", "五路门限各自维护状态，满足三次稳定间隔后参与方差选优。相同方差时倾向更高门限，减少小次峰重复触发。", "候选周期还受上一有效输出的百分之二十五连续性约束。局部 RR 看似稳定，也不能毫无条件地替换当前输出。", "一个重要诊断细节是，导出的原始间隔字段不一定等于最终平滑周期。分析时不要要求它在每个状态下都与心率严格互为倒数。"], "audio": "voice/neural-scene-24.wav", "seconds": 31.21, "beats": [0.35, 8.149, 15.991, 23.138], "id": 24, "evidence": {"scene": 24, "file": "application/app_heart.c", "line": 109, "sha256": "ee38b6fb8f98350ae3610814d13c4be0465819ff8be2e6babbe3c9ecfea2b096", "excerpt": "static void heart_track_peak(heart_peak_struct *peak, float factor, uint32_t now)\n{\n    float threshold = peak->swing * factor;\n    float swing;\n    uint32_t interval;\n    uint32_t candidate_interval;"}, "frames": 937, "startFrame": 21631, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "候选峰时间和有效心跳时间分开。", "startMs": 420, "endMs": 2925, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "短次峰被拒绝时，原来的有效基准还在，", "startMs": 3363, "endMs": 6165, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "不会污染下一次正常间隔。", "startMs": 6445, "endMs": 8029, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "五路门限各自维护状态，用三次稳定间隔", "startMs": 8219, "endMs": 11415, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "的方差选优。", "startMs": 11355, "endMs": 12303, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "同分时倾向高门限，减少小次峰触发。", "startMs": 12719, "endMs": 15871, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "新候选还受前次输出百分之二十五连续性", "startMs": 16061, "endMs": 19082, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "限制，局部稳定，并不意味着能立刻替换", "startMs": 19033, "endMs": 22404, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "当前结果。", "startMs": 22366, "endMs": 23018, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "另外，导出的原始间隔不一定是最终平滑", "startMs": 23208, "endMs": 26492, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "周期。", "startMs": 26443, "endMs": 26886, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "别要求每个状态下，它都和心率严格互为倒数。", "startMs": 27325, "endMs": 30740, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
