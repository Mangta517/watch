import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene18 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "磁参考是带年龄的约束", "chapter": "04 · 采样时间与融合约束", "kind": "quaternion", "nodes": ["75 Hz 输出 / 10 ms 轮询", "50 ms 最大年龄", "5–100 μT 总场门控", "水平场 > 5 μT"], "formula": "mₕ = m − (m · ĝ)ĝ；只沿重力轴施加航向反馈", "source": "common/common_imu.c|horizontal[i] = mag_uT[i] - dot * gravity[i];", "lines": ["磁力计输出七十五赫兹，任务每十毫秒轮询，两者不整齐同步。未就绪可以保留最近有效数据，但年龄超过五十毫秒就停止使用。", "总场模长还要通过五到一百微特斯拉门控，校准期间同样暂停磁融合。进入公共算法后，水平投影还要大于五微特斯拉。", "航向误差用正弦与余弦计算最短角差，再沿重力轴修正角速度，因此不会直接把完整磁倾角当作俯仰和横滚误差。", "调试时要区分无数据、过期、磁场越界和水平分量不足。只显示一个磁力计在线标志，无法说明航向当前到底有没有受到约束。"], "audio": "voice/neural-scene-18.wav", "seconds": 30.234166666666667, "beats": [0.35, 8.05, 14.346, 21.559], "id": 18, "evidence": {"scene": 18, "file": "common/common_imu.c", "line": 83, "sha256": "a34bc264c2fd69f9fb7ab4783acd3813634d4d1f56011951415d4b35d495ddda", "excerpt": "            horizontal[i] = mag_uT[i] - dot * gravity[i];\n        }\n        norm = sqrtf(horizontal[0] * horizontal[0] + horizontal[1] * horizontal[1] + horizontal[2] * horizontal[2]);\n        if (norm > 5.0f) /* 水平磁场大于5μT时才修正航向 */\n        {\n            for (i = 0; i < 3; ++i)"}, "frames": 908, "startFrame": 15949, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "磁力计七十五赫兹输出，十毫秒轮询，两", "startMs": 420, "endMs": 3813, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "者并不同步。", "startMs": 3753, "endMs": 4526, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "旧有效数据可以暂用，超过五十毫秒就停用。", "startMs": 4964, "endMs": 7930, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "总场还要通过五到一百微特斯拉门控。", "startMs": 8120, "endMs": 10592, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "进入融合后，水平分量必须大于五微特斯拉。", "startMs": 11020, "endMs": 14226, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "航向用最短角差修正，并且只沿重力轴施加", "startMs": 14416, "endMs": 17569, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "反馈，避免把完整磁倾角直接拉进俯仰和", "startMs": 17519, "endMs": 21011, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "横滚。", "startMs": 20962, "endMs": 21439, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "调试时分清无数据、过期、越界和水平场", "startMs": 21629, "endMs": 25329, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "过弱。", "startMs": 25280, "endMs": 25691, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "一个在线标志，说不清当前航向到底有没有", "startMs": 26119, "endMs": 29238, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "磁约束。", "startMs": 29200, "endMs": 29764, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
