import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene36 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "数据时间与到达时间分开", "chapter": "10 · 诊断归因与优化闭环", "kind": "age", "nodes": ["成功读取时间", "样本序号", "队列丢弃 / TX 失败", "手机到达与解析"], "formula": "Adata = 消费时刻 − 采样标记时刻；跨设备时钟需另行对齐", "source": "application/app_heart.c|static app_heart_sample_struct heart_samples", "lines": ["调试波形保留同次成功读取的时间和序号，发送排队与手机显示可以晚于这个时刻。到达时间抖动不应直接被当成采样抖动。", "三十二帧队列满时丢最旧样本，优先保留近期状态。这适合观察实时波形，却不等于完整保存历史，每种丢弃都需要计数解释。", "板端时间差可以分析读取间隔，序号步长可以发现缺口，再对照队列丢弃与 UART 失败，缩小问题所在的链路范围。", "手机与板端时钟没有天然统一零点，因此不能直接相减得到精确单向传输延迟。需要时钟对齐或往返实验，才能进一步量化跨设备时延。"], "audio": "voice/neural-scene-36.wav", "seconds": 31.567041666666668, "beats": [0.35, 8.214, 15.383, 22.212], "id": 36, "evidence": {"scene": 36, "file": "application/app_heart.c", "line": 70, "sha256": "ee38b6fb8f98350ae3610814d13c4be0465819ff8be2e6babbe3c9ecfea2b096", "excerpt": "static app_heart_sample_struct heart_samples[APP_HEART_SAMPLE_COUNT];\nstatic uint16_t sample_head;\nstatic uint16_t sample_count;\nstatic uint16_t sample_sequence;\nstatic uint32_t sample_start_tick;\n"}, "frames": 948, "startFrame": 32977, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "波形记录同次成功读取的时间与序号，排队", "startMs": 420, "endMs": 3824, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "和显示可以晚一些。", "startMs": 3786, "endMs": 5195, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "到达抖动不能直接算作采样抖动。", "startMs": 5622, "endMs": 8094, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "三十二帧队列满时丢最旧，优先保留近期", "startMs": 8284, "endMs": 11776, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "状态，适合实时观察，但不等于完整历史", "startMs": 11727, "endMs": 14967, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "记录。", "startMs": 14918, "endMs": 15263, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "时间差看采样节奏，序号看缺口，再关联队", "startMs": 15453, "endMs": 18748, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "列丢弃和 UART 失败，才能逐步缩小", "startMs": 18699, "endMs": 21401, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "异常范围。", "startMs": 21363, "endMs": 22092, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "手机和板端时钟没有共同零点，不能直接", "startMs": 22282, "endMs": 25434, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "相减求单向延迟。", "startMs": 25385, "endMs": 27013, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "要继续量化，就得做时钟对齐或往返实验。", "startMs": 27451, "endMs": 31097, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
