import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene27 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "异步测量是请求到完成的链路", "chapter": "07 · 多速率链路与长尾", "kind": "age", "nodes": ["触发成功", "pending 等转换", "80 ms 后检查", "200 ms 截止 / CRC"], "formula": "请求成功、设备就绪、有效样本发布是三个事件", "source": "application/app_aht21.c|if (s_pending != 0)", "lines": ["AHT 服务把触发和取结果拆开，等待转换期间返回主循环。这里的 pending 代表存在未完成测量，而不是已经拥有一份有效温湿度。", "八十毫秒后检查结果，忙状态可以继续等待到两百毫秒截止。未就绪、超时、总线错误和 CRC 错误需要保留各自含义。", "这类状态机应记录请求时间与结果发布时间，才能计算端到端延迟。只测最后那次读取，会遗漏内部转换等待和调度推迟。", "当前方案让转换等待与其他采样重叠，是比单纯减少运算指令更直接的收益。但最终成功间隔仍受同一个 sensor 任务的队头阻塞影响。"], "audio": "voice/neural-scene-27.wav", "seconds": 31.445083333333333, "beats": [0.35, 8.609, 16.857, 23.79], "id": 27, "evidence": {"scene": 27, "file": "application/app_aht21.c", "line": 49, "sha256": "483e583bd958ea46df4d5a866c8099b2f9f5ce81188ca776348740b4713ecdda", "excerpt": "    if (s_pending != 0)\n    {\n        if (elapsed < AHT21_CONVERSION_MS)\n        {\n            return;\n        }"}, "frames": 944, "startFrame": 24371, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "AHT 把触发和取结果拆开，转换时返回", "startMs": 420, "endMs": 3945, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "主循环。", "startMs": 3896, "endMs": 4482, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "pending 表示测量没结束，不代表", "startMs": 4909, "endMs": 7437, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "已经有了有效数据。", "startMs": 7398, "endMs": 8489, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "八十毫秒后检查，忙就继续等，到两百毫秒", "startMs": 8679, "endMs": 11864, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "截止。", "startMs": 11815, "endMs": 12226, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "未就绪、超时、通信错误和 CRC 错误", "startMs": 12664, "endMs": 16123, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "分别处理。", "startMs": 16085, "endMs": 16737, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "端到端延迟要从触发算到发布。", "startMs": 16927, "endMs": 19279, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "只测最后的读取，会漏掉内部转换等待和", "startMs": 19706, "endMs": 22858, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "调度推迟。", "startMs": 22809, "endMs": 23670, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "收益来自转换等待与其他采样重叠。", "startMs": 23860, "endMs": 26782, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "但最终成功间隔，仍然受同一个 sensor ", "startMs": 27220, "endMs": 29857, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "任务的长操作影响。", "startMs": 29808, "endMs": 30975, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
