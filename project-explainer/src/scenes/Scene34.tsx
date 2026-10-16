import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene34 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "共享接收入口，独立消息事务", "chapter": "09 · 通信背压与交接", "kind": "age", "nodes": ["累积消息边界", "字段与范围检查", "完整消息才提交", "保留最后有效状态"], "formula": "输入到达 ≠ 消息完整 ≠ 应用状态已经更新", "source": "application/app_weather.c|WEATHER,", "lines": ["时间和天气复用同一 UART 接收入口，但各自维护消息解析与提交条件。一次读取不是消息边界，不能按缓冲长度直接更新业务状态。", "天气必须完整解析并通过范围检查才替换有效状态，时间消息验证后再设置 RTC。非法片段不能先写一半再等待后续字段补齐。", "这与传感器快照采用相似原则：先在局部构建候选结果，确认整体有效，再发布。这样把不完整输入与稳定业务状态隔离。", "手机天气是外部模型数据，AHT 是板载测量。表盘中的新旧标记反映数据来源和年龄，不能为了视觉一致把两者混成同一种实时环境值。"], "audio": "voice/neural-scene-34.wav", "seconds": 32.163916666666665, "beats": [0.35, 7.721, 16.189, 23.511], "id": 34, "evidence": {"scene": 34, "file": "application/app_weather.c", "line": 45, "sha256": "069e2f4edbec4c42d9d93f0509e5ce10e73a8b771d0e6da9e45d205e9f2e9ccf", "excerpt": "    if (!s_overflow && s_length >= 8 && memcmp(s_frame, \"WEATHER,\", 8) == 0 && parse_number(&cursor, &code) && *cursor++ == ',' &&\n        parse_number(&cursor, &temperature) && *cursor++ == ',' && parse_number(&cursor, &humidity) && cursor == s_frame + s_length &&\n        *cursor == '\\0' && code >= 0 && code <= 99 && temperature >= -1000 && temperature <= 1000 && humidity >= 0 && humidity <= 100)\n    {\n        s_weather.code = (uint8_t)code;\n        s_weather.temperature_x10 = (int16_t)temperature;"}, "frames": 965, "startFrame": 31074, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "时间和天气共用接收入口，各自管理消息", "startMs": 420, "endMs": 3430, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "边界。", "startMs": 3380, "endMs": 3791, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "一次读取不是一条消息，不能直接据此更新", "startMs": 4230, "endMs": 7228, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "状态。", "startMs": 7179, "endMs": 7601, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "字段完整且范围合法，才提交天气；", "startMs": 7791, "endMs": 10581, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "时间也先验证再写 RTC。", "startMs": 10894, "endMs": 13290, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "非法片段不能先覆盖半份状态。", "startMs": 13728, "endMs": 16069, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "这与快照发布思路一样：先构建候选，确认", "startMs": 16259, "endMs": 19838, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "整体有效后再提交，把不完整输入挡在业务", "startMs": 19800, "endMs": 22930, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "状态外。", "startMs": 22881, "endMs": 23391, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "手机天气与板载 AHT 来源不同。", "startMs": 23581, "endMs": 26086, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "显示新旧状态时保留来源和年龄，别为了", "startMs": 26513, "endMs": 29753, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "统一界面混淆两种含义。", "startMs": 29715, "endMs": 31694, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
