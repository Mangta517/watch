import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene25 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "单任务多速率的队头阻塞", "chapter": "07 · 多速率链路与长尾", "kind": "cost", "nodes": ["IMU → HEART → AHT", "ADC / BARO 分频", "保存发生在同一任务", "让出 CPU 不等于采样继续"], "formula": "任务数没增加，但低频操作仍可能拖慢高频服务", "source": "application/app_task.c|#define APP_SENSOR_BARO_DIV", "lines": ["共用 sensor 任务节省了栈与调度开销，也给总线建立明确所有权。但服务按顺序执行，前面的慢操作会推迟后面的高频服务。", "例如校准保存位于 IMU 服务之后、心率服务之前。即使 EEPROM 等待期间通过延时让出 CPU，同一个 sensor 任务仍然不能继续读心率。", "低频并不等于低影响。一秒一次的长操作，对平均 CPU 影响很小，却可能制造一次足以触发心率重建的长间隔。", "所以优化优先级应由长尾读取间隔决定，而不是只按平均耗时排序。先定位哪次保存、超时或页面刷新与异常间隔同时出现，再选择最小修改。"], "audio": "voice/neural-scene-25.wav", "seconds": 31.001708333333333, "beats": [0.35, 7.776, 16.32, 23.275], "id": 25, "evidence": {"scene": 25, "file": "application/app_task.c", "line": 43, "sha256": "6bc7ca1c0ed40133fedc11c121ef89683a806e02473548ce9ea525d8841cdacb", "excerpt": "#define APP_SENSOR_BARO_DIV    25                       /* 气压250ms轮询，器件1Hz */\n\n/**********************\n *  STATIC PROTOTYPES\n **********************/\nstatic void app_lvgl_task(void *pvParameters);"}, "frames": 931, "startFrame": 22568, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "共用任务省栈，也统一了总线所有权。", "startMs": 420, "endMs": 3199, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "但所有服务按顺序执行，前面的慢操作会", "startMs": 3638, "endMs": 7009, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "推迟后面。", "startMs": 6960, "endMs": 7656, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "校准保存就在 IMU 之后、心率之前。", "startMs": 7846, "endMs": 10822, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "即使等待 EEPROM 时让出了 CPU，", "startMs": 11261, "endMs": 14073, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "sensor 本身仍不能继续采样。", "startMs": 14221, "endMs": 16200, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "低频操作也可能造成长尾。", "startMs": 16390, "endMs": 18314, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "一秒才运行一次，对平均 CPU 影响", "startMs": 18752, "endMs": 21301, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "不大，却可能触发心率重建。", "startMs": 21252, "endMs": 23155, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "因此先按最大成功间隔排优化优先级，再", "startMs": 23345, "endMs": 26749, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "关联保存、超时和刷新事件，别只按平均", "startMs": 26689, "endMs": 29863, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "耗时排序。", "startMs": 29814, "endMs": 30532, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
