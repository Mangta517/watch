import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene01 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "先建立工程的性能模型", "chapter": "01 · 所有权与性能模型", "kind": "watch", "nodes": ["数据年龄", "执行占用", "所有权转移", "证据闭环"], "formula": "关注端到端延迟、长尾与一致性，而不只看平均 CPU", "source": "application/app_task.c|void App_Task_Create", "lines": ["这次直接从你现有工程的约束展开：两个应用任务、共享处理器、软件总线、异步刷屏，以及跨任务的数据发布。", "我们追踪三个问题：界面看到的数据有多旧，哪个等待拖慢了采样，以及并发保护到底覆盖了哪一次读改写。", "重点不是给模块贴优化标签，而是把每项改动映射到具体成本：减少总线事务、缩短持锁时间，或者把忙等变成可调度等待。", "所有波形与时间轴都是机制示意。现有实现、可推导的上限和必须上板测量的结果，会明确分开，避免把推算当性能成绩。"], "audio": "voice/neural-scene-01.wav", "seconds": 28.156958333333332, "beats": [0.35, 6.938, 12.725, 20.315], "id": 1, "evidence": {"scene": 1, "file": "application/app_task.c", "line": 54, "sha256": "6bc7ca1c0ed40133fedc11c121ef89683a806e02473548ce9ea525d8841cdacb", "excerpt": "void App_Task_Create(void)\n{\n    if (xTaskCreate(app_lvgl_task, \"lvgl\", APP_LVGL_TASK_STK, NULL, APP_LVGL_TASK_PRIO, NULL) != pdPASS)\n    {\n        Error_Handler(); /* FreeRTOS 堆不足 */\n    }"}, "frames": 845, "startFrame": 0, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "先抓住这个工程的约束：两个任务，共用", "startMs": 420, "endMs": 3133, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "一个处理器；", "startMs": 3084, "endMs": 3802, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "采样走软件总线，刷屏则交给 DMA。", "startMs": 4038, "endMs": 6818, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "接下来围绕三个问题看代码：数据有多旧，", "startMs": 7008, "endMs": 9875, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "采样在哪里被拖慢，临界区又保护了什么。", "startMs": 10045, "endMs": 12605, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "每个优化都对应一笔成本。", "startMs": 12795, "endMs": 14730, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "它可能减少总线事务，也可能缩短持锁时间，", "startMs": 15168, "endMs": 18452, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "或者让等待不再占着处理器。", "startMs": 18622, "endMs": 20195, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "图里的波形和时序用来解释机制。", "startMs": 20385, "endMs": 22923, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "哪些结论能从代码推导，哪些还得上板测，", "startMs": 23351, "endMs": 26338, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "我们会逐项说明。", "startMs": 26574, "endMs": 27687, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
