import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene04 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "启动优化看关键路径", "chapter": "02 · 调度与等待成本", "kind": "timeline", "nodes": ["PA3 保持供电", "首帧提交", "sensor 初始化", "正常采样开始"], "formula": "优化首帧、首样本、可交互时间，是三个不同目标", "source": "application/app_task.c|app_diag_boot_set(APP_BOOT_FIRST_FRAME_MS", "lines": ["启动链路先维持电源，再准备外设。对于这个项目，首帧提交、传感器初始化完成和正常采样开始，是三个应该单独记录的事件。", "当前显示初始化包含阻塞路径，所以把任务提前创建并不自动等于传感器已经开始工作，要沿实际调用顺序找第一个让出点。", "MPU 初始化也存在器件应答重试和复位状态轮询。前者通过延时让出 CPU，后者在循环中连续进行寄存器事务，成本性质不同。", "有效的优化是缩短关键路径或把独立等待重叠，而不是只移动函数位置。启动诊断应比较这些事件的时间差，并保留失败结果。"], "audio": "voice/neural-scene-04.wav", "seconds": 33.34204166666667, "beats": [0.35, 8.204, 16.485, 24.875], "id": 4, "evidence": {"scene": 4, "file": "application/app_task.c", "line": 84, "sha256": "6bc7ca1c0ed40133fedc11c121ef89683a806e02473548ce9ea525d8841cdacb", "excerpt": "    app_diag_boot_set(APP_BOOT_FIRST_FRAME_MS, HAL_GetTick());\n\n    if (xTaskCreate(app_sensor_task, \"sensor\", APP_SENSOR_TASK_STK, NULL, APP_SENSOR_TASK_PRIO, NULL) != pdPASS)\n    {\n        Error_Handler(); /* FreeRTOS 堆不足 */\n    }"}, "frames": 1001, "startFrame": 2699, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "启动先分三个时间点：首帧提交，传感器", "startMs": 420, "endMs": 3846, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "初始化完成，还有正常采样开始。", "startMs": 3808, "endMs": 6171, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "三个指标要分别看。", "startMs": 6609, "endMs": 8083, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "显示初始化里还有同步等待。", "startMs": 8274, "endMs": 10636, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "所以任务创建得早，不等于采样就开始得", "startMs": 11074, "endMs": 13963, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "早，还要找它何时真正获得执行。", "startMs": 13914, "endMs": 16365, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "MPU 初始化里，应答重试会延时让出 ", "startMs": 16555, "endMs": 19630, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "CPU；", "startMs": 19658, "endMs": 20233, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "复位状态轮询却连续访问寄存器。", "startMs": 20491, "endMs": 22897, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "两种等待成本不同。", "startMs": 23336, "endMs": 24755, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "优化启动，要缩短关键路径，或者重叠独立", "startMs": 24945, "endMs": 28525, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "等待。", "startMs": 28476, "endMs": 28898, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "只移动函数位置，未必改变首个有效样本", "startMs": 29336, "endMs": 32401, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "的时间。", "startMs": 32341, "endMs": 32872, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
