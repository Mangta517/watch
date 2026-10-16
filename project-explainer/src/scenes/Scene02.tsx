import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene02 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "先划清执行权与数据所有权", "chapter": "01 · 所有权与性能模型", "kind": "pipeline", "nodes": ["sensor 独占器件", "LVGL 独占界面", "ISR 只完成交接", "快照 / 命令跨任务"], "formula": "所有权图，比模块目录更能解释并发行为", "source": "application/app_display.c|void App_Display_Init", "lines": ["这里真正有约束力的分层是执行上下文。sensor 独占传感器事务和融合状态，LVGL 任务负责界面与蓝牙业务推进。", "中断承担接收字节、传输完成通知或事件记录。它不替代业务任务完成一次复杂的设备操作，避免同一资源出现第二个操作者。", "跨任务边界传递的是快照或请求，不是让消费者直接进入生产者的驱动。例如校准请求先被取走，再由 sensor 执行。", "分析新增功能时，先标出状态的写入者、读取者和清除者。这样才能判断需要互斥、短临界区，还是沿用单一所有者就足够。"], "audio": "voice/neural-scene-02.wav", "seconds": 30.809166666666666, "beats": [0.35, 8.083, 15.903, 23.329], "id": 2, "evidence": {"scene": 2, "file": "application/app_display.c", "line": 28, "sha256": "8b5155f560ef2dda9dd700070c8a08e333315382282ea7ac3c859fa76eaf8eab", "excerpt": "void App_Display_Init(void)\n{\n    /* mutex 在任务上下文创建: 此时调度器已运行, 临界区计数才真正归零工作\n     * (CM4F port 的 uxCriticalNesting 以 0xaaaaaaaa 哨兵初值, 调度器前的\n     *  enter/exit 不解除 BASEPRI, 属设计行为, 见 SVC 启动时统一清零) */\n    s_lvgl_mux = xSemaphoreCreateMutex();"}, "frames": 925, "startFrame": 845, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "先按执行上下文划分职责。", "startMs": 420, "endMs": 2794, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "sensor 管器件和融合状态，界面", "startMs": 3221, "endMs": 5759, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "任务负责显示，以及蓝牙业务的推进。", "startMs": 5710, "endMs": 7963, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "中断只做字节接收、完成通知和事件记录。", "startMs": 8153, "endMs": 11601, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "复杂操作留给任务，同一个资源就不会多", "startMs": 12039, "endMs": 15005, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "出一个操作者。", "startMs": 14945, "endMs": 15783, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "跨任务传的是快照或者请求。", "startMs": 15973, "endMs": 18380, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "比如重新校准，界面先提交请求，再由 ", "startMs": 18807, "endMs": 21915, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "sensor 真正执行。", "startMs": 21877, "endMs": 23209, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "加功能前先问清楚：谁写状态，谁读状态，", "startMs": 23399, "endMs": 26968, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "谁把它清掉。", "startMs": 27105, "endMs": 27977, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "保护方式由这个关系决定。", "startMs": 28404, "endMs": 30339, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
