import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene30 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "DMA 完成也是所有权归还", "chapter": "08 · 显示吞吐与生命周期", "kind": "dma", "nodes": ["buffer A 在途", "CPU 使用 buffer B", "BSY 尾部等待", "flush_ready / notify"], "formula": "异步收益依赖缓冲生命周期，而不只是调用 DMA API", "source": "Middlewares/LVGL/lvgl/examples/porting/lv_port_disp_template.c|static void lcd_flush_ready_cb(void)", "lines": ["启动 DMA 后，源缓冲仍归传输过程使用，不能提前覆盖。完成回调通知 LVGL 归还缓冲，双缓冲才能安全交替。", "当前移植层在等待时使用任务通知，DMA 完成可以唤醒界面任务，让等待窗口里的 CPU 留给 sensor，而不是持续轮询刷新标志。", "但回调链仍有等待 SPI 忙标志清除的循环，用于末尾移位完成。正常只应很短，却仍属于中断内执行成本，不能从时延模型里删除。", "检查这条路径时，同时追踪启动、在途、完成和缓冲复用四个时刻。异步调用返回，只说明传输已提交，既不是显示完成，也不是数据可以立刻改写。"], "audio": "voice/neural-scene-30.wav", "seconds": 33.48458333333333, "beats": [0.35, 8.028, 16.178, 24.031], "id": 30, "evidence": {"scene": 30, "file": "Middlewares/LVGL/lvgl/examples/porting/lv_port_disp_template.c", "line": 25, "sha256": "3dd75613cdf81e3faca6e9aa4a634b606a41ed8bb93146f8c1c28ec663fb1d1c", "excerpt": "static void lcd_flush_ready_cb(void);\nstatic void disp_wait(lv_disp_drv_t *disp_drv);\n\n/**********************\n *  STATIC VARIABLES\n **********************/"}, "frames": 1005, "startFrame": 27229, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "DMA 启动后，缓冲仍归发送过程使用，", "startMs": 420, "endMs": 3309, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "不能覆盖。", "startMs": 3424, "endMs": 4087, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "完成回调归还所有权，双缓冲才可以交替。", "startMs": 4526, "endMs": 7908, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "当前等待采用任务通知，DMA 完成就", "startMs": 8098, "endMs": 11031, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "唤醒界面。", "startMs": 10982, "endMs": 11667, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "等待窗口里的 CPU 可以交给 sensor，", "startMs": 12105, "endMs": 14709, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "而非持续轮询。", "startMs": 14846, "endMs": 16058, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "回调还会等 SPI 忙标志清除，确保", "startMs": 16248, "endMs": 19356, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "末位移出。", "startMs": 19318, "endMs": 19981, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "正常很短，但它仍然是中断成本，不能忽略。", "startMs": 20419, "endMs": 23911, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "跟踪启动、在途、完成和复用四个时刻。", "startMs": 24101, "endMs": 27429, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "异步返回只是提交完成，不表示已经显示，", "startMs": 27867, "endMs": 31194, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "更不表示缓冲能改。", "startMs": 31354, "endMs": 33015, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
