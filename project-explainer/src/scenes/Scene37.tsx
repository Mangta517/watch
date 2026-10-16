import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene37 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "诊断分辨率与归因边界", "chapter": "10 · 诊断归因与优化闭环", "kind": "cost", "nodes": ["TIM2：100 μs 分辨率", "墙钟包含抢占", "CPU 归属包含 ISR", "未就绪不计通信失败"], "formula": "最大值定位长尾；分辨率与统计窗口决定能回答的问题", "source": "application/app_diag.c|TIM2->PSC = timer_hz / 10000U - 1U;", "lines": ["TIM2 一百微秒一计数，适合任务和驱动量级的比较，但不能据此精确测量几条寄存器指令或每个位操作的纳秒差异。", "任务处理耗时包含抢占和内部等待，运行统计的调度归属也包含中断时间。因此界面占比升高，不一定全部来自 LVGL 绘制本身。", "传感器成功、未就绪和错误分开统计，数据年龄补充最近成功时刻。只有读取耗时下降而成功间隔不变，可能说明瓶颈在调度而不是驱动。", "把周期最大值、读取最大值和成功间隔放在同一事件窗口看，才能形成因果假设。单个平均 CPU 百分比不足以证明实时性已经改善。"], "audio": "voice/neural-scene-37.wav", "seconds": 32.558708333333335, "beats": [0.35, 7.08, 14.682, 23.862], "id": 37, "evidence": {"scene": 37, "file": "application/app_diag.c", "line": 76, "sha256": "8d8f0e49cd7eba136d7b04cb1fa1c8a98e8e2c6ebd13a23baabacbe57fb524d0", "excerpt": "    TIM2->PSC = timer_hz / 10000U - 1U;\n    TIM2->ARR = UINT32_MAX;\n    TIM2->EGR = TIM_EGR_UG;\n    TIM2->SR = 0;\n    TIM2->CNT = 0;\n    TIM2->CR1 = TIM_CR1_CEN;"}, "frames": 977, "startFrame": 33925, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "诊断时钟每一百微秒计数，适合比较任务", "startMs": 420, "endMs": 3320, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "和读取耗时，不适合声称测出了几条指令", "startMs": 3282, "endMs": 6116, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "的微小差别。", "startMs": 6056, "endMs": 6960, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "墙钟耗时包含抢占，任务运行归属也包含", "startMs": 7150, "endMs": 10620, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "中断。", "startMs": 10571, "endMs": 10960, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "因此界面占比升高，不一定全部来自绘制。", "startMs": 11398, "endMs": 14562, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "成功、未就绪和错误分开，年龄补充最近", "startMs": 14752, "endMs": 18781, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "读到何时。", "startMs": 18732, "endMs": 19483, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "如果驱动变快、样本间隔却没变，就继续", "startMs": 19921, "endMs": 23215, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "查调度。", "startMs": 23166, "endMs": 23742, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "把周期、读取耗时和成功间隔放在同一窗口", "startMs": 23932, "endMs": 27654, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "关联。", "startMs": 27605, "endMs": 28049, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "单个平均 CPU 百分比，证明不了实时性", "startMs": 28487, "endMs": 31519, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "已经改善。", "startMs": 31480, "endMs": 32089, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
