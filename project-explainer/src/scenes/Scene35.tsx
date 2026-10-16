import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene35 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "观察系统也会改变系统", "chapter": "10 · 诊断归因与优化闭环", "kind": "register", "nodes": ["编译开关 = 0", "采样与统计保留", "快照 / 打包 / TX 停止", "调试版做开关对照"], "formula": "性能观察要控制诊断开销，不能只增加日志", "source": "application/app_diag.h|#define APP_DEBUG_TELEMETRY 0", "lines": ["当前编译开关为零，停止诊断、心率波形和发送测试输出，计时统计与本地测量仍保留。手机开启命令不能绕过这个编译选择。", "启用调试版后，即使 UART 已异步化，快照复制、CRC 打包和发送中断仍会消耗资源。异步不是零成本，只是改变成本分布。", "比较开关状态时，应使用同样操作、相近负载和独立统计窗口；历史最大值不会自动清除，不能把关闭后的历史峰值当作新测量。", "这个对照能判断问题是否被观察手段放大。先减掉不必要输出，再确认真实采样间隔，通常比继续增加每帧打印更可靠。"], "audio": "voice/neural-scene-35.wav", "seconds": 31.236833333333333, "beats": [0.35, 8.127, 16.101, 23.68], "id": 35, "evidence": {"scene": 35, "file": "application/app_diag.h", "line": 8, "sha256": "2616ef9df16501561c1884213d934ffb05e2e69a9c478fcee95dd8d5475a125b", "excerpt": "#define APP_DEBUG_TELEMETRY 0\n#endif\n\nenum\n{\n    APP_DIAG_LVGL,"}, "frames": 938, "startFrame": 32039, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "当前编译开关关闭调试外发，统计和本地", "startMs": 420, "endMs": 3967, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "测量仍运行。", "startMs": 3918, "endMs": 4932, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "手机的开启命令，不能绕过这个开关。", "startMs": 5370, "endMs": 8007, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "即使发送已异步，快照、CRC 和发送", "startMs": 8197, "endMs": 11360, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "中断也要消耗资源。", "startMs": 11311, "endMs": 12719, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "异步改变的是成本分布，不会让成本消失。", "startMs": 13158, "endMs": 15981, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "对照开关时，用同样操作和独立窗口。", "startMs": 16171, "endMs": 19158, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "历史最大值不会自动清零，不能把旧峰值", "startMs": 19597, "endMs": 22628, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "当成新结果。", "startMs": 22667, "endMs": 23560, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "先确认观察手段有没有放大问题。", "startMs": 23750, "endMs": 26080, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "继续增加每帧打印，可能让采样更差，反而", "startMs": 26518, "endMs": 29725, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "离真实原因更远。", "startMs": 29676, "endMs": 30767, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
