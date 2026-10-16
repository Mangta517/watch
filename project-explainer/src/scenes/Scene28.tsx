import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene28 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "ADC 链路先分清误差来源", "chapter": "07 · 多速率链路与长尾", "kind": "cost", "nodes": ["480 周期采样", "12.5 MHz ADC", "2 ms 有界等待", "3300 mV 参考假设"], "formula": "约 38.4 μs 采样窗口；电压偏差不一定来自软件时序", "source": "application/app_battery.c|sample.voltage_mv =", "lines": ["电池 ADC 的采样窗口配置为四百八十周期，在当前十二点五兆赫 ADC 时钟下约三十八点四微秒，服务又设了两毫秒转换超时。", "长采样窗口和有限等待解决不同问题：前者影响采样建立条件，后者限制故障时的任务停留时间，不能把它们混成同一个超时参数。", "电压换算使用三千三百毫伏参考假设及二倍分压恢复。整数舍入已经处理，但参考、电阻和负载误差仍会进入最终电压。", "如果读数偏差稳定，先对照引脚电压和参考条件；如果成功间隔异常，再看调度与转换结果。这样能避免拿滤波参数掩盖测量链路误差。"], "audio": "voice/neural-scene-28.wav", "seconds": 31.27575, "beats": [0.35, 7.628, 14.912, 22.634], "id": 28, "evidence": {"scene": 28, "file": "application/app_battery.c", "line": 64, "sha256": "5bc898e5e2caf055b9c812feddeef3754b252587ae7a9641c3383de4400f180f", "excerpt": "        sample.voltage_mv = (raw * APP_BATTERY_VREF_MV * 2U + 2047U) / 4095U;\n        sample.valid = 1;\n    }\n    if (sample.result == HAL_OK)\n    {\n        app_diag_sensor_record(APP_DIAG_BATTERY, APP_DIAG_OK, (uint32_t)sample.result, read_started);"}, "frames": 939, "startFrame": 25315, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "ADC 的四百八十周期采样，在当前时", "startMs": 420, "endMs": 3112, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "钟下约三十八点四微秒；", "startMs": 3063, "endMs": 4844, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "服务另外设置了两毫秒转换超时。", "startMs": 5102, "endMs": 7508, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "前者给输入建立留时间，后者限制故障等待。", "startMs": 7698, "endMs": 10840, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "这是两个不同问题，别把它们当成同一个", "startMs": 11278, "endMs": 14057, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "超时参数。", "startMs": 14019, "endMs": 14792, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "电压换算假设参考为三点三伏，再恢复二倍", "startMs": 14982, "endMs": 18518, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "分压。", "startMs": 18458, "endMs": 18901, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "舍入处理了，参考、电阻和负载误差还在。", "startMs": 19329, "endMs": 22514, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "稳定偏差先比引脚电压与参考条件；", "startMs": 22704, "endMs": 25538, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "读取间隔异常再看调度。", "startMs": 25818, "endMs": 27676, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "别用滤波参数掩盖测量链路误差。", "startMs": 28114, "endMs": 30806, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
