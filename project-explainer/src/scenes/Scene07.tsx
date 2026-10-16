import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene07 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "周期预算与真实截止时间", "chapter": "02 · 调度与等待成本", "kind": "timeline", "nodes": ["lvgl：处理 + 5 ms", "sensor：10 ms 基准", "抢占 / 内部等待", "超期后重置基准"], "formula": "Tloop = 业务 + 抢占 + 内部等待 + 周期等待", "source": "application/app_task.c|#define APP_LVGL_TASK_PRIO", "lines": ["界面结束后再延时五毫秒，因此它的循环间隔不等于五毫秒。sensor 使用十毫秒周期基准，但同样可能被界面抢占或被内部操作拖长。", "诊断的处理耗时从循环开始量到业务结束，包含抢占和内部等待。它不是纯 CPU 运算时间，也不能直接拿来估算算法指令成本。", "EEPROM 保存可能让 sensor 错过周期。当前代码重置周期基准，避免连续补跑旧拍；这选择了恢复实时节奏，而不是补偿所有历史调用。", "所以应一起看处理预算、循环间隔和成功样本间隔。平均周期正常而最大成功间隔很大，仍可能让心率或积分路径遭遇断采样。"], "audio": "voice/neural-scene-07.wav", "seconds": 30.343833333333333, "beats": [0.35, 7.699, 14.813, 22.228], "id": 7, "evidence": {"scene": 7, "file": "application/app_task.c", "line": 36, "sha256": "6bc7ca1c0ed40133fedc11c121ef89683a806e02473548ce9ea525d8841cdacb", "excerpt": "#define APP_LVGL_TASK_PRIO   3    /* configMAX_PRIORITIES=5 */\n#define APP_LVGL_TASK_STK    1024 /* 字 = 4KB (Watch 工程实测足够跑 UI+刷新) */\n#define APP_LVGL_TASK_PERIOD 5    /* 保留原LVGL服务周期(ms) */\n\n#define APP_SENSOR_TASK_PRIO   2                        /* 低于 lvgl: UI 响应优先, 采样是后台生产 */\n#define APP_SENSOR_TASK_STK    512                      /* 字 = 2KB (IMU 读 + 零偏校准 + 气压读路径) */"}, "frames": 911, "startFrame": 5634, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "界面是处理完再延时五毫秒；", "startMs": 420, "endMs": 2574, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "sensor 是十毫秒周期基准。", "startMs": 2832, "endMs": 4855, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "它们对处理耗时的敏感程度不同。", "startMs": 5282, "endMs": 7579, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "诊断测的是从循环开始到业务结束的墙钟", "startMs": 7769, "endMs": 10877, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "时间，里面包含抢占和内部等待，并不只是", "startMs": 10828, "endMs": 14079, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "算法计算。", "startMs": 14041, "endMs": 14693, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "校准保存可能错过周期。", "startMs": 14883, "endMs": 16840, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "代码会重新设定基准，避免立刻补跑旧拍，", "startMs": 17278, "endMs": 20452, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "优先恢复正常节奏。", "startMs": 20666, "endMs": 22108, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "所以周期、处理耗时、成功读取间隔必须", "startMs": 22298, "endMs": 26086, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "一起看。", "startMs": 26037, "endMs": 26525, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "平均正常，也可能夹着一次很长的断采样。", "startMs": 26941, "endMs": 29874, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
