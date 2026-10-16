import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene06 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "状态机优化的是等待占用", "chapter": "02 · 调度与等待成本", "kind": "timeline", "nodes": ["begin 发起", "service 推进", "pending 保留", "ready / timeout 收敛"], "formula": "长等待拆阶段；每次 service 的最坏耗时仍要检查", "source": "application/app_task.c|watch_rtc_init_begin();", "lines": ["RTC 晶振与蓝牙上电等待现在由 begin 和 service 共同完成。界面任务每轮推进有限阶段，传感器能够在等待期间独立运行。", "这项优化消除的是整段同步等待对业务循环的占用，不意味着每个 service 内部都只有常数级执行时间。", "需要继续查看一次调用里会发多少串口字节，做多少寄存器访问，以及失败路径会不会立即重试。状态枚举本身不是非阻塞的证明。", "验收时同时比较首样本时间、界面循环最大间隔和初始化结果。只看到开机更快，可能遗漏某个阶段超时后被提前放弃的情况。"], "audio": "voice/neural-scene-06.wav", "seconds": 33.254333333333335, "beats": [0.35, 8.083, 16.583, 24.207], "id": 6, "evidence": {"scene": 6, "file": "application/app_task.c", "line": 93, "sha256": "6bc7ca1c0ed40133fedc11c121ef89683a806e02473548ce9ea525d8841cdacb", "excerpt": "    watch_rtc_init_begin();\n    app_diag_boot_set(APP_BOOT_BT_BEGIN_MS, HAL_GetTick());\n    kt6368a_init_begin();\n\n    for (;;)\n    {"}, "frames": 998, "startFrame": 4636, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "RTC 和蓝牙把启动拆成开始与推进两", "startMs": 420, "endMs": 3265, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "个阶段。", "startMs": 3205, "endMs": 3682, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "等待硬件时，界面循环还能继续，sensor ", "startMs": 4120, "endMs": 7316, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "也能工作。", "startMs": 7344, "endMs": 7963, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "不过，写成状态机不等于每次调用都很短。", "startMs": 8153, "endMs": 11513, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "一次推进里发多少字节、读多少寄存器，", "startMs": 11941, "endMs": 15235, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "仍然要检查。", "startMs": 15482, "endMs": 16463, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "尤其看失败路径：会不会立刻重试，会不会", "startMs": 16653, "endMs": 19838, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "一次等待很久。", "startMs": 19800, "endMs": 20704, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "枚举状态再多，也不能证明没有阻塞。", "startMs": 21143, "endMs": 24087, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "验收一起看首样本、界面最大间隔和初始化", "startMs": 24277, "endMs": 28240, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "结果。", "startMs": 28191, "endMs": 28558, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "不能把某个模块超时退出，误当成启动优化", "startMs": 28996, "endMs": 32390, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "成功。", "startMs": 32341, "endMs": 32784, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
