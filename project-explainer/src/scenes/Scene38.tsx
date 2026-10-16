import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene38 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "把优化变成可复现的实验", "chapter": "10 · 诊断归因与优化闭环", "kind": "pipeline", "nodes": ["固定负载与基线", "一次改变一个成本", "验证数据与时序", "保留异常路径"], "formula": "优化闭环：假设 → 最小改动 → 对照 → 回归 → 实板证据", "source": "application/app_task.c|App_IMU_Service();", "lines": ["针对这个项目，先固定静置、切页、心率测量与校准保存四类负载，分别记录任务间隔、成功样本间隔、错误计数和内存峰值。", "每次只针对已确认的成本做最小修改，例如合并重复事务、减少持锁工作或替换忙等。保留优先级、采样参数和算法，方便归因。", "主机回归检查状态与故障语义，链接和构建检查资源边界，最后用板端统计与逻辑分析仪验证时序。各层证据回答不同问题。", "最终判断不只是更快，而是数据仍然有效、失败仍可诊断、长尾得到改善。这个闭环才是把现有手表工程继续做稳、做精的可迁移方法。"], "audio": "voice/neural-scene-38.wav", "seconds": 32.60870833333333, "beats": [0.35, 8.495, 16.414, 23.703], "id": 38, "evidence": {"scene": 38, "file": "application/app_task.c", "line": 192, "sha256": "6bc7ca1c0ed40133fedc11c121ef89683a806e02473548ce9ea525d8841cdacb", "excerpt": "        App_IMU_Service();\n        if (cal_finished == 0)\n        {\n            cal_state = my_mpu6050_zero_offset_state();\n            my_mpu6050_zero_offset_progress(&cal_stable, &cal_frames);\n            app_diag_boot_set(APP_BOOT_CAL_STATE, (uint32_t)cal_state);"}, "frames": 979, "startFrame": 34902, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "先固定静置、切页、心率和保存四类负载，", "startMs": 420, "endMs": 4033, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "建立周期、成功间隔、错误计数和内存峰值", "startMs": 4203, "endMs": 7958, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "的基线。", "startMs": 7898, "endMs": 8375, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "一次只改一个已确认的成本，比如重复事务、", "startMs": 8565, "endMs": 11815, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "持锁工作或忙等。", "startMs": 12019, "endMs": 13405, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "其他参数保留，收益才容易归因。", "startMs": 13844, "endMs": 16294, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "主机回归验证状态与故障语义，构建检查", "startMs": 16484, "endMs": 19801, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "资源边界，板端统计和逻辑分析仪验证实际", "startMs": 19752, "endMs": 23167, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "时序。", "startMs": 23118, "endMs": 23583, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "最终要同时满足：数据有效，失败可诊断，", "startMs": 23773, "endMs": 27408, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "长尾有改善。", "startMs": 27567, "endMs": 28581, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "这样留下的优化方法，才方便迁移到下个", "startMs": 29019, "endMs": 31766, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "工程。", "startMs": 31717, "endMs": 32139, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
