import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene16 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "校准判定必须可解释", "chapter": "04 · 采样时间与融合约束", "kind": "register", "nodes": ["Δa < 0.0183 g", "水平容差 0.15 g", "100 稳定 + 100 平均", "阶段超时 / 拒绝位图"], "formula": "“校准失败”应拆成读失败、姿态不符、变化过大或超时", "source": "interface/mpu6050.c|static uint8_t mpu6050_cal_still", "lines": ["当前校准静止判定检查三轴加速度变化，以及水平放置时相对重力基准的偏差。变化门限约零点零一八三倍重力，水平容差零点一五倍。", "诊断保存每个轴的变化、拒绝原因位图、累计拒绝和最大连续稳定帧数，因此可以区分放置不平、振动和读取失败。", "状态机用真实时间限制等待与平均阶段，不允许通过极快重复调用凑足帧数。成功前只积累候选，完成后才提交完整偏移。", "这比反复猜测延时参数更有价值。先看是哪一条判据持续拒绝，再核对量程、安装方向与噪声，不把所有失败都归因于任务没跑够快。"], "audio": "voice/neural-scene-16.wav", "seconds": 31.156541666666666, "beats": [0.35, 6.894, 14.83, 23.199], "id": 16, "evidence": {"scene": 16, "file": "interface/mpu6050.c", "line": 224, "sha256": "818321a2793bdc80d2ec4bc527545ea910477e36d8b471c0d88754a4c2a3d836", "excerpt": "static uint8_t mpu6050_cal_still(const Accel_struct *current)\n{\n    uint32_t reason = 0;\n    s_cal.diag[MPU_CAL_DIAG_AX] = (uint32_t)(int32_t)current->Accel_x;\n    s_cal.diag[MPU_CAL_DIAG_AY] = (uint32_t)(int32_t)current->Accel_y;\n    s_cal.diag[MPU_CAL_DIAG_AZ] = (uint32_t)(int32_t)current->Accel_z;"}, "frames": 935, "startFrame": 14059, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "校准检查加速度变化和水平偏差，门限分别", "startMs": 420, "endMs": 3934, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "约零点零一八三倍重力，以及零点一五倍", "startMs": 3885, "endMs": 6445, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "重力。", "startMs": 6396, "endMs": 6774, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "诊断还记录每轴变化、拒绝位图和最大连续", "startMs": 6964, "endMs": 11158, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "稳定帧，因此能区分放不平、振动和读失败。", "startMs": 11109, "endMs": 14710, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "阶段超时按真实时间算，不能快速重复调用", "startMs": 14900, "endMs": 18798, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "凑帧。", "startMs": 18749, "endMs": 19204, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "成功前积累候选，完成后才提交完整偏移。", "startMs": 19642, "endMs": 23079, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "遇到失败，先查哪条判据持续拒绝，再核对", "startMs": 23269, "endMs": 26926, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "量程、方向和噪声，比反复猜延时参数有效。", "startMs": 26887, "endMs": 30687, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
