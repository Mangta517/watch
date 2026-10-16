import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene15 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "dt 的语义会进入姿态误差", "chapter": "04 · 采样时间与融合约束", "kind": "age", "nodes": ["服务入口取 now", "读取前更新 s_last", "成功才执行融合", "dt 限幅 2–50 ms"], "formula": "服务间隔 ≠ 成功样本间隔 ≠ 器件转换间隔", "source": "application/app_imu.c|s_last = now;", "lines": ["当前 dt 在服务入口根据任务节拍计算，并且在读取之前更新上次时间。只有本轮 MPU 读取成功，才执行后面的融合。", "这意味着发生连续读取失败后，下一次成功融合使用的是最近服务间隔，而不是自动累计整个成功样本间隔，这一点可以直接从更新顺序确认。", "另外，二到五十毫秒的限幅能限制异常大步长，却也意味着超出上限的真实时间不会完整进入本次积分，稳定性与时间保真存在取舍。", "这里不据此修改算法或宣判故障，而是建立验证问题：在已知转动与缺采样条件下，比较角度误差、失败间隔与 dt，才能评估影响。"], "audio": "voice/neural-scene-15.wav", "seconds": 29.538541666666667, "beats": [0.35, 7.48, 14.353, 20.809], "id": 15, "evidence": {"scene": 15, "file": "application/app_imu.c", "line": 335, "sha256": "54e15580588e8ee0bc5e1f824416c93730e7bd6e24b4302446880b6939094fc6", "excerpt": "    s_last = now;\n\n    /* 1. 采样：同一sensor任务先取磁参考，再取MPU帧，避免总线竞争。 */\n    mag_online = mag_service(now, mag_body);\n    field_squared = mag_body[0] * mag_body[0] + mag_body[1] * mag_body[1] + mag_body[2] * mag_body[2];\n    /* 15 字节突发读同时释放 MPU INT 锁存，无 ISR 总线竞争。 */"}, "frames": 887, "startFrame": 13172, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "dt 在服务入口计算，上次时间也在读取", "startMs": 420, "endMs": 3528, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "前更新。", "startMs": 3468, "endMs": 4044, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "而融合只在本轮读取成功之后执行。", "startMs": 4482, "endMs": 7360, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "于是连续读失败后，下次成功融合用的还是", "startMs": 7550, "endMs": 10877, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "最近服务间隔，并不会自动累计整个断采样", "startMs": 10828, "endMs": 13816, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "时间。", "startMs": 13767, "endMs": 14233, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "二到五十毫秒限幅能控制异常步长，但超过", "startMs": 14423, "endMs": 17871, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "上限的真实时间，也不会完整进入这次积分。", "startMs": 17822, "endMs": 20689, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "这不直接等于算法有问题。", "startMs": 20879, "endMs": 22814, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "要用已知转动和缺采样实验，比较角度误差、", "startMs": 23241, "endMs": 26755, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "失败间隔与 dt 才能判断。", "startMs": 26936, "endMs": 29069, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
