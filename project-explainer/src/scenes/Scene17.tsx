import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene17 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "融合门控与反馈状态", "chapter": "04 · 采样时间与融合约束", "kind": "quaternion", "nodes": ["0.75–1.25 g 门控", "比例 0.8 / 积分 0.03", "修正角速度再积分", "归一化与 asin 限幅"], "formula": "门控当前误差，不等于清除已累计的积分反馈", "source": "common/common_imu.c|uint8_t Common_IMU_GetEulerAngleWithMag", "lines": ["加速度模长只有在零点七五到一点二五倍重力之间才参与方向纠偏，用于排除明显线性加速度，但这不是对所有运动干扰的完整分类。", "门控关闭时，本轮加速度误差为零，之前积累的积分反馈仍然存在。理解这个状态延续，才能解释冲击结束后姿态恢复的过程。", "代码把比例、积分和磁航向反馈加入角速度，再统一递推四元数，最后归一化。欧拉角输出还对反正弦输入做范围限制。", "分析收敛应分别观察门控占空比、积分状态与输出误差。单纯提高反馈增益，可能只是把动态误差更快灌进姿态，而非提高估计质量。"], "audio": "voice/neural-scene-17.wav", "seconds": 31.813041666666667, "beats": [0.35, 7.464, 14.616, 23.763], "id": 17, "evidence": {"scene": 17, "file": "common/common_imu.c", "line": 24, "sha256": "a34bc264c2fd69f9fb7ab4783acd3813634d4d1f56011951415d4b35d495ddda", "excerpt": "uint8_t Common_IMU_GetEulerAngleWithMag(const Accel_Gyro_struct *raw, const float mag_uT[3], Euler_struct *euler, float dt)\n{\n    /* 三轴数组统一按机体X/Y/Z排列；四元数q0为标量，q1/q2/q3为向量。 */\n    float q0 = s_q.q0, q1 = s_q.q1, q2 = s_q.q2, q3 = s_q.q3;\n    float gravity[3];     /* 四元数预测的单位重力向量 */\n    float accel[3];       /* 加速度，先为LSB，再归一化 */"}, "frames": 955, "startFrame": 14994, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "加速度模长落在零点七五到一点二五倍重力", "startMs": 420, "endMs": 3177, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "之间才纠偏，这能排除明显冲击，但不能", "startMs": 3128, "endMs": 6193, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "识别所有运动干扰。", "startMs": 6144, "endMs": 7344, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "门控关闭，本轮误差变成零，之前累计的", "startMs": 7534, "endMs": 10883, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "积分反馈却还在。", "startMs": 10834, "endMs": 11892, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "它会影响冲击之后的恢复过程。", "startMs": 12320, "endMs": 14496, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "比例、积分和磁反馈先加入角速度，再统一", "startMs": 14686, "endMs": 18627, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "递推四元数。", "startMs": 18578, "endMs": 19461, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "之后归一化，输出角度时再限制反正弦输入。", "startMs": 19899, "endMs": 23643, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "看收敛，应该同时看门控、积分状态和角度", "startMs": 23833, "endMs": 27446, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "误差。", "startMs": 27397, "endMs": 27851, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "提高增益，也可能只是更快把动态干扰灌", "startMs": 28290, "endMs": 31047, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "进去。", "startMs": 30987, "endMs": 31343, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
