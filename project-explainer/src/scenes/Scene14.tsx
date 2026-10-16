import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene14 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "读成功不一定是新样本", "chapter": "04 · 采样时间与融合约束", "kind": "register", "nodes": ["INT_STATUS 同次读回", "锁存释放", "总线成功返回 0", "DATA_RDY 未作为返回门控"], "formula": "读取成功率、数据更新率与中断边沿计数不能互相替代", "source": "interface/mpu6050.c|mpu6050_int_status = packet[0];", "lines": ["当前 MPU 读取函数把中断状态保存下来，但只要十五字节总线事务成功，就返回成功，没有用 DATA READY 位再限制这个返回值。", "因此诊断里的读取成功，首先证明总线事务完成，不自动证明拿到了新的内部转换结果。一百赫兹服务也不等于严格对齐一百赫兹采样。", "中断采用锁存，直到任务读取状态才释放。边沿计数反映这条锁存与释放链路，不能直接当作所有内部转换次数。", "评估数据新鲜度时，应联合看状态位、读取间隔和任务时序。动画展示的是现有语义，不会把轮询架构包装成精确的数据就绪驱动采集。"], "audio": "voice/neural-scene-14.wav", "seconds": 33.200833333333335, "beats": [0.35, 7.782, 16.261, 24.958], "id": 14, "evidence": {"scene": 14, "file": "interface/mpu6050.c", "line": 493, "sha256": "818321a2793bdc80d2ec4bc527545ea910477e36d8b471c0d88754a4c2a3d836", "excerpt": "    mpu6050_int_status = packet[0];\n    data->Accel.Accel_x = (int16_t)(((buf[0] << 8) | buf[1]) - offset_accel_x);\n    data->Accel.Accel_y = (int16_t)(((buf[2] << 8) | buf[3]) - offset_accel_y);\n    data->Accel.Accel_z = (int16_t)(((buf[4] << 8) | buf[5]) - offset_accel_z);\n    data->Gyro.Gyro_x = (int16_t)(((buf[8] << 8) | buf[9]) - offset_gyro_x);\n    data->Gyro.Gyro_y = (int16_t)(((buf[10] << 8) | buf[11]) - offset_gyro_y);"}, "frames": 997, "startFrame": 12175, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "MPU 会保存读回的中断状态，但只要", "startMs": 420, "endMs": 3188, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "十五字节事务成功就返回成功，没有再用", "startMs": 3150, "endMs": 6094, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "数据就绪位限制返回。", "startMs": 6056, "endMs": 7662, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "所以读取成功先证明通信完成，不保证每次", "startMs": 7852, "endMs": 11541, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "拿到新转换。", "startMs": 11492, "endMs": 12364, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "一百赫兹服务，也不等于与器件输出精确", "startMs": 12791, "endMs": 15713, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "对齐。", "startMs": 15664, "endMs": 16140, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "数据就绪采用锁存，任务读状态后才释放。", "startMs": 16330, "endMs": 19833, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "因此中断边沿数，也不能直接当成所有内部", "startMs": 20261, "endMs": 24158, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "转换次数。", "startMs": 24120, "endMs": 24838, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "要判断新鲜度，就把状态位、读取间隔和", "startMs": 25028, "endMs": 28454, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "任务时序放在一起看，别把周期轮询当成", "startMs": 28405, "endMs": 31766, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "精确触发采样。", "startMs": 31728, "endMs": 32731, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
