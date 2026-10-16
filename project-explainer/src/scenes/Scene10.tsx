import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene10 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "快照一致，不等于同时采样", "chapter": "03 · 数据一致性与临界区", "kind": "ownership", "nodes": ["事务与计算在锁外", "整组状态锁内发布", "消费者锁内复制", "格式化与发送在锁外"], "formula": "一致性解决“半更新”；时间戳解决“有多旧”", "source": "application/app_imu.c|uint8_t App_IMU_GetSnapshot", "lines": ["IMU 在锁外读取和融合，最后把原始值、姿态与磁状态放进同一个保护区发布，消费者再锁内复制整组快照。", "这保证消费者不会读到结构体更新到一半的状态，但不会让磁力计和 MPU 在同一时刻完成内部转换，它们仍存在采样时间差。", "同样，诊断逐个复制不同传感器状态，只能保证每份快照内部一致，不能把这一组记录解释成整个系统同时采样的瞬间。", "数据链路需要同时携带有效性和年龄。扩大锁范围去追求表面上的同时性，通常只会增加等待，并不能让外部器件的转换真正同步。"], "audio": "voice/neural-scene-10.wav", "seconds": 30.267083333333332, "beats": [0.35, 8.028, 15.86, 22.414], "id": 10, "evidence": {"scene": 10, "file": "application/app_imu.c", "line": 462, "sha256": "54e15580588e8ee0bc5e1f824416c93730e7bd6e24b4302446880b6939094fc6", "excerpt": "uint8_t App_IMU_GetSnapshot(app_imu_struct *imu)\n{\n    uint8_t online;\n\n    if (imu == NULL)\n    {"}, "frames": 909, "startFrame": 8481, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "IMU 在锁外读取和融合，再锁内发布", "startMs": 420, "endMs": 3441, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "整组数据。", "startMs": 3402, "endMs": 4076, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "消费者也锁内复制，避免读到只更新一半", "startMs": 4515, "endMs": 7393, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "的结构体。", "startMs": 7333, "endMs": 7908, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "但快照一致，不代表器件同时采样。", "startMs": 8098, "endMs": 11031, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "磁力计和 MPU 的内部转换仍有时间差，", "startMs": 11469, "endMs": 14226, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "加锁改变不了这一点。", "startMs": 14408, "endMs": 15740, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "诊断逐个复制传感器状态，也是每份内部", "startMs": 15930, "endMs": 18895, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "一致，不能解释成整组数据在同一瞬间产生。", "startMs": 18846, "endMs": 22294, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "所以还要携带有效性和年龄。", "startMs": 22484, "endMs": 25012, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "扩大锁范围，只会延长等待，并不能让外部", "startMs": 25450, "endMs": 28723, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "器件真正同步。", "startMs": 28673, "endMs": 29797, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
