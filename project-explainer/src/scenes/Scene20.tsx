import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene20 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "持久化的有效性是多重条件", "chapter": "05 · 持久化提交语义", "kind": "memory", "nodes": ["MPU1 格式标识", "量程标记", "CRC32 完整性", "序号判新 / 回绕"], "formula": "记录可读 ≠ 校准可恢复；量程属于数据语义", "source": "application/app_imu_store.c|#define IMU_STORE_MAGIC", "lines": ["恢复校准不只看 EEPROM 有没有字节。记录必须同时满足格式标识、当前量程和 CRC 校验，才能把六轴偏移解释为有效数据。", "量程标记尤其重要：同样的原始计数偏移，在不同灵敏度下对应不同物理量。内容完整但量程不符，仍然不能恢复。", "两个副本都有效时，按无符号序号差选择较新记录，以处理正常序号回绕。选新逻辑与校验逻辑分别解决版本顺序和内容完整性。", "状态五表示恢复已保存偏移，而非本次完成静止校准。运行诊断应保留这个来源信息，否则很难解释为什么这次启动没有校准帧。"], "audio": "voice/neural-scene-20.wav", "seconds": 31.6315, "beats": [0.35, 8.072, 15.706, 23.593], "id": 20, "evidence": {"scene": 20, "file": "application/app_imu_store.c", "line": 9, "sha256": "511820e6f30bdfeb2abb7c7e3f98cb07804a0f280ae9c0045d08a3a0a0ee11f1", "excerpt": "#define IMU_STORE_MAGIC        0x3155504DU /* 小端字符MPU1 */\n#define IMU_STORE_RECORD_BYTES 28U\n#define IMU_STORE_CRC_OFFSET   24U\n#define IMU_STORE_SLOT_BYTES   32U\n#define IMU_STORE_BASE         0x60U\n#define IMU_STORE_RANGE        ((uint32_t)MPU6050_ACCEL_RANGE << 8 | MPU6050_GYRO_RANGE)"}, "frames": 949, "startFrame": 17784, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "恢复零偏要同时验证格式、量程和 CRC。", "startMs": 420, "endMs": 4022, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "EEPROM 里能读到字节，不等于这些", "startMs": 4460, "endMs": 6899, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "字节可以直接用。", "startMs": 6850, "endMs": 7952, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "量程尤其重要：同样的计数偏移，换了灵敏度", "startMs": 8142, "endMs": 11765, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "就对应不同物理量。", "startMs": 11793, "endMs": 13070, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "内容没坏，也可能语义不匹配。", "startMs": 13509, "endMs": 15586, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "两份都有效时，按无符号序号差选较新记录，", "startMs": 15776, "endMs": 19531, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "以兼容回绕。", "startMs": 19745, "endMs": 20683, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "序号和校验解决的是不同问题。", "startMs": 21121, "endMs": 23472, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "恢复状态五，表示用了保存记录，不是这", "startMs": 23662, "endMs": 26497, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "次又完成了静止校准。", "startMs": 26437, "endMs": 27922, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "来源信息保留下来，启动过程才好解释。", "startMs": 28360, "endMs": 31161, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
