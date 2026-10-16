import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene21 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "掉电一致性落在提交顺序", "chapter": "05 · 持久化提交语义", "kind": "memory", "nodes": ["先读两槽且成功", "选择非最新槽", "分页写入", "回读比对 + 解码"], "formula": "保存失败保留旧副本；RAM 校准成功与持久化成功分别报告", "source": "application/app_imu_store.c|app_imu_store_result app_imu_store_save", "lines": ["保存前必须成功读取两个槽位，不能把通信错误误当成空白槽位。确认有效状态后，选择不是最新副本的另一槽写入。", "二十八字节记录跨过十六字节页边界，所以驱动分段写，并用有界应答轮询等待内部写周期，避免页地址回卷覆盖。", "写完后还会回读，逐字节比对并再次解码。这样即使写保护让总线看似完成，也不能只凭发送返回值报告保存成功。", "双副本降低部分写入破坏当前有效记录的风险，但仍需断电注入验证。RAM 已校准而保存失败是独立状态，不应伪装成全部失败或全部成功。"], "audio": "voice/neural-scene-21.wav", "seconds": 30.979791666666667, "beats": [0.35, 8.467, 14.714, 22.425], "id": 21, "evidence": {"scene": 21, "file": "application/app_imu_store.c", "line": 161, "sha256": "511820e6f30bdfeb2abb7c7e3f98cb07804a0f280ae9c0045d08a3a0a0ee11f1", "excerpt": "app_imu_store_result app_imu_store_save(const mpu6050_offset_struct *offset)\n{\n    uint8_t record[2][IMU_STORE_RECORD_BYTES], verify[IMU_STORE_RECORD_BYTES], valid[2];\n    uint8_t i, latest, target;\n    uint32_t sequence = 1;\n    uint32_t started_ms = HAL_GetTick();"}, "frames": 930, "startFrame": 18733, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "保存前先成功读两个槽，不能把通信错误", "startMs": 420, "endMs": 3726, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "当空白。", "startMs": 3688, "endMs": 4329, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "之后选非最新槽写入，保留上一份有效记录。", "startMs": 4756, "endMs": 8347, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "二十八字节记录会跨十六字节页，所以驱动", "startMs": 8537, "endMs": 11469, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "分段写，再用有界 ACK 轮询等待内部", "startMs": 11431, "endMs": 13958, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "写入结束。", "startMs": 13909, "endMs": 14594, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "写完还要回读，比对字节并重新解码。", "startMs": 14784, "endMs": 17805, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "这样写保护即使没让总线报错，也不能被", "startMs": 18243, "endMs": 21154, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "误报成保存成功。", "startMs": 21105, "endMs": 22305, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "双副本仍需断电实验验证。", "startMs": 22495, "endMs": 24694, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "RAM 校准成功而保存失败，要单独报告，", "startMs": 25132, "endMs": 28152, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "不能笼统说全成功或全失败。", "startMs": 28366, "endMs": 30510, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
