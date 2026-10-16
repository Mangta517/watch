import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene12 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "合并事务的收益与边界", "chapter": "03 · 数据一致性与临界区", "kind": "pipeline", "nodes": ["减少重复寻址", "保留器件协议", "同次读取成组", "不虚构采样原子性"], "formula": "事务合并减少固定开销；数据一致性仍由器件语义决定", "source": "interface/spl006.c|uint16_t spl006_read_sample", "lines": ["MPU 的十五字节突发读取，把状态、加速度、温度和角速度合成一次事务，减少重复寻址，也顺带释放中断锁存。", "SPL006 则先确认压力和温度都就绪，再连续读取六个原始数据字节，不提前消耗其中一路的就绪状态。", "这种优化必须遵守器件的自动递增、状态清除和数据锁存规则。EM7028 当前采用高低高读取校验，不能仅为了少一次事务就直接删掉。", "验收不仅比较耗时，还要故障注入未就绪、读取中数据变化和中途失败，确认优化后仍然只发布完整且语义有效的一组结果。"], "audio": "voice/neural-scene-12.wav", "seconds": 30.031916666666667, "beats": [0.35, 7.826, 14.161, 21.73], "id": 12, "evidence": {"scene": 12, "file": "interface/spl006.c", "line": 230, "sha256": "ba75199b3888dde08e10615e18391c1d3716dfe2f3d55afa9c487a1cddc2d632", "excerpt": "uint16_t spl006_read_sample(int32_t *pressure_raw, int32_t *temperature_raw)\n{\n    uint8_t status, data[6];\n    uint16_t result = 0;\n    int32_t pressure, temperature;\n    if (spl006_read_reg(SPL06_MEAS_CFG, &status) != 0)"}, "frames": 901, "startFrame": 10310, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "MPU 把状态和六轴数据放进一次十五", "startMs": 420, "endMs": 3364, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "字节读取，既减少寻址，也在同次事务里", "startMs": 3315, "endMs": 6467, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "释放中断锁存。", "startMs": 6506, "endMs": 7706, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "气压则等压力和温度都就绪，再连续读六", "startMs": 7896, "endMs": 11059, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "字节，避免提前消耗其中一路的就绪标志。", "startMs": 10999, "endMs": 14041, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "合并之前必须确认自动递增和锁存规则。", "startMs": 14231, "endMs": 17405, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "心率的高低高校验，就不能为了少一次读取", "startMs": 17844, "endMs": 20733, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "随意删除。", "startMs": 20771, "endMs": 21610, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "验证时既比耗时，也注入未就绪和中途失败。", "startMs": 21800, "endMs": 25533, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "减少事务以后，完整结果和错误语义都要", "startMs": 25971, "endMs": 29222, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "保住。", "startMs": 29173, "endMs": 29562, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
