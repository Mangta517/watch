import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene26 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "气压合并读取保留配对语义", "chapter": "07 · 多速率链路与长尾", "kind": "pipeline", "nodes": ["一次状态读取", "压力与温度都就绪", "连续六字节", "补偿后再发布"], "formula": "少一次重复事务，也要保持未就绪与错误的区分", "source": "interface/spl006.c|uint16_t spl006_read_sample", "lines": ["当前气压路径先读取一次状态，只有压力和温度都就绪，才连续读取六字节原始数据，避免先消耗其中一路就绪标志。", "这减少了重复事务，但保留了压力与温度作为一对输入进入补偿函数的语义。不能用旧温度悄悄替换而声称只是性能优化。", "结果码把压力与温度状态分别编码，补偿失败又有单独标记。初始化系数无效时，原始数据读到也不能认为压力已经可信。", "验证应该覆盖只有一路就绪、总线失败和系数初始化失败，并比较合并前后的事务数量与墙钟耗时，防止用少报错误换取表面成功率。"], "audio": "voice/neural-scene-26.wav", "seconds": 29.05125, "beats": [0.35, 6.696, 13.35, 21.236], "id": 26, "evidence": {"scene": 26, "file": "interface/spl006.c", "line": 230, "sha256": "ba75199b3888dde08e10615e18391c1d3716dfe2f3d55afa9c487a1cddc2d632", "excerpt": "uint16_t spl006_read_sample(int32_t *pressure_raw, int32_t *temperature_raw)\n{\n    uint8_t status, data[6];\n    uint16_t result = 0;\n    int32_t pressure, temperature;\n    if (spl006_read_reg(SPL06_MEAS_CFG, &status) != 0)"}, "frames": 872, "startFrame": 23499, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "气压先读一次状态，压力和温度都就绪后，", "startMs": 420, "endMs": 3462, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "再连续读六字节，保留两路配对的读取语义。", "startMs": 3578, "endMs": 6576, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "这样减少重复事务，但不能悄悄改成新压力", "startMs": 6766, "endMs": 9984, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "配旧温度，然后把行为变化也叫作性能优化。", "startMs": 9946, "endMs": 13230, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "压力、温度和补偿失败都有独立结果。", "startMs": 13420, "endMs": 16824, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "系数无效时，原始数据读到了，也不代表", "startMs": 17262, "endMs": 20480, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "压力可信。", "startMs": 20442, "endMs": 21116, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "回归覆盖单路就绪、总线失败和初始化失败，", "startMs": 21306, "endMs": 24787, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "再比事务数与耗时，避免靠少报错误提高", "startMs": 24936, "endMs": 28132, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "成功率。", "startMs": 28083, "endMs": 28581, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
