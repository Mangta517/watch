import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene11 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "软件总线的成本怎么拆", "chapter": "03 · 数据一致性与临界区", "kind": "cost", "nodes": ["地址 / 寄存器开销", "数据位与 ACK", "GPIO 方向切换", "任务抢占拉长事务"], "formula": "Ttransaction = Twire + Tgpio + Tsoftware + Tpreemption", "source": "common/my_i2c.c|#define MY_I2C_SENSOR_DELAY_LOOPS", "lines": ["软件总线的耗时不仅取决于有效数据字节，还包含地址阶段、重复起始、应答、方向切换和每一位的延时循环。", "当前 SENSOR 位延时循环是其他总线的两倍。它增加线路稳定余量，也直接增加 CPU 忙等成本，不能同时把它宣传为吞吐优化。", "方向缓存省掉同方向重复配置，减少的是 GPIO 初始化开销；短临界区保护读改写，减少的是竞争风险，二者目标不同。", "性能测量应区分连续事务线时和驱动墙钟耗时。后者包含抢占；只用理想频率乘字节数，会低估真实任务占用与读取间隔。"], "audio": "voice/neural-scene-11.wav", "seconds": 30.666541666666667, "beats": [0.35, 8.576, 15.333, 23.022], "id": 11, "evidence": {"scene": 11, "file": "common/my_i2c.c", "line": 21, "sha256": "f48dcc597a2daa2eee9fbd8cdb388b00cdf66277eacbff491bff6abb4f0b99cb", "excerpt": "#define MY_I2C_SENSOR_DELAY_LOOPS 160U\n\n/* ================================================================ 总线表 */\ntypedef struct\n{\n    GPIO_TypeDef *scl_port;"}, "frames": 920, "startFrame": 9390, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "软件总线的成本，不止有效字节。", "startMs": 420, "endMs": 2991, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "地址、应答、重复起始、方向切换和位延时，", "startMs": 3429, "endMs": 7415, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "全要算进去。", "startMs": 7519, "endMs": 8456, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "SENSOR 的位延时循环是其他总线", "startMs": 8646, "endMs": 10812, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "的两倍。", "startMs": 10752, "endMs": 11261, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "它留了线路稳定余量，同时也增加 CPU ", "startMs": 11688, "endMs": 14566, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "忙等成本。", "startMs": 14517, "endMs": 15213, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "方向缓存省的是重复 GPIO 配置；", "startMs": 15403, "endMs": 17974, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "短临界区防的是读改写竞争。", "startMs": 18221, "endMs": 20365, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "两项措施解决的不是同一个问题。", "startMs": 20803, "endMs": 22902, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "测量还要区分线上的时间和驱动墙钟时间。", "startMs": 23092, "endMs": 26003, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "后者包含抢占，用理想频率乘字节数会低估", "startMs": 26441, "endMs": 29955, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "它。", "startMs": 29895, "endMs": 30197, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
