import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene13 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "临界区要包住真正的竞争", "chapter": "03 · 数据一致性与临界区", "kind": "critical", "nodes": ["读取 MODER 旧值", "另一任务改 PB13", "旧值覆盖新配置", "保存 / 恢复 PRIMASK"], "formula": "保护 GPIO 配置 RMW；不保护整段 I²C 事务", "source": "common/my_i2c.c|uint32_t primask = __get_PRIMASK();", "lines": ["GPIOB 的方向配置是读改写。触摸修改 PB4 期间被抢占，sensor 修改 PB13，然后触摸把旧副本写回，就可能覆盖刚完成的 PB13 配置。", "这与两条总线有没有共用 SDA 无关，竞争发生在同一个端口配置寄存器。只给每条总线加自己的锁，也没有保护这个共享点。", "当前实现保存 PRIMASK，关闭中断完成短配置，再恢复原值。恢复而不是无条件开中断，才能保留调用者原先的屏蔽状态。", "边界必须足够小：只围住 GPIO 配置和方向缓存更新。把整个寄存器读写事务都关中断，会把微小共享点放大成系统延迟源。"], "audio": "voice/neural-scene-13.wav", "seconds": 32.12629166666667, "beats": [0.35, 8.528, 16.721, 23.769], "id": 13, "evidence": {"scene": 13, "file": "common/my_i2c.c", "line": 105, "sha256": "f48dcc597a2daa2eee9fbd8cdb388b00cdf66277eacbff491bff6abb4f0b99cb", "excerpt": "    uint32_t primask = __get_PRIMASK();\n    uint8_t direction = as_input + 1U;\n    if (s_sda_direction[bus] == direction)\n    {\n        return;\n    }"}, "frames": 964, "startFrame": 11211, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "看这个竞争顺序：触摸读出 GPIOB ", "startMs": 420, "endMs": 3199, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "旧值，中途被抢占，sensor 改了 ", "startMs": 3139, "endMs": 5853, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "PB13，触摸回来又写入旧值。", "startMs": 5880, "endMs": 8408, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "结果是 PB13 的新配置被覆盖了。", "startMs": 8598, "endMs": 11443, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "竞争点在端口寄存器，即使给两条总线分别", "startMs": 11870, "endMs": 15132, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "加锁，也保护不到这里。", "startMs": 15083, "endMs": 16601, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "现在的做法是保存 PRIMASK，关中", "startMs": 16791, "endMs": 19691, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "断完成短配置，再恢复原值，保留调用者", "startMs": 19642, "endMs": 22586, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "原先的屏蔽状态。", "startMs": 22548, "endMs": 23649, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "临界区只包住配置读改写和方向缓存更新。", "startMs": 23839, "endMs": 27397, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "别把整次通信也塞进去，否则小竞争会变成", "startMs": 27835, "endMs": 30998, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "长延迟。", "startMs": 30949, "endMs": 31656, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
