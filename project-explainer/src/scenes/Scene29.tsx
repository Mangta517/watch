import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene29 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "刷屏瓶颈要拆成两个阶段", "chapter": "08 · 显示吞吐与生命周期", "kind": "dma", "nodes": ["CPU 绘制", "地址命令开销", "SPI 像素搬运", "脏区数量与面积"], "formula": "理想全屏线时约 21.5 ms；5 ms 服务周期不等于 200 fps", "source": "interface/lcd.c|LCD_WR_REG(0x2A);", "lines": ["显示链路分为 CPU 绘制和 SPI 发送。当前配置下，全屏纯像素理想线时约二十一点五毫秒，已经说明五毫秒服务周期不能代表实际帧率。", "局部刷新降低的是需要搬运的总像素数量；减少碎小脏区则可能降低每块的窗口命令与调用开销，两者不是同一个优化方向。", "双缓冲只有在绘制和传输能够重叠时才有收益。若 CPU 绘制远慢于 SPI，增大串行频率不会按比例改善页面响应。", "测量时记录脏区数量、总像素、绘制耗时与 DMA 等待时间。先确认瓶颈处于哪段，再考虑对象更新频率、区域合并或缓冲尺寸。"], "audio": "voice/neural-scene-29.wav", "seconds": 32.492875, "beats": [0.35, 7.743, 15.958, 24.426], "id": 29, "evidence": {"scene": 29, "file": "interface/lcd.c", "line": 93, "sha256": "4afbcd58c5761b97548152852c39b63f3b83102c74cf73617e9655efa7c66544", "excerpt": "    LCD_WR_REG(0x2A); /* 列地址 */\n    LCD_WR_DATA8(0x00);\n    LCD_WR_DATA8((uint8_t)x1);\n    LCD_WR_DATA8(0x00);\n    LCD_WR_DATA8((uint8_t)x2);\n    LCD_WR_REG(0x2B); /* 行地址 */"}, "frames": 975, "startFrame": 26254, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "刷屏拆成绘制和发送。", "startMs": 420, "endMs": 2180, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "全屏理想线时约二十一点五毫秒，五毫秒", "startMs": 2607, "endMs": 5825, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "服务周期显然不能当帧率。", "startMs": 5787, "endMs": 7623, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "局部刷新减少像素量，合并碎小脏区则减少", "startMs": 7813, "endMs": 11524, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "窗口命令。", "startMs": 11486, "endMs": 12160, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "两种优化针对不同成本，收益也不同。", "startMs": 12598, "endMs": 15838, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "双缓冲要靠绘制和发送重叠。", "startMs": 16028, "endMs": 18522, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "如果慢在 CPU 绘制，继续提高 SPI ", "startMs": 18961, "endMs": 22124, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "时钟，响应也未必明显改善。", "startMs": 22075, "endMs": 24306, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "先记录脏区数、总像素、绘制耗时和 DMA ", "startMs": 24496, "endMs": 28086, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "等待，再决定改更新频率、区域合并，还是", "startMs": 28037, "endMs": 31321, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "缓冲大小。", "startMs": 31283, "endMs": 32023, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
