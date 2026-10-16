import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene05 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "配置最终值比生成值更重要", "chapter": "02 · 调度与等待成本", "kind": "priority", "nodes": ["APB2 / SPI1", "APB1 / TIM2", "DMA IRQ：0 → 12", "USART1 IRQ：0"], "formula": "时钟推导与中断约束，必须沿初始化顺序追到最后一次写入", "source": "interface/lcd.c|HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 12, 0);", "lines": ["SPI1 的配置可以推导五十兆赫串行时钟，TIM2 则需要考虑 APB 分频后的定时器倍频关系。直接拿总线频率代入可能算错计数尺度。", "另一个典型例子是 DMA 中断。生成文件把优先级设为零，但液晶初始化随后改成十二，这才是显示完成通知使用的运行配置。", "当前 DMA 回调链调用内核的中断通知接口，因此必须满足系统调用优先级约束。不能只检查生成文件就判定配置对错。", "USART1 仍使用优先级零，其回调只处理驱动状态与字节缓冲。以后若在这里加入内核中断接口，必须重新审视优先级边界。"], "audio": "voice/neural-scene-05.wav", "seconds": 31.172333333333334, "beats": [0.35, 7.837, 16.008, 23.122], "id": 5, "evidence": {"scene": 5, "file": "interface/lcd.c", "line": 272, "sha256": "4afbcd58c5761b97548152852c39b63f3b83102c74cf73617e9655efa7c66544", "excerpt": "    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 12, 0);\n\n    LCD_RES_Clr(); /* 硬复位 */\n    lcd_delay_ms(100);\n    LCD_RES_Set();\n    lcd_delay_ms(100);"}, "frames": 936, "startFrame": 3700, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "SPI1 可推导出五十兆赫时钟；", "startMs": 420, "endMs": 2739, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "TIM2 则要考虑 APB 分频后的", "startMs": 3030, "endMs": 5370, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "定时器倍频，不能直接套总线频率。", "startMs": 5332, "endMs": 7717, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "DMA 优先级也要追到最终值。", "startMs": 7907, "endMs": 10335, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "生成文件里是零，液晶初始化又改成十二，", "startMs": 10751, "endMs": 14156, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "运行时采用后面的配置。", "startMs": 14293, "endMs": 15888, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "因为完成回调会调用内核通知接口，DMA ", "startMs": 16078, "endMs": 19208, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "中断必须满足系统调用的优先级要求，不能", "startMs": 19159, "endMs": 22114, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "只看生成文件。", "startMs": 22065, "endMs": 23002, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "USART1 目前仍是零，但回调没有", "startMs": 23192, "endMs": 25774, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "调用内核中断接口。", "startMs": 25725, "endMs": 27013, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "以后加了通知或者队列，这个边界就要重查。", "startMs": 27452, "endMs": 30702, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
