import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene33 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "接收链路的背压与重同步", "chapter": "09 · 通信背压与交接", "kind": "uart", "nodes": ["ISR 写环形区", "唯一任务消费者", "满时丢新字节", "解析器处理拆包粘包"], "formula": "1023 B 可用容量 ≈ 88.8 ms 满线速突发缓冲", "source": "interface/kt6368a.h|#define KT6368A_RX_BUFFER_SIZE", "lines": ["一千零二十三字节有效环形容量，在十一万五千二百波特、八无一配置下，大约能吸收八十八点八毫秒满线速数据，不能无限抵抗消费停顿。", "接收中断只入队，界面任务是唯一消费者，再把同一份字节交给时钟、天气和命令解析。增加第二个抢读者会破坏消息完整性。", "缓冲满时丢新字节并计数，因此解析器需要在换行、帧头或校验失败后重新同步。仅仅增大缓冲，只是延长容忍窗口，并未解决持续吞吐不足。", "排查丢包应同时记录环形溢出、解析拒绝和消费者最大停顿。只有手机端出现序号缺口，无法直接断定是无线链路丢失。"], "audio": "voice/neural-scene-33.wav", "seconds": 30.782375, "beats": [0.35, 7.08, 14.462, 22.864], "id": 33, "evidence": {"scene": 33, "file": "interface/kt6368a.h", "line": 20, "sha256": "3790902912ba9bb6ac21b6a675f3a496a03132e542e2d1fa22a5f953edb6d236", "excerpt": "#define KT6368A_RX_BUFFER_SIZE 1024U /* 环形缓冲有效容量1023字节 */\n#define KT6368A_TX_MAX_SIZE    512U\n#define KT6368A_BOOT_DELAY_MS  1200U /* 蓝牙模块上电等待时间 */\n#define KT6368A_AT_TIMEOUT_MS  500U\n\n    /* 调用前须完成MX_GPIO_Init/MX_USART1_UART_Init。"}, "frames": 924, "startFrame": 30150, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "接收环形区可用一千零二十三字节，大约", "startMs": 420, "endMs": 3451, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "容纳八十八点八毫秒满线速突发，吸收停顿", "startMs": 3402, "endMs": 6368, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "的能力有限。", "startMs": 6308, "endMs": 6960, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "中断只入队，界面是唯一消费者，再把同", "startMs": 7150, "endMs": 10324, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "一份字节交给各解析器。", "startMs": 10264, "endMs": 11969, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "第二个抢读者会破坏拼包。", "startMs": 12407, "endMs": 14342, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "满时丢新字节，所以解析器要能重同步。", "startMs": 14532, "endMs": 17991, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "加大缓冲只是延长容忍窗口，解决不了持续", "startMs": 18408, "endMs": 22031, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "吞吐不足。", "startMs": 21982, "endMs": 22744, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "丢包要关联环形溢出、解析拒绝和消费停顿。", "startMs": 22934, "endMs": 26579, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "只有手机序号缺口，还不能直接认定无线", "startMs": 27018, "endMs": 29907, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "丢包。", "startMs": 29858, "endMs": 30312, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
