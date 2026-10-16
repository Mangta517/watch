import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene22 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "跨寄存器读取也会撕裂", "chapter": "06 · 信号与候选状态", "kind": "register", "nodes": ["EM7028 高字节", "再读低字节", "重读高字节", "变化则 NOT_READY"], "formula": "高—低—高检查抑制跨更新拼接；不是硬件同步采样证明", "source": "interface/em7028.c|if (high != high_check)", "lines": ["EM7028 当前没有直接把两次独立寄存器读取当作天然一致。它先读高字节，再读低字节，最后重新读取高字节。", "如果两次高字节不同，就返回未就绪，拒绝把跨更新拼出来的整数送进算法。这样把数据变化与总线错误区分开。", "每次开始测量还重新确认器件标识和配置，而不是永远信任开机时的 ready。失败信息先保存，再执行关闭照明与测量的清理。", "优化这一链路时，不能只数函数调用次数。删除重读或启动验证必须有器件语义依据，否则减少的事务可能以偶发异常样本为代价。"], "audio": "voice/neural-scene-22.wav", "seconds": 32.492916666666666, "beats": [0.35, 7.771, 15.394, 23.993], "id": 22, "evidence": {"scene": 22, "file": "interface/em7028.c", "line": 196, "sha256": "92891c7af510b098380bbd4b73c1a359ac4b8caabd50bfc9dd3d5417fac34b34", "excerpt": "    if (high != high_check)\n    {\n        em7028_last_fault.kind = EM7028_FAULT_DATA_CHANGED;\n        em7028_last_fault.reg = EM7028_REG_DATA_HIGH;\n        return EM7028_NOT_READY;\n    }"}, "frames": 975, "startFrame": 19663, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "心率读取先高字节，再低字节，最后重读", "startMs": 420, "endMs": 3737, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "高字节，避免把独立寄存器读取直接当作", "startMs": 3698, "endMs": 6741, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "一份一致数据。", "startMs": 6692, "endMs": 7651, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "两次高字节不同，就返回未就绪，拒绝拼接", "startMs": 7841, "endMs": 11443, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "结果。", "startMs": 11394, "endMs": 11805, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "这把采样变化与总线通信错误区分开了。", "startMs": 12243, "endMs": 15274, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "每次开始测量还重查标识和配置。", "startMs": 15464, "endMs": 18353, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "失败时先保存错误，再关照明，避免清理", "startMs": 18791, "endMs": 22382, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "过程覆盖最初故障。", "startMs": 22333, "endMs": 23873, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "所以优化不能只数读取次数。", "startMs": 24063, "endMs": 26393, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "删掉验证之前，得有器件语义依据，否则", "startMs": 26831, "endMs": 30038, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "省下的事务可能换来异常样本。", "startMs": 30000, "endMs": 32023, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
