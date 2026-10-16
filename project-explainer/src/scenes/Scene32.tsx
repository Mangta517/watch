import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene32 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "异步串口把线时移出忙等", "chapter": "09 · 通信背压与交接", "kind": "ownership", "nodes": ["固定静态发送缓冲", "HAL_UART_Transmit_IT", "TX 回调记录结果", "发起者收取结果"], "formula": "36 B 约 3.125 ms；异步降低等待占用，不缩短物理线时", "source": "interface/kt6368a.c|HAL_StatusTypeDef kt6368a_write_async", "lines": ["诊断与心率调试输出使用中断异步发送，把串口线时间从界面任务的同步忙等中移出。线上的字节仍然需要同样长时间发送。", "因此发送缓冲必须静态保存，在完成或中止前不能复用。只把局部数组传进去然后立即返回，会破坏这条生命周期约束。", "驱动区分空闲、在途和完成待收取状态。完成回调只记录结果，发起者收取之后才释放事务槽位，避免另一类发送抢走完成事件。", "当前蓝牙目标是 BLE 与 SPP 同时开启，模块配置仍走原流程。无线模式改变和 MCU 串口异步化，是两个独立的性能与协议问题。"], "audio": "voice/neural-scene-32.wav", "seconds": 32.77183333333333, "beats": [0.35, 8.719, 15.871, 23.91], "id": 32, "evidence": {"scene": 32, "file": "interface/kt6368a.c", "line": 187, "sha256": "983c0986910313ef1a473ba4c3a754e582175480767cf69147c34306cc9851e7", "excerpt": "HAL_StatusTypeDef kt6368a_write_async(const uint8_t *data, uint16_t size, uint32_t timeout_ms)\n{\n    HAL_StatusTypeDef status;\n    if (data == NULL || size == 0U || size > KT6368A_TX_MAX_SIZE || timeout_ms == 0U)\n    {\n        return HAL_ERROR;"}, "frames": 984, "startFrame": 29166, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "诊断和波形改用中断发送，把线时移出界面", "startMs": 420, "endMs": 3967, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "忙等。", "startMs": 3918, "endMs": 4318, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "三十六字节约三点一毫秒，物理发送时间", "startMs": 4745, "endMs": 7952, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "并没缩短。", "startMs": 7914, "endMs": 8599, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "发送缓冲因此要一直保留到完成或中止。", "startMs": 8789, "endMs": 11831, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "局部数组发出去就返回，会破坏这条生命", "startMs": 12269, "endMs": 15104, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "周期约束。", "startMs": 15055, "endMs": 15751, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "状态分空闲、在途和完成待收取。", "startMs": 15941, "endMs": 18665, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "发起者收取后才释放槽位，防止别的发送", "startMs": 19092, "endMs": 22606, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "抢走完成事件。", "startMs": 22634, "endMs": 23790, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "蓝牙当前目标是 BLE 和 SPP ", "startMs": 23980, "endMs": 26847, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "都开启。", "startMs": 26798, "endMs": 27374, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "无线模式与 MCU 侧异步发送，是两项", "startMs": 27801, "endMs": 30876, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "独立的配置和性能问题。", "startMs": 30838, "endMs": 32302, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
