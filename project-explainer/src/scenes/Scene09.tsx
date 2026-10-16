import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene09 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "内存预算决定刷新策略", "chapter": "03 · 数据一致性与临界区", "kind": "memory", "nodes": ["RTOS 堆 16 KiB", "LVGL 池 44 KiB", "双缓冲 26,880 B", "链接占用 / 运行峰值"], "formula": "全屏 134,400 B > SRAM 131,072 B；局部缓冲不是装饰", "source": "Middlewares/FreeRTOS/include/FreeRTOSConfig.h|#define configTOTAL_HEAP_SIZE", "lines": ["全屏十六位像素需要十三万四千四百字节，已经超过这颗芯片的一百二十八千字节 SRAM，完整帧缓冲在当前约束下就不成立。", "工程选择两块十分之一屏缓冲，占二万六千八百八十字节。它们是静态区，不属于十六千字节的内核堆，也不属于 LVGL 内存池。", "增加缓冲可以减少分块开销，但会挤压对象池、栈和其他静态状态。只盯着一个堆的空闲值，很容易错误判断还有多少可用空间。", "正确证据来自链接映射与运行峰值的组合。栈水位是历史扫描估计，LVGL 快照峰值可能漏掉短暂峰值，两者都需要覆盖复杂页面切换。"], "audio": "voice/neural-scene-09.wav", "seconds": 32.196875, "beats": [0.35, 7.398, 15.329, 23.489], "id": 9, "evidence": {"scene": 9, "file": "Middlewares/FreeRTOS/include/FreeRTOSConfig.h", "line": 50, "sha256": "56004b000b912130a2d0738bc91d3b78bcb277488e3e3e7e244b853b5202af87", "excerpt": "#define configTOTAL_HEAP_SIZE\t\t( ( size_t ) ( 16 * 1024 ) )\t/* 10K 偏紧: lvgl任务4K栈+idle+mutex≈6.5K, 留余量防建任务失败 */\n#define configMAX_TASK_NAME_LEN\t\t( 16 )\n#define configUSE_TRACE_FACILITY\t1\n\n#include \"app_diag.h\"\n#define configGENERATE_RUN_TIME_STATS 1"}, "frames": 966, "startFrame": 7515, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "全屏十六位像素要十三万四千四百字节，", "startMs": 420, "endMs": 3616, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "已经超过这颗芯片的 SRAM，所以必须", "startMs": 3874, "endMs": 6258, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "采用局部刷新。", "startMs": 6209, "endMs": 7278, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "两块十分之一屏缓冲，共占二万六千八百八十", "startMs": 7468, "endMs": 10982, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "字节。", "startMs": 10933, "endMs": 11355, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "它们在静态区，不属于内核堆或 LVGL ", "startMs": 11793, "endMs": 14858, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "池。", "startMs": 14798, "endMs": 15208, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "增大缓冲能减少分块，却会挤压对象池、", "startMs": 15398, "endMs": 18847, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "栈和其他状态。", "startMs": 18962, "endMs": 20294, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "只盯着一个堆的剩余量，判断不完整。", "startMs": 20732, "endMs": 23369, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "内存要结合链接结果和运行峰值看。", "startMs": 23559, "endMs": 26240, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "栈水位是估计，快照也可能漏掉瞬时峰值，", "startMs": 26667, "endMs": 30082, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "复杂切页必须覆盖。", "startMs": 30329, "endMs": 31727, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
