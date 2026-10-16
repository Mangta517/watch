import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene08 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "忙等、阻塞与优先级反转", "chapter": "02 · 调度与等待成本", "kind": "cost", "nodes": ["位延时占用 CPU", "vTaskDelay 进入阻塞", "互斥锁可等待", "PRIMASK 不可调度"], "formula": "同样等 2 ms，对系统的影响可能完全不同", "source": "common/my_i2c.c|static void my_i2c_delay", "lines": ["软件总线的位延时是忙等，执行期间占用 CPU；任务延时和通知等待则允许调度其他就绪任务，二者不能都笼统称为阻塞函数。", "如果高优先级界面等低优先级 sensor 持有的互斥锁，内核可以通过优先级继承缓解中间优先级干扰，但持锁代码本身仍必须尽快完成。", "关闭中断的临界区更严格：它影响中断响应，并可能推迟调度。因此不能把毫秒级总线事务或串口发送包进这种保护区。", "判断优化价值时，分别标出 CPU 占用、任务阻塞和关中断时长。只有这样，降低一个函数耗时才能对应到真正改善的系统指标。"], "audio": "voice/neural-scene-08.wav", "seconds": 32.3175, "beats": [0.35, 8.269, 15.493, 23.434], "id": 8, "evidence": {"scene": 8, "file": "common/my_i2c.c", "line": 46, "sha256": "f48dcc597a2daa2eee9fbd8cdb388b00cdf66277eacbff491bff6abb4f0b99cb", "excerpt": "static void my_i2c_delay(my_i2c_bus_t bus)\n{\n    volatile uint32_t n;\n    if (bus == MY_I2C_BUS_SENSOR)\n    {\n        n = MY_I2C_SENSOR_DELAY_LOOPS;"}, "frames": 970, "startFrame": 6545, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "同样等两毫秒，影响可能完全不同。", "startMs": 420, "endMs": 3068, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "软件总线的位延时占着 CPU，任务延时", "startMs": 3506, "endMs": 6844, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "则让其他任务运行。", "startMs": 6872, "endMs": 8149, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "高优先级界面等 sensor 的互斥", "startMs": 8339, "endMs": 10450, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "锁时，优先级继承可以缓解干扰，但持锁", "startMs": 10401, "endMs": 13761, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "工作还是要尽快完成。", "startMs": 13701, "endMs": 15373, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "关中断比互斥更严格，它会影响中断响应", "startMs": 15563, "endMs": 18945, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "和调度。", "startMs": 18907, "endMs": 19461, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "因此不能把整次总线事务或串口发送包进去。", "startMs": 19899, "endMs": 23314, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "分析耗时，先拆出 CPU 执行、任务", "startMs": 23504, "endMs": 26678, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "等待和关中断时间。", "startMs": 26629, "endMs": 28235, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "知道省掉哪部分，才能判断系统实际收益。", "startMs": 28673, "endMs": 31847, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
