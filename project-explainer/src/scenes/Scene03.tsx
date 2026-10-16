import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene03 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "独立总线仍共享调度资源", "chapter": "01 · 所有权与性能模型", "kind": "ownership", "nodes": ["SENSOR / sensor", "TOUCH / lvgl", "EEPROM / sensor", "GPIOB / CPU 共享"], "formula": "物理隔离 → 事务隔离；不等于 CPU 与寄存器隔离", "source": "common/my_i2c.c|static const my_i2c_pin_t", "lines": ["三条软件总线把触摸、传感器和存储器分开，避免界面读触摸时直接插入惯性器件的同一条事务。", "但它们仍共享 CPU。高优先级界面任务执行软件触摸事务时，传感器任务会延后，独立引脚不会自动提供并行执行能力。", "存储器虽然位于另一条总线，却同样归 sensor 使用。因此校准保存的等待会影响这个任务后面的心率和电池服务。", "另外，PB4 与 PB13 共享 GPIOB 配置寄存器。工程上的独立性必须分别检查电气总线、执行上下文和寄存器访问三个层面。"], "audio": "voice/neural-scene-03.wav", "seconds": 30.935916666666667, "beats": [0.35, 8.883, 16.221, 23.176], "id": 3, "evidence": {"scene": 3, "file": "common/my_i2c.c", "line": 32, "sha256": "f48dcc597a2daa2eee9fbd8cdb388b00cdf66277eacbff491bff6abb4f0b99cb", "excerpt": "static const my_i2c_pin_t my_i2c_pin[MY_I2C_BUS_COUNT] = {\n    /* SENSOR 总线: MPU6050 等 */\n    {SENSOR_SCL_GPIO_Port, SENSOR_SCL_Pin, SENSOR_SDA_GPIO_Port, SENSOR_SDA_Pin},\n    /* TP 总线: 触摸屏 (touch.c 用自己的位翻实现, 本表项为其保留) */\n    {TP_SCL_GPIO_Port, TP_SCL_Pin, TP_SDA_GPIO_Port, TP_SDA_Pin},\n    {EEPROM_SCL_GPIO_Port, EEPROM_SCL_Pin, EEPROM_SDA_GPIO_Port, EEPROM_SDA_Pin},"}, "frames": 929, "startFrame": 1770, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "三条软件总线把触摸、传感器和存储器分开", "startMs": 420, "endMs": 4340, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "了。", "startMs": 4280, "endMs": 4482, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "界面读触摸时，不会直接打断同一条传感器", "startMs": 4909, "endMs": 8434, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "事务。", "startMs": 8385, "endMs": 8763, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "但 CPU 还是同一个。", "startMs": 8953, "endMs": 10724, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "高优先级界面在模拟触摸总线时，sensor ", "startMs": 11151, "endMs": 14468, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "仍然得等它让出处理器。", "startMs": 14495, "endMs": 16101, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "EEPROM 也由 sensor 操作。", "startMs": 16291, "endMs": 17952, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "所以保存校准的等待，会推迟同一个任务", "startMs": 18391, "endMs": 21411, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "后面的心率和电池服务。", "startMs": 21362, "endMs": 23056, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "还有 GPIOB：PB4 和 PB13 ", "startMs": 23246, "endMs": 26091, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "虽然不是同一根线，方向配置却共享寄存器，", "startMs": 26053, "endMs": 29194, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "这就是另一层竞争。", "startMs": 29353, "endMs": 30466, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
