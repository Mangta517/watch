import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene19 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "校准模型决定可修正的误差", "chapter": "04 · 采样时间与融合约束", "kind": "quaternion", "nodes": ["芯片坐标去偏置", "对角比例校正", "再映射机体轴", "覆盖不足拒绝提交"], "formula": "m_body = axis_map(diag(scale) · (m_sensor − offset))", "source": "application/app_imu.c|static void mag_cal_command", "lines": ["当前磁校准先在芯片坐标中去偏置和按轴缩放，然后再映射到机体坐标。这个顺序把器件校准参数和安装映射分开。", "最大最小值方法只估计偏置与对角比例，不包含交叉轴项。因此能够改善轴向椭球，却不能被称为完整的三维软铁矩阵校正。", "提交条件要求足够样本和每轴跨度。样本数量很多但姿态覆盖很窄，仍然可能无法辨识参数，所以覆盖范围比采样总数更关键。", "如果环境磁场在校准期间发生变化，最大最小值也会包含环境变化。验证应比较不同姿态下的模长和航向一致性，而不只看保存成功。"], "audio": "voice/neural-scene-19.wav", "seconds": 30.887291666666666, "beats": [0.35, 7.76, 15.324, 22.914], "id": 19, "evidence": {"scene": 19, "file": "application/app_imu.c", "line": 154, "sha256": "54e15580588e8ee0bc5e1f824416c93730e7bd6e24b4302446880b6939094fc6", "excerpt": "static void mag_cal_command(uint8_t command)\n{\n    uint8_t i;\n    float radius[3], average;\n\n    if (command == APP_MAG_CMD_START) /* 保留上次已生效的校准系数 */"}, "frames": 927, "startFrame": 16857, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "磁校准先在芯片坐标下去偏置、按轴缩放，", "startMs": 420, "endMs": 4000, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "之后再映射到机体坐标，把校准与安装方向", "startMs": 4236, "endMs": 7245, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "分开。", "startMs": 7196, "endMs": 7640, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "最大最小值法只估计偏置和对角比例，不", "startMs": 7830, "endMs": 11530, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "含交叉轴项，因此不能称为完整软铁矩阵", "startMs": 11470, "endMs": 14853, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "校正。", "startMs": 14804, "endMs": 15204, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "样本够多，还得每轴跨度够大。", "startMs": 15394, "endMs": 17647, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "只在一个方向反复晃动，采得再多，也未必", "startMs": 18074, "endMs": 21303, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "能辨识出完整参数。", "startMs": 21254, "endMs": 22794, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "环境变化也会进入最大最小值。", "startMs": 22984, "endMs": 25456, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "验证应该比较不同姿态下的模长与航向，", "startMs": 25884, "endMs": 28455, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "不能只看保存是否成功。", "startMs": 28745, "endMs": 30417, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
