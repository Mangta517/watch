import {SceneFrame} from '../SceneFrame';
import {BasicCaptions} from '../basic-captions';
import {useVideoConfig} from 'remotion';

export const Scene31 = () => {
    const {fps} = useVideoConfig();
    return (
        <SceneFrame scene={{"title": "界面重建要避开事件生命周期", "chapter": "08 · 显示吞吐与生命周期", "kind": "pipeline", "nodes": ["事件只记录请求", "等待触点释放", "等待过渡结束", "统一重建页面"], "formula": "延后执行，把事件目标生命周期与对象删除解耦", "source": "application/app_ui.c|if (s_screen_on && !s_transition && !touch_pressed())", "lines": ["当前页面跳转不是在任意事件里立即删除和重建对象，而是先设置待处理页面或返回请求，在界面定时器里统一消费。", "真正重建还要求触点已经释放且上一个过渡结束，这样避免滑动手势落成点击，也避免事件目标在处理链中途失效。", "刷新数据同样避开过渡阶段。减少重复对象修改可以降低脏区和内存峰值，但必须保持页面状态与用户输入的既有语义。", "按键熄屏只切换背光和输入，不代表任务停止或处理器深睡。评价整机功耗时，要把显示功耗与 CPU、传感器持续运行分开测量。"], "audio": "voice/neural-scene-31.wav", "seconds": 31.051708333333334, "beats": [0.35, 7.519, 14.534, 22.136], "id": 31, "evidence": {"scene": 31, "file": "application/app_ui.c", "line": 1898, "sha256": "63f735ac9f8d6bf745f92ba1c64f08cfb396573bedf44841eb0a810b54b2b7cc", "excerpt": "    if (s_screen_on && !s_transition && !touch_pressed())\n    {\n        if (s_pending_back)\n        {\n            s_pending_back = 0;\n            s_pending_page = PAGE_COUNT;"}, "frames": 932, "startFrame": 28234, "playbackRate": 1.0}}>
            <BasicCaptions
                name="中文逐句字幕"
                premountFor={fps}
                width={2125}
                combineTokensWithinMilliseconds={1}
                style={{position: 'absolute', left: 110, top: 900, scale: 0.8, transformOrigin: 'top left'}}
                captions={[{"text": "页面事件先记录跳转请求，之后在界面定时器", "startMs": 420, "endMs": 4098, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "里消费，避免处理事件时立刻删除它的目标", "startMs": 4038, "endMs": 7059, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "对象。", "startMs": 7010, "endMs": 7399, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "重建还要等触点释放、过渡结束，防止滑动", "startMs": 7589, "endMs": 11037, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "变点击，也保住事件处理链中的对象生命", "startMs": 10999, "endMs": 13987, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "周期。", "startMs": 13938, "endMs": 14414, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "刷新也避开过渡。", "startMs": 14604, "endMs": 15881, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "少改对象能减少脏区和内存峰值，但必须", "startMs": 16320, "endMs": 19822, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "保留原有页面状态与交互语义。", "startMs": 19773, "endMs": 22015, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "熄屏这里只切背光和输入，任务仍在运行。", "startMs": 22206, "endMs": 25708, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "测功耗要分别看显示、CPU 和传感器，", "startMs": 26147, "endMs": 29123, "timestampMs": null, "confidence": null, "pageBreakAfter": true}, {"text": "不能把黑屏当深睡。", "startMs": 29283, "endMs": 30582, "timestampMs": null, "confidence": null, "pageBreakAfter": true}]}
            />
        </SceneFrame>
    );
};
