# F411：数据链路、性能与并发进阶

面向已有嵌入式开发经验的中文讲解动画。基于当前手表工程源码，重点讲数据时效、任务长尾、临界区边界、DMA 与 UART 缓冲生命周期、校准提交语义及性能诊断归因。

- 38 段，1920×1080，30 fps；当前时长见讲解稿首部。
- 云希神经男声、整段合成、词级时间戳字幕，每段可以独立查看。
- 波形、向量和时间轴为机制示意，不是实板测量。
- 固件参考版本：本地 HEAD fbd8928。源码位置与 SHA256 见 evidence.json。

## 播放

当前预览：[F411-Deep-Dive](http://localhost:3000/F411-Deep-Dive)。点击播放，可用全屏按钮放大。

服务器关闭后，运行本目录的 `启动预览.ps1`，或执行：

```powershell
npm run dev -- --no-open
```

重新安装依赖使用 `npm ci`。Remotion 版本 4.0.535，依赖由 lockfile 固定。

本次交付是完整可播放、可编辑的 Remotion 动画工程，尚未导出 MP4。Studio 的 Render 可用于导出成片。

## 内容

| 段落 | 工程问题 |
|---|---|
| 01–03 | 性能模型、执行权、数据所有权与共享资源 |
| 04–08 | 启动关键路径、最终 IRQ 配置、任务周期、等待与抢占 |
| 09–13 | 内存预算、快照一致性、总线成本、事务合并、GPIO 临界区 |
| 14–19 | 新鲜度、dt 更新顺序、校准判据、反馈状态与磁参考门控 |
| 20–21 | EEPROM 双副本、分页写、回读验证与掉电一致性 |
| 22–24 | 跨寄存器变化、滤波状态重建、候选峰与有效心跳分离 |
| 25–28 | 多速率队头阻塞、气压配对、异步转换与 ADC 误差来源 |
| 29–31 | 脏区吞吐、DMA 所有权归还、事件与页面对象生命周期 |
| 32–34 | 异步串口、接收背压、重同步与消息提交 |
| 35–38 | 观察开销、时间标记、诊断分辨率与优化实验 |

`讲解稿.md` 包含旁白、时间索引和源码锚点；`中文字幕.srt` 可用于后续剪辑。

## 编辑与重新生成

技术正文与画面配置在 `content.json`，口语旁白在 `口语旁白.txt`，两者按 38 段、每段 4 步对应。修改后运行：

```powershell
python -m pip install --target .tts-deps -r requirements-voice.txt
python -X utf8 scripts/prepare.py
python -X utf8 scripts/neural_voice.py
python -X utf8 scripts/assemble_neural.py
python -X utf8 scripts/assemble.py --neural
npm run lint
node scripts/qa.mjs
```

当前使用 Edge 在线云希神经语音，未配置或使用付费语音 API 密钥。每个段落连续合成，保留语气与句间停顿；字幕来自词级时间戳，动画按段落起点推进。成片播放速度为 1 倍，避免后期压缩节奏。口语稿压缩了重复表述，画面仍保留完整技术解释。

新配音缓存位于 `.neural-cache`，章节音轨位于 `public/voice/neural-scene-*.wav`。旧本地音轨保留，可供对照；`scripts/voice.ps1` 和不带 `--neural` 的生成流程属于旧语音方案。

各段在 `src/scenes` 独立注册。公共版式位于 `src/SceneFrame.tsx`，动态图形位于 `src/Visuals.tsx`。重新生成会覆盖段落组件与时间线，保留公共图形组件。

## 验证范围

- ESLint、TypeScript、38 个章节音轨及字幕边界检查通过。
- 38 个源码锚点核验通过；具体时间线帧数记录在 timeline.json 与 Root.tsx。
- 浏览器检查 152 个讲解阶段的安全区、SVG 文字和正文间距。
- 关键章节静帧已通过 Remotion 渲染并抽查，预览播放可运行。
- 本次未修改固件、未重新构建或上板测试。视频中的推算不能替代实机性能测量。

硬件资料：[ST STM32F411CE](https://www.st.com/en/microcontrollers-microprocessors/stm32f411ce.html)；动画 API：[Remotion Composition](https://www.remotion.dev/docs/composition)。项目行为以源码证据为准。
