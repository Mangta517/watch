import React from 'react';
import {useCurrentFrame, useVideoConfig, interpolate} from 'remotion';

type Props = {kind: string; nodes: string[]; beat: number; accent: string; sceneId: number};
const ink = '#e5f1ff';
const muted = '#7d9bb5';
const other = '#82baff';
const mono = 'Consolas, Microsoft YaHei, monospace';

const Label = ({text, x, y, size = 30, color = ink, anchor = 'middle'}: {text: string; x: number; y: number; size?: number; color?: string; anchor?: 'middle' | 'start'}) => {
    const chunks: string[] = [];
    let remaining = text;
    while (remaining.length > 14)
    {
        let cut = remaining.lastIndexOf(' ', 14);
        if (cut < 6)
        {
            cut = 13;
        }
        chunks.push(remaining.slice(0, cut));
        remaining = remaining.slice(cut).trim();
    }
    chunks.push(remaining);
    return <text x={x} y={y} fill={color} textAnchor={anchor} fontSize={size}>{chunks.map((line, index) => <tspan key={index} x={x} dy={index * (size + 8)}>{line}</tspan>)}</text>;
};

const Wave = ({y, time, filtered, accent}: {y: number; time: number; filtered: boolean; accent: string}) => {
    const points: string[] = [];
    for (let i = 0; i <= 180; i++)
    {
        const phase = i * 0.105 + time * 1.5;
        let value = Math.sin(phase) * 34 + Math.sin(phase * 2 + 0.8) * 12;
        if (!filtered)
        {
            value += Math.sin(phase / 5) * 18 + Math.sin(phase * 9) * 3;
        }
        points.push(`${140 + i * 4.8},${y - value}`);
    }
    return <polyline points={points.join(' ')} fill="none" stroke={accent} strokeWidth={4} />;
};

export const Diagram = ({kind, nodes, beat, accent, sceneId}: Props) => {
    const frame = useCurrentFrame();
    const {fps} = useVideoConfig();
    const time = frame / fps;
    const loop = (time % 6) / 6;
    let graphic: React.ReactNode;
    if (kind === 'critical')
    {
        const step = Math.floor((time % 10) / 2);
        const labels = ['touch 读取旧值', 'sensor 改 PB13', 'touch 写回旧值', '更新被覆盖', '短临界区串行化'];
        const bars = [{x: 245, y: 90, w: 170, label: 'READ'}, {x: 455, y: 190, w: 205, label: 'MODIFY / WRITE'}, {x: 695, y: 90, w: 175, label: 'WRITE'}];
        graphic = <>
            <text x={40} y={132} fill={other} fontSize={30}>touch</text><text x={40} y={232} fill={accent} fontSize={30}>sensor</text>
            <line x1={220} x2={1030} y1={125} y2={125} stroke="#345774" strokeWidth={3}/><line x1={220} x2={1030} y1={225} y2={225} stroke="#345774" strokeWidth={3}/>
            {bars.map((bar, i) => <g key={bar.label} opacity={0.3 + 0.7 * Number(step >= i)}><rect x={bar.x} y={bar.y} width={bar.w} height={67} rx={10} fill="#214a65" stroke={accent}/><text x={bar.x + bar.w / 2} y={bar.y + 42} fill={ink} textAnchor="middle" fontSize={22}>{bar.label}</text></g>)}
            <path d="M415 125 L455 225 M660 225 L695 125" fill="none" stroke="#ffbf77" strokeWidth={3} strokeDasharray="7 5"/>
            <rect x={220} y={300} width={810} height={66} rx={12} fill="#243b4a" stroke={accent}/>
            <text x={625} y={343} textAnchor="middle" fill={ink} fontSize={30}>{labels[step]}</text>
            <text x={220} y={418} fill={muted} fontSize={25}>GPIOB.MODER：不同引脚，同一读改写对象</text>
        </>;
    }
    else if (kind === 'cost')
    {
        const names = ['CPU 执行', '内部等待', '被抢占', '周期等待'];
        const colors = [accent, '#82baff', '#ffbf77', '#637e96'];
        const widths = [280, 185, 155, 285];
        let start = 90;
        graphic = <>
            <text x={90} y={68} fill={muted} fontSize={27}>把“耗时”拆成可归因的部分</text>
            {names.map((name, index) => {
                const x = start;
                start += widths[index];
                return <g key={name}><rect x={x} y={140} width={widths[index] - 6} height={95} rx={9} fill={colors[index]} opacity={0.3 + 0.65 * Number(loop * 4 >= index)}/><text x={x + widths[index] / 2} y={275} textAnchor="middle" fill={ink} fontSize={27}>{name}</text></g>;
            })}
            <path d="M90 305 L90 325 L710 325 L710 305 M90 350 L90 370 L995 370 L995 350" fill="none" stroke={other} strokeWidth={2}/>
            <text x={400} y={351} textAnchor="middle" fill={other} fontSize={25}>业务墙钟：含等待和抢占</text><text x={800} y={410} textAnchor="middle" fill={muted} fontSize={23}>整轮间隔 / 示意比例</text>
            <line x1={90 + loop * 905} x2={90 + loop * 905} y1={105} y2={240} stroke="white" strokeWidth={2}/>
        </>;
    }
    else if (kind === 'age')
    {
        const positions = [125, 375, 655, 975];
        const labels = ['器件转换', '任务读取', '快照 / 排队', '消费 / 显示'];
        graphic = <>
            <text x={70} y={68} fill={muted} fontSize={27}>沿数据链路保留时间与有效性</text>
            <line x1={125} x2={975} y1={185} y2={185} stroke="#345774" strokeWidth={4}/>
            {positions.map((x, i) => <g key={x}><circle cx={x} cy={185} r={22} fill={accent} opacity={0.25 + 0.75 * Number(loop * 4 >= i)}/><text x={x} y={245} textAnchor="middle" fill={ink} fontSize={28}>{labels[i]}</text><text x={x} y={140} textAnchor="middle" fill={other} fontSize={25}>t{i}</text></g>)}
            <path d="M375 285 L375 310 L975 310 L975 285" fill="none" stroke={accent} strokeWidth={3}/>
            <text x={675} y={355} textAnchor="middle" fill={accent} fontSize={31}>消费端看到的年龄：t₃ − t₁</text>
            <text x={550} y={418} textAnchor="middle" fill={muted} fontSize={24}>t₁ 是板端读取标记，不能替代 t₀；跨设备时钟需对齐</text>
        </>;
    }
    else if (kind === 'ownership')
    {
        const labels = ['生产者', '在途 / 共享状态', '消费者'];
        graphic = <>
            {labels.map((label, i) => <g key={label}><rect x={70 + i * 345} y={120} width={270} height={150} rx={22} fill="#19364b" stroke={accent} strokeWidth={2}/><text x={205 + i * 345} y={205} textAnchor="middle" fill={ink} fontSize={31}>{label}</text></g>)}
            <path d="M345 192 L405 192 M690 192 L750 192" stroke={accent} strokeWidth={4} markerEnd="url(#arrow)"/>
            <circle cx={190 + loop * 700} cy={310} r={11} fill={accent}/>
            <text x={205} y={365} textAnchor="middle" fill={other} fontSize={27}>构建 / 发布</text><text x={550} y={365} textAnchor="middle" fill={other} fontSize={27}>生命周期 / 短保护区</text><text x={895} y={365} textAnchor="middle" fill={other} fontSize={27}>复制 / 完成归还</text>
            <text x={550} y={425} textAnchor="middle" fill={muted} fontSize={24}>每次交接明确：谁能写、何时能复用、谁收取结果</text>
        </>;
    }
    else if (kind === 'priority')
    {
        graphic = <>
            <text x={70} y={65} fill={muted} fontSize={28}>最终配置由调用顺序决定</text>
            <rect x={70} y={120} width={380} height={130} rx={18} fill="#19364b" stroke={other}/><text x={260} y={170} fill={ink} textAnchor="middle" fontSize={30}>MX_DMA_Init</text><text x={260} y={220} fill="#ffbf77" textAnchor="middle" fontSize={38}>IRQ priority = 0</text>
            <path d="M470 185 L610 185" stroke={accent} strokeWidth={5} markerEnd="url(#arrow)"/>
            <rect x={640} y={120} width={380} height={130} rx={18} fill="#19364b" stroke={accent}/><text x={830} y={170} fill={ink} textAnchor="middle" fontSize={30}>LCD_Init</text><text x={830} y={220} fill={accent} textAnchor="middle" fontSize={38}>IRQ priority = 12</text>
            <text x={550} y={330} textAnchor="middle" fill={ink} fontSize={32}>DMA 完成 → FromISR 通知 → LVGL 唤醒</text>
            <text x={550} y={405} textAnchor="middle" fill={muted} fontSize={24}>USART1 仍为 0；当前回调不调用内核中断 API</text>
        </>;
    }
    else if (kind === 'i2c')
    {
        const scl = ['70,100', '140,100', '140,150'];
        const sda = ['70,220', '110,220', '110,270', '140,270'];
        const bits = [1, 1, 0, 1, 0, 0, 0, 0, 0];
        bits.forEach((bit, index) => {
            const x = 140 + index * 84;
            scl.push(`${x + 20},150`, `${x + 20},100`, `${x + 60},100`, `${x + 60},150`, `${x + 84},150`);
            let y = 270;
            if (bit === 1)
            {
                y = 220;
            }
            sda.push(`${x},${y}`, `${x + 84},${y}`);
        });
        scl.push('930,150', '930,100', '1020,100');
        sda.push('960,270', '960,220', '1020,220');
        graphic = <>
            <text x={28} y={130} fill={other} fontSize={27}>SCL</text><text x={28} y={250} fill={accent} fontSize={27}>SDA</text>
            <polyline points={scl.join(' ')} fill="none" stroke={other} strokeWidth={4}/>
            <polyline points={sda.join(' ')} fill="none" stroke={accent} strokeWidth={4}/>
            {bits.map((bit, i) => <text key={i} x={178 + i * 84} y={195} textAnchor="middle" fill={ink} fontSize={28}>{bit}</text>)}
            <text x={110} y={310} fill={muted} fontSize={24} textAnchor="middle">START</text><text x={855} y={310} fill={muted} fontSize={24}>ACK</text><text x={970} y={310} fill={muted} fontSize={24}>STOP</text>
            <line x1={100 + loop * 910} x2={100 + loop * 910} y1={60} y2={325} stroke="white" strokeOpacity={0.5} strokeWidth={2} strokeDasharray="7 7" />
            <text x={550} y={380} textAnchor="middle" fill={ink} fontSize={30}>单字节示意：0xD0 + ACK</text>
        </>;
    }
    else if (kind === 'wave')
    {
        graphic = <>
            {[80, 145, 210, 275, 340].map(y => <line key={y} x1={130} x2={1010} y1={y} y2={y} stroke="#253f54" />)}
            <text x={28} y={145} fill={muted} fontSize={25}>RAW</text><Wave y={140} time={time} filtered={false} accent={other}/>
            <text x={28} y={290} fill={muted} fontSize={25}>AC</text><Wave y={285} time={time} filtered accent={accent}/>
            <line x1={140} x2={1005} y1={260} y2={260} stroke="#ffbf77" strokeDasharray="9 9" strokeWidth={2}/>
            <line x1={140 + loop * 864} x2={140 + loop * 864} y1={70} y2={345} stroke="white" strokeOpacity={0.4}/>
            <text x={140} y={405} fill={ink} fontSize={28}>背景漂移</text><text x={430} y={405} fill={accent} fontSize={28}>脉搏交流分量</text><text x={785} y={405} fill="#ffbf77" fontSize={28}>候选门限</text>
            <text x={1000} y={40} fill={muted} fontSize={19} textAnchor="end">合成示意 · 非实测</text>
        </>;
    }
    else if (kind === 'quaternion' && sceneId === 19)
    {
        const angle = time * 0.65;
        graphic = <>
            <text x={285} y={65} fill={other} fontSize={30} textAnchor="middle">芯片坐标：偏置 + 轴向缩放</text>
            <line x1={95} x2={485} y1={230} y2={230} stroke="#345774"/><line x1={260} x2={260} y1={80} y2={390} stroke="#345774"/>
            <ellipse cx={300} cy={215} rx={155} ry={95} fill="#163249" stroke={other} strokeWidth={4}/>
            <circle cx={300 + Math.cos(angle) * 155} cy={215 + Math.sin(angle) * 95} r={9} fill={accent}/>
            <circle cx={300} cy={215} r={5} fill="#ffbf77"/>
            <path d="M495 230 L600 230" stroke={accent} strokeWidth={4} markerEnd="url(#arrow)"/>
            <circle cx={800} cy={230} r={125} fill="#163249" stroke={accent} strokeWidth={4}/>
            <line x1={635} x2={965} y1={230} y2={230} stroke="#345774"/><line x1={800} x2={800} y1={65} y2={395} stroke="#345774"/>
            <circle cx={800 + Math.cos(angle) * 125} cy={230 + Math.sin(angle) * 125} r={9} fill={accent}/>
            <text x={800} y={65} fill={accent} fontSize={30} textAnchor="middle">去偏置 / 比例校正</text>
            <text x={550} y={425} fill={muted} fontSize={25} textAnchor="middle">对角校正模型的二维截面示意；不包含交叉轴项</text>
        </>;
    }
    else if (kind === 'quaternion')
    {
        const angle = time * 0.35;
        const x = 430 + Math.cos(angle) * 175;
        const y = 230 + Math.sin(angle) * 80;
        let measured = '测量向量 m';
        let reference = '水平投影 mₕ';
        if (sceneId === 17)
        {
            measured = '实测方向 a';
            reference = '预测重力 ĝ';
        }
        graphic = <>
            <ellipse cx={430} cy={230} rx={240} ry={90} fill="#21425c33" stroke="#345774" strokeWidth={2}/>
            <ellipse cx={430} cy={230} rx={240} ry={90} fill="none" stroke="#345774" strokeWidth={2} transform="rotate(65 430 230)"/>
            <ellipse cx={430} cy={230} rx={240} ry={90} fill="none" stroke="#345774" strokeWidth={2} transform="rotate(-65 430 230)"/>
            <path d="M430 230 L430 55 M430 230 L685 275 M430 230 L205 330" stroke={muted} strokeWidth={3} fill="none"/>
            <text x={430} y={40} fill={ink} fontSize={28}>Z</text><text x={700} y={290} fill={ink} fontSize={28}>X</text><text x={175} y={350} fill={ink} fontSize={28}>Y</text>
            <line x1={430} y1={230} x2={x} y2={y - 85} stroke={accent} strokeWidth={7} markerEnd="url(#arrow)"/>
            <line x1={430} y1={230} x2={x} y2={y} stroke={other} strokeWidth={5} markerEnd="url(#arrow)"/>
            <line x1={x} y1={y - 85} x2={x} y2={y} stroke={accent} strokeWidth={2} strokeDasharray="6 6"/>
            <circle cx={430} cy={230} r={10} fill={ink}/>
            <text x={770} y={110} fill={accent} fontSize={32}>{measured}</text><text x={770} y={175} fill={other} fontSize={32}>{reference}</text>
            <text x={770} y={270} fill={ink} fontSize={30}>q = [q₀, q₁, q₂, q₃]</text><text x={770} y={325} fill={muted} fontSize={28}>积分 / 纠偏 / 归一化</text>
            <text x={1000} y={408} textAnchor="end" fill={muted} fontSize={21}>向量与旋转机制示意</text>
        </>;
    }
    else if (kind === 'memory')
    {
        graphic = <>{nodes.map((label, i) => {
            let fill = '#142a3d';
            let color = muted;
            if (i === beat)
            {
                fill = '#24516a';
                color = ink;
            }
            return <g key={label}>
                <rect x={80} y={50 + i * 87} width={850} height={67} rx={8} fill={fill} stroke={color}/>
                <text x={110} y={94 + i * 87} fill={color} fontSize={31}>{label}</text>
                <rect x={954} y={62 + i * 87} width={40} height={43} rx={5} fill={accent} opacity={0.2 + Number(i === beat) * 0.8}/>
            </g>;
        })}<text x={80} y={427} fill={muted} fontSize={22}>区域 / 操作示意，长度不按比例</text></>;
    }
    else if (kind === 'timeline')
    {
        graphic = <>{nodes.map((label, i) => {
            let color = other;
            if (i === beat)
            {
                color = accent;
            }
            return <g key={label}>
                <Label text={label} x={35} y={72 + i * 91} size={26} anchor="start"/>
                <line x1={420} x2={1030} y1={72 + i * 91} y2={72 + i * 91} stroke="#32506a" strokeWidth={3}/>
                {[0, 1, 2, 3, 4].map(j => <rect key={j} x={430 + j * 116 + (i % 2) * 18} y={49 + i * 91} width={55 + (i % 3) * 14} height={46} rx={6} fill={color} opacity={0.25 + Number(loop * 5 > j) * 0.55}/>) }
            </g>;
        })}<line x1={425 + loop * 610} x2={425 + loop * 610} y1={25} y2={386} stroke="white" strokeWidth={2}/><text x={430} y={425} fill={muted} fontSize={22}>推进与等待示意 · 非等比例实测时间轴</text></>;
    }
    else if (kind === 'dma')
    {
        const cycle = time % 4;
        let first = accent;
        let second = '#29465e';
        if (cycle >= 2)
        {
            first = '#29465e';
            second = accent;
        }
        graphic = <>
            <rect x={65} y={145} width={180} height={130} rx={20} fill="#19364e" stroke={other}/><Label text="CPU / LVGL" x={155} y={215} size={30}/>
            <rect x={360} y={75} width={220} height={110} rx={14} fill="#19364e" stroke={first} strokeWidth={4}/><Label text="BUFFER A" x={470} y={138} size={30}/>
            <rect x={360} y={255} width={220} height={110} rx={14} fill="#19364e" stroke={second} strokeWidth={4}/><Label text="BUFFER B" x={470} y={318} size={30}/>
            <path d="M245 185 L300 185 L300 130 L350 130 M245 235 L300 235 L300 310 L350 310 M580 130 L690 130 L690 210 L790 210 M580 310 L690 310 L690 210" stroke={accent} fill="none" strokeWidth={3}/>
            <rect x={805} y={65} width={200} height={300} rx={25} fill="#07121e" stroke={other} strokeWidth={3}/>
            {Array.from({length: 10}, (_, i) => <rect key={i} x={822} y={85 + i * 25} width={166} height={19} fill={accent} opacity={0.1 + Number(loop * 10 > i) * 0.8}/>)}
            <circle cx={590 + ((time % 2) / 2) * 200} cy={210} r={8} fill={accent}/>
            <text x={640} y={190} fill={ink} fontSize={29}>DMA</text><text x={905} y={405} textAnchor="middle" fill={ink} fontSize={27}>ST7789</text>
            <text x={65} y={420} fill={muted} fontSize={24}>完成中断 → flush_ready → 任务继续</text>
        </>;
    }
    else if (kind === 'uart')
    {
        graphic = <>
            <text x={65} y={60} fill={muted} fontSize={27}>字节到达 / 字节流分段示意</text>
            {Array.from({length: 12}, (_, i) => {
                let fill = '#163249';
                if (i <= Math.floor(loop * 12))
                {
                    fill = accent;
                }
                return <g key={i}><rect x={65 + i * 80} y={105} width={66} height={65} rx={7} fill={fill}/><text x={98 + i * 80} y={147} textAnchor="middle" fill="#07121e" fontSize={26} fontFamily={mono}>{['48', '52', '53', '31', '00', '01', '02', '03', '00', '00', '80', '7F'][i]}</text></g>;
            })}
            <path d="M100 205 L340 205 M395 205 L635 205 M690 205 L1010 205" stroke={other} strokeWidth={5}/>
            <text x={220} y={245} fill={muted} textAnchor="middle" fontSize={26}>片段 1</text><text x={515} y={245} fill={muted} textAnchor="middle" fontSize={26}>片段 2</text><text x={850} y={245} fill={muted} textAnchor="middle" fontSize={26}>片段 3</text>
            {nodes.map((label, i) => <Label key={label} text={label} x={160 + i * 260} y={340} size={25} color={ink}/>)}
            <text x={65} y={425} fill={muted} fontSize={21}>字节值为结构示意，不作为有效协议包</text>
        </>;
    }
    else if (kind === 'bus')
    {
        graphic = <>
            <rect x={410} y={140} width={280} height={170} rx={25} fill="#1b3b53" stroke={accent} strokeWidth={3}/>
            <Label text="STM32F411" x={550} y={215} size={36}/><Label text="GPIO / UART" x={550} y={265} size={26} color={muted}/>
            {nodes.map((label, i) => {
                let x = 195;
                if (i % 2 === 1)
                {
                    x = 905;
                }
                const y = 95 + Math.floor(i / 2) * 250;
                return <g key={label}>
                    <path d={`M${x} ${y} L550 ${y} L550 140`} fill="none" stroke={other} strokeWidth={3}/>
                    <rect x={x - 165} y={y - 48} width={330} height={100} rx={15} fill="#152d42" stroke={accent}/>
                    <Label text={label} x={x} y={y - 5} size={27}/>
                    <circle cx={x + (550 - x) * loop} cy={y} r={7} fill={accent}/>
                </g>;
            })}
        </>;
    }
    else if (kind === 'watch')
    {
        graphic = <>
            <rect x={305} y={-80} width={210} height={580} rx={70} fill="#253d51"/>
            <rect x={242} y={45} width={336} height={360} rx={64} fill="#050c14" stroke="#7895ac" strokeWidth={7}/>
            <rect x={267} y={70} width={286} height={310} rx={44} fill="#0d2033"/>
            <text x={410} y={126} fill={accent} fontSize={22} textAnchor="middle">F411 SMARTWATCH</text>
            <text x={410} y={210} fill={ink} fontSize={70} textAnchor="middle" fontFamily={mono}>10:15</text>
            <path d={`M290 290 Q330 ${270 + 15 * Math.sin(time)} 350 290 T410 290 T470 290 T530 290`} stroke={accent} fill="none" strokeWidth={5}/>
            <text x={410} y={346} fill={muted} fontSize={24} textAnchor="middle">架构示意 / 非界面截图</text>
            {nodes.map((label, i) => <g key={label}><circle cx={710} cy={100 + i * 84} r={8} fill={accent}/><Label text={label} x={740} y={110 + i * 84} size={34} anchor="start"/></g>)}
        </>;
    }
    else
    {
        graphic = <>
            <line x1={130} x2={960} y1={190} y2={190} stroke="#315570" strokeWidth={5}/>
            {nodes.map((label, i) => {
                let border = '#385b76';
                if (i === beat)
                {
                    border = accent;
                }
                return <g key={label}>
                    <rect x={32 + i * 270} y={120} width={230} height={150} rx={20} fill="#19364b" stroke={border} strokeWidth={3}/>
                    <text x={147 + i * 270} y={165} textAnchor="middle" fill={border} fontSize={28} fontFamily={mono}>0{i + 1}</text>
                    <Label text={label} x={147 + i * 270} y={210} size={28}/>
                </g>;
            })}
            <circle cx={100 + loop * 900} cy={320} r={9} fill={accent}/>
            <line x1={100} y1={320} x2={1000} y2={320} stroke={accent} strokeOpacity={0.25} strokeWidth={2}/>
            <text x={550} y={395} textAnchor="middle" fill={muted} fontSize={27}>沿调用顺序与数据依赖逐层推进</text>
        </>;
    }
    return <svg width="1100" height="455" viewBox="0 0 1100 455" style={{fontFamily: 'Microsoft YaHei, Segoe UI, sans-serif', opacity: interpolate(frame, [0, fps * 0.4], [0.75, 1], {extrapolateRight: 'clamp'})}}>
        <defs><marker id="arrow" markerWidth="7" markerHeight="7" refX="5" refY="3" orient="auto"><path d="M0,0 L0,6 L6,3 z" fill={accent}/></marker></defs>
        {graphic}
        <text x={1070} y={444} textAnchor="end" fill="#426078" fontSize={15}>{String(sceneId).padStart(2, '0')}</text>
    </svg>;
};
