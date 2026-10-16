import {Composition, Folder, Series, useVideoConfig} from 'remotion';
import {Scene01} from './scenes/Scene01';
import {Scene02} from './scenes/Scene02';
import {Scene03} from './scenes/Scene03';
import {Scene04} from './scenes/Scene04';
import {Scene05} from './scenes/Scene05';
import {Scene06} from './scenes/Scene06';
import {Scene07} from './scenes/Scene07';
import {Scene08} from './scenes/Scene08';
import {Scene09} from './scenes/Scene09';
import {Scene10} from './scenes/Scene10';
import {Scene11} from './scenes/Scene11';
import {Scene12} from './scenes/Scene12';
import {Scene13} from './scenes/Scene13';
import {Scene14} from './scenes/Scene14';
import {Scene15} from './scenes/Scene15';
import {Scene16} from './scenes/Scene16';
import {Scene17} from './scenes/Scene17';
import {Scene18} from './scenes/Scene18';
import {Scene19} from './scenes/Scene19';
import {Scene20} from './scenes/Scene20';
import {Scene21} from './scenes/Scene21';
import {Scene22} from './scenes/Scene22';
import {Scene23} from './scenes/Scene23';
import {Scene24} from './scenes/Scene24';
import {Scene25} from './scenes/Scene25';
import {Scene26} from './scenes/Scene26';
import {Scene27} from './scenes/Scene27';
import {Scene28} from './scenes/Scene28';
import {Scene29} from './scenes/Scene29';
import {Scene30} from './scenes/Scene30';
import {Scene31} from './scenes/Scene31';
import {Scene32} from './scenes/Scene32';
import {Scene33} from './scenes/Scene33';
import {Scene34} from './scenes/Scene34';
import {Scene35} from './scenes/Scene35';
import {Scene36} from './scenes/Scene36';
import {Scene37} from './scenes/Scene37';
import {Scene38} from './scenes/Scene38';

export const FullVideo = () => {
    const {fps} = useVideoConfig();
    return (
        <Series>
            <Series.Sequence name="01 · 先建立工程的性能模型" durationInFrames={845} premountFor={fps}>
                <Scene01 />
            </Series.Sequence>
            <Series.Sequence name="02 · 先划清执行权与数据所有权" durationInFrames={925} premountFor={fps}>
                <Scene02 />
            </Series.Sequence>
            <Series.Sequence name="03 · 独立总线仍共享调度资源" durationInFrames={929} premountFor={fps}>
                <Scene03 />
            </Series.Sequence>
            <Series.Sequence name="04 · 启动优化看关键路径" durationInFrames={1001} premountFor={fps}>
                <Scene04 />
            </Series.Sequence>
            <Series.Sequence name="05 · 配置最终值比生成值更重要" durationInFrames={936} premountFor={fps}>
                <Scene05 />
            </Series.Sequence>
            <Series.Sequence name="06 · 状态机优化的是等待占用" durationInFrames={998} premountFor={fps}>
                <Scene06 />
            </Series.Sequence>
            <Series.Sequence name="07 · 周期预算与真实截止时间" durationInFrames={911} premountFor={fps}>
                <Scene07 />
            </Series.Sequence>
            <Series.Sequence name="08 · 忙等、阻塞与优先级反转" durationInFrames={970} premountFor={fps}>
                <Scene08 />
            </Series.Sequence>
            <Series.Sequence name="09 · 内存预算决定刷新策略" durationInFrames={966} premountFor={fps}>
                <Scene09 />
            </Series.Sequence>
            <Series.Sequence name="10 · 快照一致，不等于同时采样" durationInFrames={909} premountFor={fps}>
                <Scene10 />
            </Series.Sequence>
            <Series.Sequence name="11 · 软件总线的成本怎么拆" durationInFrames={920} premountFor={fps}>
                <Scene11 />
            </Series.Sequence>
            <Series.Sequence name="12 · 合并事务的收益与边界" durationInFrames={901} premountFor={fps}>
                <Scene12 />
            </Series.Sequence>
            <Series.Sequence name="13 · 临界区要包住真正的竞争" durationInFrames={964} premountFor={fps}>
                <Scene13 />
            </Series.Sequence>
            <Series.Sequence name="14 · 读成功不一定是新样本" durationInFrames={997} premountFor={fps}>
                <Scene14 />
            </Series.Sequence>
            <Series.Sequence name="15 · dt 的语义会进入姿态误差" durationInFrames={887} premountFor={fps}>
                <Scene15 />
            </Series.Sequence>
            <Series.Sequence name="16 · 校准判定必须可解释" durationInFrames={935} premountFor={fps}>
                <Scene16 />
            </Series.Sequence>
            <Series.Sequence name="17 · 融合门控与反馈状态" durationInFrames={955} premountFor={fps}>
                <Scene17 />
            </Series.Sequence>
            <Series.Sequence name="18 · 磁参考是带年龄的约束" durationInFrames={908} premountFor={fps}>
                <Scene18 />
            </Series.Sequence>
            <Series.Sequence name="19 · 校准模型决定可修正的误差" durationInFrames={927} premountFor={fps}>
                <Scene19 />
            </Series.Sequence>
            <Series.Sequence name="20 · 持久化的有效性是多重条件" durationInFrames={949} premountFor={fps}>
                <Scene20 />
            </Series.Sequence>
            <Series.Sequence name="21 · 掉电一致性落在提交顺序" durationInFrames={930} premountFor={fps}>
                <Scene21 />
            </Series.Sequence>
            <Series.Sequence name="22 · 跨寄存器读取也会撕裂" durationInFrames={975} premountFor={fps}>
                <Scene22 />
            </Series.Sequence>
            <Series.Sequence name="23 · 滤波器时间常数与重建策略" durationInFrames={993} premountFor={fps}>
                <Scene23 />
            </Series.Sequence>
            <Series.Sequence name="24 · 候选峰与有效心跳要分开" durationInFrames={937} premountFor={fps}>
                <Scene24 />
            </Series.Sequence>
            <Series.Sequence name="25 · 单任务多速率的队头阻塞" durationInFrames={931} premountFor={fps}>
                <Scene25 />
            </Series.Sequence>
            <Series.Sequence name="26 · 气压合并读取保留配对语义" durationInFrames={872} premountFor={fps}>
                <Scene26 />
            </Series.Sequence>
            <Series.Sequence name="27 · 异步测量是请求到完成的链路" durationInFrames={944} premountFor={fps}>
                <Scene27 />
            </Series.Sequence>
            <Series.Sequence name="28 · ADC 链路先分清误差来源" durationInFrames={939} premountFor={fps}>
                <Scene28 />
            </Series.Sequence>
            <Series.Sequence name="29 · 刷屏瓶颈要拆成两个阶段" durationInFrames={975} premountFor={fps}>
                <Scene29 />
            </Series.Sequence>
            <Series.Sequence name="30 · DMA 完成也是所有权归还" durationInFrames={1005} premountFor={fps}>
                <Scene30 />
            </Series.Sequence>
            <Series.Sequence name="31 · 界面重建要避开事件生命周期" durationInFrames={932} premountFor={fps}>
                <Scene31 />
            </Series.Sequence>
            <Series.Sequence name="32 · 异步串口把线时移出忙等" durationInFrames={984} premountFor={fps}>
                <Scene32 />
            </Series.Sequence>
            <Series.Sequence name="33 · 接收链路的背压与重同步" durationInFrames={924} premountFor={fps}>
                <Scene33 />
            </Series.Sequence>
            <Series.Sequence name="34 · 共享接收入口，独立消息事务" durationInFrames={965} premountFor={fps}>
                <Scene34 />
            </Series.Sequence>
            <Series.Sequence name="35 · 观察系统也会改变系统" durationInFrames={938} premountFor={fps}>
                <Scene35 />
            </Series.Sequence>
            <Series.Sequence name="36 · 数据时间与到达时间分开" durationInFrames={948} premountFor={fps}>
                <Scene36 />
            </Series.Sequence>
            <Series.Sequence name="37 · 诊断分辨率与归因边界" durationInFrames={977} premountFor={fps}>
                <Scene37 />
            </Series.Sequence>
            <Series.Sequence name="38 · 把优化变成可复现的实验" durationInFrames={979} premountFor={fps}>
                <Scene38 />
            </Series.Sequence>
        </Series>
    );
};

export const RemotionRoot = () => (
    <>
        <Composition id="F411-Deep-Dive" component={FullVideo} durationInFrames={35881} fps={30} width={1920} height={1080} />
        <Folder name="Chapters">
                <Composition id="Chapter-01" component={Scene01} durationInFrames={845} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-02" component={Scene02} durationInFrames={925} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-03" component={Scene03} durationInFrames={929} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-04" component={Scene04} durationInFrames={1001} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-05" component={Scene05} durationInFrames={936} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-06" component={Scene06} durationInFrames={998} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-07" component={Scene07} durationInFrames={911} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-08" component={Scene08} durationInFrames={970} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-09" component={Scene09} durationInFrames={966} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-10" component={Scene10} durationInFrames={909} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-11" component={Scene11} durationInFrames={920} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-12" component={Scene12} durationInFrames={901} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-13" component={Scene13} durationInFrames={964} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-14" component={Scene14} durationInFrames={997} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-15" component={Scene15} durationInFrames={887} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-16" component={Scene16} durationInFrames={935} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-17" component={Scene17} durationInFrames={955} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-18" component={Scene18} durationInFrames={908} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-19" component={Scene19} durationInFrames={927} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-20" component={Scene20} durationInFrames={949} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-21" component={Scene21} durationInFrames={930} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-22" component={Scene22} durationInFrames={975} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-23" component={Scene23} durationInFrames={993} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-24" component={Scene24} durationInFrames={937} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-25" component={Scene25} durationInFrames={931} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-26" component={Scene26} durationInFrames={872} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-27" component={Scene27} durationInFrames={944} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-28" component={Scene28} durationInFrames={939} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-29" component={Scene29} durationInFrames={975} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-30" component={Scene30} durationInFrames={1005} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-31" component={Scene31} durationInFrames={932} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-32" component={Scene32} durationInFrames={984} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-33" component={Scene33} durationInFrames={924} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-34" component={Scene34} durationInFrames={965} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-35" component={Scene35} durationInFrames={938} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-36" component={Scene36} durationInFrames={948} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-37" component={Scene37} durationInFrames={977} fps={30} width={1920} height={1080} />
                <Composition id="Chapter-38" component={Scene38} durationInFrames={979} fps={30} width={1920} height={1080} />
        </Folder>
    </>
);
