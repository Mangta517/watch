import {bundle} from '@remotion/bundler';
import {getCompositions, renderStill, openBrowser} from '@remotion/renderer';
import path from 'node:path';

const browserExecutable = 'C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe';
const serveUrl = await bundle({entryPoint: path.resolve('src/index.ts'), outDir: path.resolve('qa/bundle')});
const browser = await openBrowser('chrome', {browserExecutable});
const compositions = await getCompositions(serveUrl, {puppeteerInstance: browser});
for (const chapter of [1, 5, 10, 13, 15, 21, 24, 29, 36, 38])
{
    const id = `Chapter-${String(chapter).padStart(2, '0')}`;
    const composition = compositions.find(item => item.id === id);
    await renderStill({serveUrl, composition, frame: 160, output: `qa/chapter-${String(chapter).padStart(2, '0')}.png`,
        scale: 0.5, puppeteerInstance: browser});
    console.log(`Verified still: ${id}`);
}
await browser.close({silent: true});
