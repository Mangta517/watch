import fs from 'node:fs';
import assert from 'node:assert/strict';
import path from 'node:path';

const root = process.cwd();
const timeline = JSON.parse(fs.readFileSync('timeline.json', 'utf8'));
let previous_end = 0;
for (const scene of timeline)
{
    assert.equal(scene.startFrame, previous_end);
    assert.equal(scene.beats.length, 4);
    assert.ok(fs.statSync(path.join(root, 'public', scene.audio)).size > 44);
    for (const caption of scene.captions)
    {
        assert.ok(caption.startMs >= 0 && caption.endMs > caption.startMs);
        assert.ok(caption.endMs <= scene.frames / 30 * 1000);
    }
    previous_end += scene.frames;
}
assert.ok(previous_end >= 27000 && previous_end <= 36000);
const checkpoints = timeline.flatMap(scene => scene.beats.map((beat, index) => ({
    id: scene.id,
    beat: index,
    frame: scene.startFrame + Math.round((beat + 0.65) * 30),
})));
const browser_check = `async (page) => {
    const results = await page.evaluate(async (checkpoints) => {
        const issues = [];
        for (const checkpoint of checkpoints)
        {
            window.remotion_setFrame(checkpoint.frame, 'F411-Deep-Dive');
            await new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve)));
            const root = document.querySelector('[data-scene="' + checkpoint.id + '"]');
            if (!root)
            {
                issues.push({...checkpoint, issue: 'scene missing'});
                continue;
            }
            const bounds = root.getBoundingClientRect();
            const scale = bounds.width / 1920;
            for (const element of root.querySelectorAll('[data-role]'))
            {
                const rect = element.getBoundingClientRect();
                if (rect.right > bounds.right - 50 * scale || rect.bottom > bounds.bottom - 50 * scale)
                {
                    issues.push({...checkpoint, issue: 'outside safe area', role: element.dataset.role});
                }
            }
            const side = root.querySelector('[data-role="explanation"]').getBoundingClientRect();
            const formula = root.querySelector('[data-role="formula"]').getBoundingClientRect();
            if (side.bottom > formula.top - 12 * scale)
            {
                issues.push({...checkpoint, issue: 'explanation collides with formula', bottom: (side.bottom - bounds.top) / scale});
            }
            const body = root.querySelector('[data-role="body"]').getBoundingClientRect();
            const source = root.querySelector('[data-role="source"]').getBoundingClientRect();
            if (body.bottom > source.top - 8 * scale)
            {
                issues.push({...checkpoint, issue: 'body collides with source'});
            }
            for (const element of root.querySelectorAll('svg text'))
            {
                const box = element.getBBox();
                if (box.x < -2 || box.x + box.width > 1102 || box.y + box.height > 457)
                {
                    issues.push({...checkpoint, issue: 'SVG text clipped', text: element.textContent});
                }
            }
        }
        window.remotion_setFrame(150, 'F411-Deep-Dive');
        return {checkpoints: checkpoints.length, issues};
    }, ${JSON.stringify(checkpoints)});
    await page.evaluate(result => {
        document.documentElement.dataset.qaResult = JSON.stringify(result);
    }, results);
    return results;
}`;
fs.writeFileSync('qa/browser-check.js', browser_check);
console.log(`Verified ${timeline.length} audio files, captions, contiguous timeline: ${previous_end} frames.`);
