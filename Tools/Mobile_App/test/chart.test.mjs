/* chart.test.mjs - 波形量程纯函数单测
 *
 *   node Tools/Mobile_App/test/chart.test.mjs
 *
 * 重点不是「值对不对」本身，而是：峰值小幅波动时量程必须纹丝不动，
 * 否则图每 100 ms 缩一次，看着像车在抖，其实是画布在抖。
 */
import { readFileSync, writeFileSync, mkdtempSync } from 'node:fs'
import { tmpdir } from 'node:os'
import { join, dirname } from 'node:path'
import { fileURLToPath, pathToFileURL } from 'node:url'

const here = dirname(fileURLToPath(import.meta.url))
const src = readFileSync(join(here, '..', 'common', 'chart.js'), 'utf8')
const dir = mkdtempSync(join(tmpdir(), 'bcchart-'))
const mod = join(dir, 'chart.mjs')
writeFileSync(mod, src)
const { pickRange, stepRange } = await import(pathToFileURL(mod).href)

let fails = 0
let checks = 0
function eq(label, got, want) {
  checks++
  if (JSON.stringify(got) !== JSON.stringify(want)) {
    fails++
    console.log('  [FAIL]', label, '\n         got ', JSON.stringify(got),
                '\n         want', JSON.stringify(want))
  } else {
    console.log('  [OK]  ', label)
  }
}

console.log('--- 峰值 -> 量程（档位 1/2/3/5/10）---')
eq('0 度（没数据）给最小量程 1', pickRange(0), 1)
eq('0.4° 抬到 1', pickRange(0.4), 1)
eq('噪声 0.05° 也不小于 1', pickRange(0.05), 1)
eq('1.5° -> 2', pickRange(1.5), 2)
eq('2.4° -> 3', pickRange(2.4), 3)
eq('4.2° -> 5', pickRange(4.2), 5)
eq('7° -> 10', pickRange(7), 10)
eq('12° -> 20', pickRange(12), 20)
eq('45° -> 50', pickRange(45), 50)
eq('120° -> 200（保护停机时也不出框）', pickRange(120), 200)

console.log('--- 迟滞：同一量程内小幅波动不能改量程 ---')
eq('首次 cur=0 直接取 want', stepRange(0, 3), 5)
eq('cur=10 峰值 3° 保持 10', stepRange(10, 3), 10)
eq('cur=10 峰值 6° 保持 10', stepRange(10, 6), 10)
eq('cur=10 峰值 9° 顶出去 -> 20', stepRange(10, 9), 20)
eq('cur=10 峰值 0.5° 缩到 1', stepRange(10, 0.5), 1)
eq('cur=20 峰值 2° 缩到 3', stepRange(20, 2), 3)

console.log('--- 抖动用例：峰值在 2.0 附近来回，量程只许变一次 ---')
{
  const seq = [1.8, 2.05, 1.95, 2.1, 1.9, 2.2, 1.85, 2.0]
  let cur = 0
  const seen = []
  for (const p of seq) {
    cur = stepRange(cur, p)
    seen.push(cur)
  }
  const uniq = [...new Set(seen)]
  eq('整段只有一种量程', uniq.length, 1)
}

console.log('--- 掉到零再回来，不能夹在中间不动 ---')
{
  let cur = 20
  cur = stepRange(cur, 20)
  eq('20° 加 15% 余量 -> 30，顶出去要放大', cur, 30)
  cur = stepRange(cur, 20)
  eq('之后稳定在 30', cur, 30)
  cur = stepRange(cur, 0.2)
  eq('突然静止 -> 缩到 1', cur, 1)
  cur = stepRange(cur, 0.2)
  eq('继续静止保持 1', cur, 1)
  cur = stepRange(cur, 28)
  eq('重新倾斜 28° 立刻放大', cur, 50)
}

console.log(fails === 0
  ? `\nCHART_OK  ${checks} checks, 0 failure`
  : `\nCHART_FAIL  ${checks} checks, ${fails} failure`)
process.exit(fails === 0 ? 0 : 1)
