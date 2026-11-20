/* chart.js - 波形显示用的纯计算，不碰 canvas，方便在 Node 里单测
 *
 *   node Tools/Mobile_App/test/chart.test.mjs
 */

/* 峰值 -> 纵轴量程。
 *
 * 量程固定的话两头都不好看：峰值只有 1° 时按 ±25° 画，曲线基本是一条直线，
 * 看不出角度环在抖还是在稳；峰值 30° 时又会削顶。
 *
 * 档位只取 1/2/3/5/10 × 10^n（人眼好读），向上取，最小 1°——
 * 车没使能时会有一点点噪声，量程再小就会把噪声画成「剧烈震荡」。
 */
export function pickRange(peak) {
  const p = Math.max(peak, 1)
  const pow = Math.pow(10, Math.floor(Math.log10(p)))
  const n = p / pow
  const step = n <= 1 ? 1 : n <= 2 ? 2 : n <= 3 ? 3 : n <= 5 ? 5 : 10
  return Math.max(1, step * pow)
}

/* 带迟滞的量程推进，返回新的量程。
 *
 * 每帧都用 pickRange(peak) 当量程会抖：峰值在 2.0 附近来回，量程就在
 * 2° 和 3° 之间每 100 ms 跳一次，整张图跟着缩放，看的人会晕。
 * 所以只允许两种变化：
 *   - 峰值顶出去（want > cur）→ 立刻放大，不能削顶
 *   - 峰值缩到当前量程 45% 以下 → 才缩小
 * 中间地带保持不变。
 */
export function stepRange(cur, peak) {
  const want = pickRange(peak * 1.15)
  if (!cur) { return want }
  if (want > cur) { return want }
  if (want <= cur / 2.2) { return want }
  return cur
}
