/* protocol.test.mjs - 手机端协议层单测，不需要手机也不需要 HBuilderX
 *
 *   node Tools/Mobile_App/test/protocol.test.mjs
 *
 * common/protocol.js 是 ES module，但工程根目录没有 package.json（uni-app 不需要）。
 * 这里把它复制成临时 .mjs 再动态 import，避免给工程加 type:module 干扰 HBuilderX。
 */
import { readFileSync, writeFileSync, mkdtempSync } from 'node:fs'
import { tmpdir } from 'node:os'
import { join, dirname } from 'node:path'
import { fileURLToPath, pathToFileURL } from 'node:url'

const here = dirname(fileURLToPath(import.meta.url))
const src = readFileSync(join(here, '..', 'common', 'protocol.js'), 'utf8')
const dir = mkdtempSync(join(tmpdir(), 'bcproto-'))
const mod = join(dir, 'protocol.mjs')
writeFileSync(mod, src)
const P = await import(pathToFileURL(mod).href)

const { buildMove, buildSet, buildSetv, buildSett, parseLine, CMD, BST_STATES } = P

let fails = 0
let checks = 0
function eq(label, got, want) {
  checks++
  const ok = JSON.stringify(got) === JSON.stringify(want)
  if (!ok) {
    fails++
    console.log('  [FAIL]', label, '\n         got ', JSON.stringify(got),
                '\n         want', JSON.stringify(want))
  } else {
    console.log('  [OK]  ', label)
  }
}

console.log('--- 组帧：双杆比例（摇杆在中间要有中间量）---')
eq('前进 60%', buildMove(1, 0, 60), 'MOV,60,0\n')
eq('后退 + 左转', buildMove(-1, -1, 40), 'MOV,-40,-40\n')
eq('前进 + 右转（斜推）', buildMove(1, 1, 100), 'MOV,100,100\n')
eq('停车', buildMove(0, 0, 40), 'MOV,0,0\n')
eq('摇杆推一半 = 半个量', buildMove(0.5, 0, 60), 'MOV,30,0\n')
eq('摇杆 0.25 前进 + 0.75 左', buildMove(0.25, -0.75, 80), 'MOV,20,-60\n')
eq('极小值不产生 -0', buildMove(-0.004, 0, 100), 'MOV,0,0\n')
eq('限速夹紧 250 -> 100', buildMove(1, 0, 250), 'MOV,100,0\n')
eq('比例夹紧 2 / -3 -> 1 / -1', buildMove(2, -3, 50), 'MOV,50,-50\n')
eq('负数限速不产生 -0', buildMove(0, 0, -10), 'MOV,0,0\n')
eq('undefined 比例当 0', buildMove(undefined, null, 40), 'MOV,0,0\n')

console.log('--- 组帧：三组增益（定点放大）---')
eq('SET x100', buildSet(3.3, 0.15, 3.0), 'SET,330,15,300\n')
eq('SETV x10000', buildSetv(0.8, 0.065), 'SETV,8000,650\n')
eq('SETT x100', buildSett(3.0, 4.0), 'SETT,300,400\n')
eq('SET 四舍五入', buildSet(3.305, 0.149, 2.999), 'SET,331,15,300\n')

console.log('--- 解析：固件真实输出 ---')
let d = parseLine('T,1200,1234,0,120,30,-40,3500,-150,25,600,12')
eq('T kind', d.kind, 't')
eq('T 角度 /100', d.angle, 12.34)
eq('T 目标角', d.target, 0)
eq('T P/I/D', [d.p, d.i, d.d], [1.2, 0.3, -0.4])
eq('T PWM 是原值不是 x100', d.pwm, 3500)
eq('T 速度环输出', d.vel, -1.5)
eq('T 转向/编码器', [d.turn, d.enc], [25, 600])
eq('T 陀螺 Z', d.gyroz, 0.12)

d = parseLine('BST,2,150,-120,345,200,0,50,100,-150,25,60,-30,85')
eq('BST 状态 RUN', [d.kind, d.state, BST_STATES[d.state]], ['bst', 2, 'RUN'])
eq('BST 角度/标定值', [d.angle, d.offset], [1.5, -1.2])
eq('BST PWM', d.duty, 345)
eq('BST 运动命令 + 实测速度', [d.cmdFwd, d.cmdTurn, d.spd], [60, -30, 0.85])
eq('BST 旧固件 9 段仍兼容', parseLine('BST,0,10,0,0,0,0,0,100').state, 0)
eq('BST 旧固件 11 段仍兼容',
   parseLine('BST,3,4500,-120,0,0,0,0,100,0,0').state, 3)

d = parseLine('ACK,330,15,300')
eq('ACK 角度环', [d.kind, d.kp, d.ki, d.kd], ['ack', 3.3, 0.15, 3])
d = parseLine('ACKV,8000,650')
eq('ACKV 速度环', [d.kind, d.kvp, d.kvi], ['ackv', 0.8, 0.065])
d = parseLine('ACKT,300,400')
eq('ACKT 转向环', [d.kind, d.ktp, d.kti], ['ackt', 3, 4])
d = parseLine('ACKM,60,0,1')
eq('ACKM 运行中', [d.kind, d.fwd, d.turn, d.running], ['ackm', 60, 0, true])
eq('ACKM 未使能', parseLine('ACKM,0,0,0').running, false)

d = parseLine('LOG,Params saved to flash - survives power off')
eq('LOG 成功不标错', [d.kind, d.error], ['log', false])
eq('LOG 文本完整保留逗号', d.text, 'Params saved to flash - survives power off')
eq('LOG ERR 标错', parseLine('LOG,ERR SAVE rejected - BAL,0 first').error, true)
eq('LOG ERR 正文', parseLine('LOG,ERR MOV turn').text, 'ERR MOV turn')

console.log('--- 异常输入必须不崩 ---')
for (const bad of ['', '   ', '\r\n', 'T,1,2', 'GARBAGE', 'BST,0,1', 'ACK,1,2',
                   'T,a,b,c,d,e,f,g', null, undefined, 'ACKM,x,y,z', 'LOG']) {
  const r = parseLine(bad)
  checks++
  if (!r || typeof r.kind !== 'string') {
    fails++
    console.log('  [FAIL] 输入 ' + JSON.stringify(bad))
  }
}
console.log('  [OK]  12 条畸形输入全部返回对象')
eq('空行', parseLine('').kind, 'empty')
eq('未知命令', parseLine('NOPE').kind, 'other')

console.log('--- 命令常量 ---')
eq('CAL', CMD.cal, 'CAL\n')
eq('BAL,1 / BAL,0', [CMD.on, CMD.off], ['BAL,1\n', 'BAL,0\n'])
eq('BST / SAVE', [CMD.bst, CMD.save], ['BST\n', 'SAVE\n'])

console.log('')
if (fails) {
  console.log('=== ' + fails + ' / ' + checks + ' failure(s) ===')
  process.exit(1)
}
console.log('PROTOCOL_OK  ' + checks + ' checks, 0 failure')
