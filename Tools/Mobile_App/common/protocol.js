/* protocol.js - 与固件 Core/Src/app_pid_tune.c 一致的 ASCII 协议（手机端）
 *
 * 纯函数，不依赖 uni / plus，方便在浏览器里直接跑单元测试。
 *
 *   PC/手机 -> 小车 : MOV,<fwd>,<turn>   SET/SETV/SETT   GET/GETV/GETT
 *                     CAL   BAL,<0|1>    SAVE   BST
 *   小车 -> 手机     : T,..  BST,..  ACK/ACKV/ACKT/ACKM  LOG,..
 *
 * 单位：增益在链路上一律放大成整数（角度/转向 x100，速度 x10000）。
 */

export const BST_STATES = { 0: 'IDLE', 1: 'CAL', 2: 'RUN', 3: 'FAULT' }

/* 方向比例 + 限速百分比 -> MOV 帧。
 *
 * fwd / turn 是 -1..1 的比例（摇杆位置），不是整数档位：摇杆推一半就是
 * 一半的量，所以这里只夹紧不取整，取整放到最后乘完 pct 之后做。
 * 固件把 MOV 当失联看门狗：同一个值重复发就是心跳，不会重复回执。 */
export function buildMove(fwd, turn, pct) {
  const p = Math.max(0, Math.min(100, Math.round(pct)))
  const f = Math.max(-1, Math.min(1, Number(fwd) || 0))
  const t = Math.max(-1, Math.min(1, Number(turn) || 0))
  const fv = Math.round(f * p)
  const tv = Math.round(t * p)
  return 'MOV,' + (fv === 0 ? 0 : fv) + ',' + (tv === 0 ? 0 : tv) + '\n'
}

export function buildSet(kp, ki, kd) {
  return 'SET,' + Math.round(kp * 100) + ',' + Math.round(ki * 100) +
         ',' + Math.round(kd * 100) + '\n'
}

export function buildSetv(kp, ki) {
  return 'SETV,' + Math.round(kp * 10000) + ',' + Math.round(ki * 10000) + '\n'
}

export function buildSett(kp, ki) {
  return 'SETT,' + Math.round(kp * 100) + ',' + Math.round(ki * 100) + '\n'
}

export const CMD = {
  get: 'GET\n',
  getv: 'GETV\n',
  gett: 'GETT\n',
  cal: 'CAL\n',
  on: 'BAL,1\n',
  off: 'BAL,0\n',
  bst: 'BST\n',
  save: 'SAVE\n'
}

function nums(parts, from) {
  const out = []
  for (let i = from; i < parts.length; i++) {
    const v = parseFloat(parts[i])
    if (isNaN(v)) { return null }
    out.push(v)
  }
  return out
}

/* 一行 -> 结构化对象。永远返回对象，kind='other' 表示不认识。 */
export function parseLine(raw) {
  const line = String(raw == null ? '' : raw).replace(/[\r\n]+$/, '').trim()
  if (!line) { return { kind: 'empty' } }
  const p = line.split(',')

  switch (p[0]) {
    case 'T': {
      if (p.length < 8) { break }
      const v = nums(p, 1)
      if (!v) { break }
      return {
        kind: 't',
        tms: v[0],
        angle: v[1] / 100,
        target: v[2] / 100,
        p: v[3] / 100,
        i: v[4] / 100,
        d: v[5] / 100,
        pwm: v[6],
        vel: v[7] / 100,
        turn: v.length > 8 ? v[8] : 0,
        enc: v.length > 9 ? v[9] : 0,
        gyroz: v.length > 10 ? v[10] / 100 : 0
      }
    }
    case 'BST': {
      if (p.length < 9) { break }
      const v = nums(p, 1)
      if (!v) { break }
      return {
        kind: 'bst',
        state: v[0],
        angle: v[1] / 100,
        offset: v[2] / 100,
        duty: v[3],
        cal: v[7],
        vel: v.length > 8 ? v[8] / 100 : 0,
        turn: v.length > 9 ? v[9] : 0,
        cmdFwd: v.length > 10 ? v[10] : 0,
        cmdTurn: v.length > 11 ? v[11] : 0,
        spd: v.length > 12 ? v[12] / 100 : 0
      }
    }
    case 'ACK': {
      if (p.length !== 4) { break }
      const v = nums(p, 1)
      if (!v) { break }
      return { kind: 'ack', kp: v[0] / 100, ki: v[1] / 100, kd: v[2] / 100 }
    }
    case 'ACKV': {
      if (p.length !== 3) { break }
      const v = nums(p, 1)
      if (!v) { break }
      return { kind: 'ackv', kvp: v[0] / 10000, kvi: v[1] / 10000 }
    }
    case 'ACKT': {
      if (p.length !== 3) { break }
      const v = nums(p, 1)
      if (!v) { break }
      return { kind: 'ackt', ktp: v[0] / 100, kti: v[1] / 100 }
    }
    case 'ACKM': {
      if (p.length !== 4) { break }
      return {
        kind: 'ackm',
        fwd: parseInt(p[1], 10),
        turn: parseInt(p[2], 10),
        running: parseInt(p[3], 10) === 1
      }
    }
    case 'LOG':
      return { kind: 'log', text: p.slice(1).join(','), error: /^ERR\b/.test(p[1] || '') }
  }
  return { kind: 'other', raw: line }
}
