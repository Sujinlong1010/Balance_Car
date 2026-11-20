<template>
  <view class="page">
    <!-- ============ 顶部：连接状态 ============ -->
    <view class="topbar">
      <view class="dot" :class="connected ? 'on' : 'off'"></view>
      <view class="devinfo">
        <text class="devname">{{ connected ? deviceName : '未连接' }}</text>
        <text class="devsub" v-if="connected">{{ deviceMac }} · 心跳 {{ hbOk ? '正常' : '中断' }}</text>
        <text class="devsub" v-else>先在系统设置里配对 HC-05（PIN 1234）</text>
      </view>
      <button class="connbtn" size="mini" :type="connected ? 'warn' : 'primary'"
              @click="pickDevice">{{ connected ? '断开' : '连接' }}</button>
    </view>

    <!-- ============ 页签 ============ -->
    <view class="tabs">
      <view class="tab" :class="{ sel: tab === 'drive' }" @click="tab = 'drive'">驾驶</view>
      <view class="tab" :class="{ sel: tab === 'pid' }" @click="tab = 'pid'">调参</view>
      <view class="tab" :class="{ sel: tab === 'more' }" @click="tab = 'more'">存档/日志</view>
    </view>

    <!-- ==================== 驾驶 ==================== -->
    <scroll-view class="body" scroll-y v-show="tab === 'drive'">
      <view class="stats">
        <view class="stat">
          <text class="k">角度</text>
          <text class="v" :class="cls(angle)">{{ angle.toFixed(2) }}°</text>
        </view>
        <view class="stat">
          <text class="k">目标角</text>
          <text class="v">{{ target.toFixed(2) }}°</text>
        </view>
        <view class="stat">
          <text class="k">PWM</text>
          <text class="v" :class="cls(pwm)">{{ pwm }}</text>
        </view>
        <view class="stat">
          <text class="k">轮速</text>
          <text class="v">{{ carSpd.toFixed(2) }}</text>
        </view>
      </view>

      <view class="staterow">
        <text class="badge" :class="stateClass">{{ stateName }}</text>
        <text class="hint" v-if="state === 0">已急停，电机不通电</text>
        <text class="hint" v-else-if="state === 1">标定中 · 扶正保持静止 {{ cal }}%</text>
        <text class="hint" v-else-if="state === 2">闭环运行 · 按住方向键行驶</text>
        <text class="hint" v-else>保护停机 · 车身超过 50°，先急停再标定</text>
      </view>

      <canvas canvas-id="wave" id="wave" class="wave"
              :style="{ width: cw + 'px', height: ch + 'px' }"></canvas>

      <!-- 限速 -->
      <view class="pctrow">
        <text class="pctlabel">限速</text>
        <slider class="pctslider" :value="pct" :min="10" :max="100" :step="5"
                activeColor="#2f9e44" backgroundColor="#22303f" block-size="22"
                @changing="onPct" @change="onPct" />
        <text class="pctval">{{ pct }}%</text>
      </view>

      <!-- 方向 + 动作 -->
      <view class="ctrl">
        <view class="pad">
          <view class="row">
            <view class="k" :class="{ act: keys.w }"
                  @touchstart.prevent="press('w')" @touchend.prevent="release('w')"
                  @touchcancel.prevent="release('w')">前</view>
          </view>
          <view class="row">
            <view class="k" :class="{ act: keys.a }"
                  @touchstart.prevent="press('a')" @touchend.prevent="release('a')"
                  @touchcancel.prevent="release('a')">左</view>
            <view class="k stop" @click="moveStop">停</view>
            <view class="k" :class="{ act: keys.d }"
                  @touchstart.prevent="press('d')" @touchend.prevent="release('d')"
                  @touchcancel.prevent="release('d')">右</view>
          </view>
          <view class="row">
            <view class="k" :class="{ act: keys.s }"
                  @touchstart.prevent="press('s')" @touchend.prevent="release('s')"
                  @touchcancel.prevent="release('s')">后</view>
          </view>
        </view>

        <view class="acts">
          <button class="act" @click="doCal">扶正标定</button>
          <button class="act go" @click="doBalOn">开始平衡</button>
          <button class="act stop" @click="doBalOff">急停</button>
          <text class="cmdtext">目标 {{ cmdFwd > 0 ? '+' : '' }}{{ cmdFwd }}% / {{ cmdTurn > 0 ? '+' : '' }}{{ cmdTurn }}%</text>
        </view>
      </view>

      <view class="logbox">
        <text class="logline" v-for="(l, i) in lastLogs" :key="i">{{ l }}</text>
      </view>
    </scroll-view>

    <!-- ==================== 调参 ==================== -->
    <scroll-view class="body" scroll-y v-show="tab === 'pid'">
      <view class="tip">拖滑块松手立刻下发；也可以点 − / + 微调</view>

      <view class="pidrow" v-for="s in specs" :key="s.key">
        <view class="pidhead">
          <text class="pidname">{{ s.label }}</text>
          <view class="stepper">
            <text class="stepbtn" @click="step(s, -1)">−</text>
            <text class="pidval">{{ pid[s.key].toFixed(s.fmt) }}</text>
            <text class="stepbtn" @click="step(s, 1)">+</text>
          </view>
        </view>
        <slider :value="pid[s.key]" :min="s.min" :max="s.max" :step="s.step"
                activeColor="#1c7ed6" backgroundColor="#22303f" block-size="20"
                @changing="onSlide(s, $event)" @change="onSlide(s, $event)" />
      </view>

      <view class="btnrow">
        <button class="wide" type="primary" @click="sendAllPid">下发全部</button>
        <button class="wide" @click="readPid">读取</button>
      </view>
      <view class="btnrow">
        <button class="wide" type="default" @click="saveFlash">存入小车 Flash</button>
      </view>
      <view class="tip">存 Flash 前要急停；写入约 30 ms，运行中固件会拒绝。</view>
    </scroll-view>

    <!-- ==================== 存档 / 日志 ==================== -->
    <scroll-view class="body" scroll-y v-show="tab === 'more'">
      <view class="sect">
        <text class="secttitle">参数存档</text>
        <view class="saverow">
          <input class="nameinput" v-model="presetName" placeholder="存档名，可留空" />
          <button size="mini" type="primary" @click="savePreset">保存当前</button>
        </view>
        <view class="preset" v-for="(p, i) in presets" :key="i">
          <view class="pinfo">
            <text class="pname">{{ p.name }}</text>
            <text class="ptime">{{ p.time }}</text>
            <text class="pvals">{{ p.brief }}</text>
          </view>
          <button size="mini" @click="loadPreset(i)">载入</button>
          <button size="mini" type="warn" @click="delPreset(i)">删</button>
        </view>
        <view class="tip" v-if="!presets.length">还没有存档。调好参数后在这里存一份。</view>
      </view>

      <view class="sect">
        <text class="secttitle">通信日志（最近 80 行）</text>
        <view class="logbox tall">
          <text class="logline" v-for="(l, i) in logs" :key="i">{{ l }}</text>
          <text class="logline dim" v-if="!logs.length">尚未收到数据</text>
        </view>
        <button size="mini" @click="logs = []">清空日志</button>
      </view>

      <view class="sect">
        <text class="secttitle">使用提示</text>
        <text class="tip">1. 手机先在「设置 → 蓝牙」里和 HC-05 配对（PIN 1234），App 只列已配对设备。</text>
        <text class="tip">2. 每次连接会自动读回三组增益，避免手机上的数值和车上不一致。</text>
        <text class="tip">3. 方向键按住即走，松手停；App 每 100 ms 发一次心跳，蓝牙断开 0.6 s 后小车自动停。</text>
        <text class="tip">4. 车切到后台或退出，心跳停止，车会自己停下——这是设计好的保护。</text>
      </view>
    </scroll-view>
  </view>
</template>

<script>
import SPP from '@/common/spp.js'
import { CMD, buildMove, buildSet, buildSetv, buildSett, parseLine, BST_STATES } from '@/common/protocol.js'

const WAVE_N = 240
const STATE_NAMES = { 0: 'IDLE', 1: 'CAL', 2: 'RUN', 3: 'FAULT' }

export default {
  data() {
    return {
      tab: 'drive',
      connected: false,
      hbOk: false,
      deviceName: '',
      deviceMac: '',

      // 遥测
      angle: 0, target: 0, pwm: 0, vel: 0, enc: 0,
      state: 0, cal: 100, carSpd: 0, cmdFwd: 0, cmdTurn: 0,

      // 行驶
      pct: 40,
      keys: { w: false, s: false, a: false, d: false },

      // 调参
      pid: { kp: 3.30, ki: 0.15, kd: 3.00, kvp: 0.800, kvi: 0.065, ktp: 3.00, kti: 4.00 },
      specs: [
        { key: 'kp',  label: '角度 KP', min: 0,    max: 15,  step: 0.01,  fine: 0.01,  fmt: 2 },
        { key: 'ki',  label: '角度 KI', min: 0,    max: 1,   step: 0.01,  fine: 0.01,  fmt: 2 },
        { key: 'kd',  label: '角度 KD', min: 0,    max: 15,  step: 0.01,  fine: 0.01,  fmt: 2 },
        { key: 'kvp', label: '速度 KP', min: -2,   max: 8,   step: 0.005, fine: 0.005, fmt: 3 },
        { key: 'kvi', label: '速度 KI', min: -0.2, max: 1,   step: 0.001, fine: 0.001, fmt: 3 },
        { key: 'ktp', label: '转向 KP', min: 0,    max: 15,  step: 0.01,  fine: 0.01,  fmt: 2 },
        { key: 'kti', label: '转向 KI', min: 0,    max: 10,  step: 0.01,  fine: 0.01,  fmt: 2 }
      ],

      // 存档 / 日志
      presetName: '',
      presets: [],
      logs: [],

      // 波形
      cw: 320, ch: 170,
      wa: [], wt: []
    }
  },

  computed: {
    stateName() { return STATE_NAMES[this.state] || '--' },
    stateClass() {
      return ['b-idle', 'b-cal', 'b-run', 'b-fault'][this.state] || 'b-idle'
    },
    lastLogs() { return this.logs.slice(-3) }
  },

  onLoad() {
    const si = uni.getSystemInfoSync()
    this.cw = Math.floor(si.windowWidth - 24)
    this.ch = Math.floor(Math.min(200, this.cw * 0.52))
    try {
      const saved = uni.getStorageSync('bc_presets')
      if (Array.isArray(saved)) { this.presets = saved }
    } catch (e) { /* ignore */ }
  },

  onReady() { this.drawWave() },
  onShow() { if (this.connected) { this.startLoops() } },

  onHide() { this.stopAll() },
  onUnload() { this.stopAll() },

  /* 让页面离开时一定是「停车」状态 */
  beforeDestroy() { this.stopAll() },

  methods: {
    cls(v) { return { pos: v > 0.15, neg: v < -0.15 } },

    /* ---------------- 连接 ---------------- */
    pickDevice() {
      if (this.connected) {
        this.stopAll()
        uni.showToast({ title: '已断开', icon: 'none' })
        return
      }
      if (!SPP.isSupported()) {
        uni.showModal({
          title: '需要手机 App',
          content: '经典蓝牙（SPP）只有打包成 App 才能用，网页版做不到。请在 HBuilderX 里云打包成 APK 安装到手机。',
          showCancel: false
        })
        return
      }
      const list = SPP.listBonded()
      if (!list.length) {
        const err = SPP.getLastError()
        uni.showModal({
          title: '没有可用设备',
          content: err || '请先到「系统设置 → 蓝牙」里搜索并配对 HC-05（PIN 1234），配对成功后再回来点连接。',
          showCancel: false
        })
        return
      }
      if (list.length === 1) { this.doConnect(list[0]); return }
      uni.showActionSheet({
        itemList: list.slice(0, 6).map(d => (d.name || '未命名') + ' ' + d.mac),
        success: res => this.doConnect(list[res.tapIndex])
      })
    },

    doConnect(dev) {
      uni.showLoading({ title: '连接中…', mask: true })
      // 先把 loading 渲染出来，再进阻塞的 socket.connect()
      setTimeout(() => {
        try {
          SPP.connect(dev.mac)
          this.deviceName = dev.name || dev.mac
          this.deviceMac = dev.mac
          this.connected = true
          this.hbOk = true
          uni.hideLoading()
          this.pushLog('已连接 ' + this.deviceName)
          this.startLoops()
          this.readPid()          // 连上先和车上参数对齐
        } catch (e) {
          this.connected = false
          uni.hideLoading()
          uni.showModal({
            title: '连接失败',
            content: e.userMessage || String(e.message || e),
            showCancel: false
          })
        }
      }, 80)
    },

    startLoops() {
      this.stopTimers()
      this._rx = setInterval(this.tickRx, 20)         // 收
      this._move = setInterval(this.tickMove, 100)    // 心跳 + 方向
      this._draw = setInterval(this.drawWave, 100)
      this._bst = setInterval(() => { if (this.connected) { SPP.send(CMD.bst) } }, 500)
      this._hb = setInterval(() => {
        // 心跳节拍器：发得出去就算正常
        this.hbOk = this.connected && SPP.isConnected()
        if (!this.hbOk && this.connected) { this.dropLink('蓝牙连接已断开') }
      }, 1000)
    },

    stopTimers() {
      ['_rx', '_move', '_draw', '_bst', '_hb'].forEach(k => {
        if (this[k]) { clearInterval(this[k]); this[k] = null }
      })
    },

    stopAll() {
      this.stopTimers()
      this.keys = { w: false, s: false, a: false, d: false }
      if (this.connected) {
        SPP.send(buildMove(0, 0, this.pct))   // 先踩一脚刹车再走
      }
      SPP.close()
      this.connected = false
      this.hbOk = false
    },

    dropLink(reason) {
      this.stopTimers()
      SPP.close()
      this.connected = false
      this.hbOk = false
      this.keys = { w: false, s: false, a: false, d: false }
      this.pushLog('× ' + reason)
      uni.showToast({ title: reason, icon: 'none' })
    },

    /* ---------------- 接收 ---------------- */
    tickRx() {
      if (!this.connected) { return }
      const lines = SPP.poll()
      for (let i = 0; i < lines.length; i++) { this.handleLine(lines[i]) }
      if (!SPP.isConnected() && this.connected) {
        this.dropLink(SPP.getLastError() || '连接中断')
      }
    },

    handleLine(line) {
      const d = parseLine(line)
      switch (d.kind) {
        case 't':
          this.angle = d.angle
          this.target = d.target
          this.pwm = d.pwm
          this.vel = d.vel
          this.enc = d.enc
          this.wa.push(d.angle); if (this.wa.length > WAVE_N) { this.wa.shift() }
          this.wt.push(d.target); if (this.wt.length > WAVE_N) { this.wt.shift() }
          break
        case 'bst':
          this.state = d.state
          this.cal = d.cal
          this.carSpd = d.spd
          this.cmdFwd = d.cmdFwd
          this.cmdTurn = d.cmdTurn
          break
        case 'ack':
          this.pid.kp = d.kp; this.pid.ki = d.ki; this.pid.kd = d.kd
          this.pushLog('读回角度环 ' + d.kp + '/' + d.ki + '/' + d.kd)
          break
        case 'ackv':
          this.pid.kvp = d.kvp; this.pid.kvi = d.kvi
          this.pushLog('读回速度环 ' + d.kvp + '/' + d.kvi)
          break
        case 'ackt':
          this.pid.ktp = d.ktp; this.pid.kti = d.kti
          this.pushLog('读回转向环 ' + d.ktp + '/' + d.kti)
          break
        case 'ackm':
          this.cmdFwd = d.fwd
          this.cmdTurn = d.turn
          if (!d.running) { this.pushLog('提示：小车未使能，先点「开始平衡」') }
          break
        case 'log':
          this.pushLog((d.error ? '! ' : '· ') + d.text)
          if (/Params saved to flash/.test(d.text)) {
            uni.showToast({ title: '已存入 Flash', icon: 'success' })
          } else if (/SAVE rejected/.test(d.text)) {
            uni.showToast({ title: '运行中不能存，先急停', icon: 'none' })
          } else if (/unknown cmd/.test(d.text)) {
            uni.showToast({ title: '固件太旧，请烧最新 hex', icon: 'none' })
          }
          break
      }
    },

    pushLog(text) {
      const t = new Date()
      const hh = ('0' + t.getHours()).slice(-2)
      const mm = ('0' + t.getMinutes()).slice(-2)
      const ss = ('0' + t.getSeconds()).slice(-2)
      this.logs.push(hh + ':' + mm + ':' + ss + '  ' + text)
      if (this.logs.length > 80) { this.logs.shift() }
    },

    /* ---------------- 行驶 ---------------- */
    heartbeatFrame() {
      const fwd = (this.keys.w ? 1 : 0) - (this.keys.s ? 1 : 0)
      const turn = (this.keys.d ? 1 : 0) - (this.keys.a ? 1 : 0)
      return buildMove(fwd, turn, this.pct)
    },

    tickMove() {
      if (!this.connected) { return }
      SPP.send(this.heartbeatFrame())     // 值不变也发：这是失联保护的心跳
    },

    pushMove() {
      if (!this.connected) { return }
      SPP.send(this.heartbeatFrame())
    },

    press(k) {
      if (!this.connected) {
        uni.showToast({ title: '先连接蓝牙', icon: 'none' })
        return
      }
      this.keys[k] = true
      this.pushMove()
      try { uni.vibrateShort() } catch (e) { /* ignore */ }
    },

    release(k) {
      this.keys[k] = false
      this.pushMove()
    },

    moveStop() {
      this.keys = { w: false, s: false, a: false, d: false }
      this.pushMove()
      if (this.connected) { uni.showToast({ title: '停车', icon: 'none' }) }
    },

    onPct(e) {
      this.pct = e.detail.value
      this.pushMove()
    },

    /* ---------------- 动作 ---------------- */
    doCal() {
      if (!this.connected) { return this.needConn() }
      uni.showModal({
        title: '扶正标定',
        content: '把车扶到竖直并保持静止，标定约 1 秒，期间电机不动。',
        success: res => {
          if (res.confirm) {
            SPP.send(CMD.cal)
            this.pushLog('发送 CAL')
          }
        }
      })
    },

    doBalOn() {
      if (!this.connected) { return this.needConn() }
      uni.showModal({
        title: '开始平衡',
        content: '确认已标定、车基本竖直、周围无障碍？',
        success: res => {
          if (res.confirm) {
            this.keys = { w: false, s: false, a: false, d: false }
            SPP.send(buildMove(0, 0, this.pct))
            SPP.send(CMD.on)
            this.pushLog('发送 BAL,1')
          }
        }
      })
    },

    doBalOff() {
      if (!this.connected) { return }
      this.keys = { w: false, s: false, a: false, d: false }
      SPP.send(buildMove(0, 0, this.pct))
      SPP.send(CMD.off)
      this.pushLog('发送 BAL,0 急停')
      try { uni.vibrateShort() } catch (e) { /* ignore */ }
    },

    needConn() {
      uni.showToast({ title: '先连接蓝牙', icon: 'none' })
    },

    /* ---------------- 调参 ---------------- */
    onSlide(s, e) {
      const v = typeof e.detail.value === 'number' ? e.detail.value : parseFloat(e.detail.value)
      this.pid[s.key] = Math.round(v / s.step) * s.step
      if (!this.connected) { return }
      this.sendOne(s.key)
    },

    step(s, dir) {
      const v = this.pid[s.key] + dir * s.fine
      this.pid[s.key] = Math.min(s.max, Math.max(s.min, Math.round(v / s.fine) * s.fine))
      if (this.connected) { this.sendOne(s.key) }
    },

    sendOne(key) {
      if (!this.connected) { return }
      const p = this.pid
      let text = null
      if (key === 'kp' || key === 'ki' || key === 'kd') {
        text = buildSet(p.kp, p.ki, p.kd)
      } else if (key === 'kvp' || key === 'kvi') {
        text = buildSetv(p.kvp, p.kvi)
      } else {
        text = buildSett(p.ktp, p.kti)
      }
      SPP.send(text)
    },

    sendAllPid() {
      if (!this.connected) { return this.needConn() }
      const p = this.pid
      SPP.send(buildSet(p.kp, p.ki, p.kd))
      SPP.send(buildSetv(p.kvp, p.kvi))
      SPP.send(buildSett(p.ktp, p.kti))
      this.pushLog('下发 7 项增益')
      uni.showToast({ title: '已下发 7 项', icon: 'none' })
    },

    readPid() {
      if (!this.connected) { return }
      SPP.send(CMD.get)
      SPP.send(CMD.getv)
      SPP.send(CMD.gett)
    },

    saveFlash() {
      if (!this.connected) { return this.needConn() }
      if (this.state === 2) {
        uni.showToast({ title: '先急停再存', icon: 'none' })
        return
      }
      SPP.send(CMD.save)
      this.pushLog('发送 SAVE')
    },

    /* ---------------- 存档 ---------------- */
    brief() {
      const p = this.pid
      return 'A ' + p.kp.toFixed(2) + '/' + p.ki.toFixed(2) + '/' + p.kd.toFixed(2) +
             '  V ' + p.kvp.toFixed(2) + '/' + p.kvi.toFixed(3) +
             '  T ' + p.ktp.toFixed(2) + '/' + p.kti.toFixed(2)
    },

    savePreset() {
      const name = (this.presetName || '').trim() ||
        ('存档 ' + new Date().toLocaleString('zh-CN', { hour12: false }))
      this.presets.unshift({
        name: name,
        time: new Date().toLocaleString('zh-CN', { hour12: false }),
        gains: Object.assign({}, this.pid),
        brief: this.brief()
      })
      this.presetName = ''
      this.persist()
      uni.showToast({ title: '已保存', icon: 'success' })
    },

    loadPreset(i) {
      const p = this.presets[i]
      if (!p || !p.gains) { return }
      this.pid = Object.assign({}, this.pid, p.gains)
      this.persist()
      if (this.connected) { this.sendAllPid() }
      uni.showToast({ title: '已载入', icon: 'none' })
    },

    delPreset(i) {
      uni.showModal({
        title: '删除存档',
        content: '确定删除「' + this.presets[i].name + '」？',
        success: res => {
          if (res.confirm) {
            this.presets.splice(i, 1)
            this.persist()
          }
        }
      })
    },

    persist() {
      try { uni.setStorageSync('bc_presets', this.presets) } catch (e) { /* ignore */ }
    },

    /* ---------------- 波形 ---------------- */
    drawWave() {
      const w = this.cw
      const h = this.ch
      const ctx = uni.createCanvasContext('wave', this)
      ctx.setFillStyle('#0f1720')
      ctx.fillRect(0, 0, w, h)

      ctx.setStrokeStyle('#1d2836')
      ctx.setLineWidth(1)
      for (let i = 1; i < 4; i++) {
        const y = Math.round(h * i / 4)
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke()
      }
      ctx.beginPath(); ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2); ctx.stroke()

      const mid = h / 2
      const scale = (h / 2 - 4) / 25      // ±25°
      const series = (arr, color) => {
        if (arr.length < 2) { return }
        ctx.setStrokeStyle(color)
        ctx.setLineWidth(1.6)
        ctx.beginPath()
        for (let i = 0; i < arr.length; i++) {
          const x = w * i / (WAVE_N - 1)
          let v = arr[i]
          if (v > 25) { v = 25 } else if (v < -25) { v = -25 }
          const y = mid - v * scale
          if (i === 0) { ctx.moveTo(x, y) } else { ctx.lineTo(x, y) }
        }
        ctx.stroke()
      }
      series(this.wt, '#2b8a3e')
      series(this.wa, '#e03131')

      ctx.setFontSize(10)
      ctx.setFillStyle('#5c6f85')
      ctx.fillText('±25°', 4, 12)
      ctx.setFillStyle('#e03131'); ctx.fillText('角度', w - 52, 12)
      ctx.setFillStyle('#2b8a3e'); ctx.fillText('目标', w - 26, 12)
      ctx.draw()
    }
  }
}
</script>

<style>
.page {
  display: flex;
  flex-direction: column;
  height: 100vh;
  background-color: #0b1016;
}

/* ---- 顶部连接条 ---- */
.topbar {
  display: flex;
  align-items: center;
  padding: 18rpx 24rpx;
  background-color: #111820;
  border-bottom: 1rpx solid #1d2836;
}
.dot {
  width: 16rpx;
  height: 16rpx;
  border-radius: 50%;
  margin-right: 16rpx;
  flex-shrink: 0;
}
.dot.on { background-color: #37b24d; box-shadow: 0 0 12rpx #37b24d; }
.dot.off { background-color: #f03e3e; }
.devinfo { flex: 1; overflow: hidden; }
.devname { font-size: 30rpx; font-weight: 600; color: #e6edf3; }
.devsub { display: block; font-size: 22rpx; color: #5c6f85; margin-top: 4rpx; }
.connbtn { margin: 0; }

/* ---- 页签 ---- */
.tabs { display: flex; background-color: #111820; }
.tab {
  flex: 1;
  text-align: center;
  padding: 20rpx 0;
  font-size: 28rpx;
  color: #7d8fa3;
  border-bottom: 4rpx solid transparent;
}
.tab.sel { color: #4dabf7; border-bottom-color: #4dabf7; }

.body { flex: 1; padding: 16rpx 12rpx; box-sizing: border-box; }

/* ---- 数值 ---- */
.stats { display: flex; }
.stat {
  flex: 1;
  background-color: #111820;
  border-radius: 12rpx;
  padding: 14rpx 8rpx;
  margin: 0 4rpx;
  text-align: center;
}
.stat .k { font-size: 22rpx; color: #7d8fa3; }
.stat .v { display: block; font-size: 34rpx; font-weight: 700; color: #e6edf3; margin-top: 4rpx; }
.stat .v.pos { color: #f03e3e; }
.stat .v.neg { color: #37b24d; }

.staterow { display: flex; align-items: center; padding: 14rpx 4rpx 6rpx; }
.badge {
  font-size: 24rpx; padding: 4rpx 16rpx; border-radius: 8rpx;
  margin-right: 14rpx; font-weight: 600;
}
.b-idle  { background-color: #24313f; color: #a9bccf; }
.b-cal   { background-color: #3b2f10; color: #ffd43b; }
.b-run   { background-color: #12351f; color: #51cf66; }
.b-fault { background-color: #3d1414; color: #ff8787; }
.hint { font-size: 22rpx; color: #6b7c8f; flex: 1; }

/* ---- 波形 ---- */
.wave { margin: 8rpx 0; border-radius: 12rpx; background-color: #0f1720; }

/* ---- 限速 ---- */
.pctrow { display: flex; align-items: center; margin: 4rpx 0; }
.pctlabel { font-size: 24rpx; color: #7d8fa3; width: 72rpx; }
.pctslider { flex: 1; margin: 0 8rpx; }
.pctval { font-size: 26rpx; color: #51cf66; width: 80rpx; text-align: right; }

/* ---- 控制区 ---- */
.ctrl { display: flex; margin-top: 8rpx; }
.pad { flex: 1.25; }
.pad .row { display: flex; justify-content: center; }
.k {
  width: 30%;
  height: 104rpx;
  line-height: 104rpx;
  text-align: center;
  background-color: #1a2531;
  color: #c8d6e5;
  border-radius: 14rpx;
  margin: 5rpx;
  font-size: 32rpx;
  font-weight: 600;
  user-select: none;
}
.k.act { background-color: #1c7ed6; color: #ffffff; }
.k.stop { background-color: #3d1414; color: #ff8787; }

.acts { flex: 1; display: flex; flex-direction: column; justify-content: center; }
.act {
  margin: 5rpx 0;
  font-size: 26rpx;
  background-color: #1a2531;
  color: #c8d6e5;
  border-radius: 12rpx;
}
.act.go { background-color: #12351f; color: #51cf66; }
.act.stop { background-color: #3d1414; color: #ff8787; }
.cmdtext { font-size: 22rpx; color: #5c6f85; text-align: center; margin-top: 6rpx; }

/* ---- 日志 ---- */
.logbox {
  margin-top: 12rpx;
  padding: 10rpx 14rpx;
  background-color: #0f1720;
  border-radius: 12rpx;
}
.logbox.tall { max-height: 620rpx; overflow: hidden; margin-bottom: 12rpx; }
.logline { display: block; font-size: 22rpx; color: #8fa3b8; line-height: 1.5; }
.logline.dim { color: #46586b; }

/* ---- 调参 ---- */
.pidrow {
  background-color: #111820;
  border-radius: 12rpx;
  padding: 10rpx 16rpx 0;
  margin-bottom: 12rpx;
}
.pidhead { display: flex; align-items: center; justify-content: space-between; }
.pidname { font-size: 26rpx; color: #a9bccf; }
.stepper { display: flex; align-items: center; }
.stepbtn {
  width: 56rpx; height: 56rpx; line-height: 52rpx; text-align: center;
  background-color: #1a2531; color: #4dabf7; border-radius: 10rpx;
  font-size: 32rpx; font-weight: 700;
}
.pidval {
  min-width: 130rpx; text-align: center; font-size: 30rpx;
  color: #e6edf3; font-weight: 700;
}

.btnrow { display: flex; margin-top: 12rpx; }
.wide { flex: 1; margin: 0 6rpx; font-size: 28rpx; }

.tip { font-size: 22rpx; color: #5c6f85; line-height: 1.6; margin: 8rpx 6rpx; display: block; }

/* ---- 存档 ---- */
.sect {
  background-color: #111820;
  border-radius: 12rpx;
  padding: 16rpx;
  margin-bottom: 16rpx;
}
.secttitle { font-size: 26rpx; font-weight: 600; color: #c8d6e5; display: block; margin-bottom: 12rpx; }
.saverow { display: flex; align-items: center; }
.nameinput {
  flex: 1;
  height: 64rpx;
  background-color: #0b1016;
  color: #e6edf3;
  border-radius: 10rpx;
  padding: 0 16rpx;
  font-size: 26rpx;
  margin-right: 12rpx;
}
.preset {
  display: flex;
  align-items: center;
  padding: 12rpx 0;
  border-top: 1rpx solid #1d2836;
}
.pinfo { flex: 1; overflow: hidden; }
.pname { font-size: 26rpx; color: #e6edf3; }
.ptime { font-size: 20rpx; color: #5c6f85; margin-left: 10rpx; }
.pvals { display: block; font-size: 20rpx; color: #7d8fa3; margin-top: 4rpx; }
</style>
