/* spp.js - Android 经典蓝牙 SPP（RFCOMM）封装，走 Native.js
 *
 * 为什么不用 uni.bluetooth：uni-app / 5+ 的蓝牙 API 只实现 BLE（GATT），
 * 无法与经典蓝牙的串口透传（SPP）通信，HC-05 属于后者。DCloud 官方也建议
 * 非 BLE 设备改用 Native.js。
 *
 * 两个关键实现选择（都是为了绕开 Native.js 的坑）：
 *  1) 发送用 DataOutputStream.writeBytes(String)，不需要在 JS 里构造 byte[]；
 *  2) 接收用 BufferedReader.readLine()，但先用 ready() 判断有没有数据再读，
 *     否则 readLine() 会阻塞 WebView 主线程把界面卡死。
 * 我们的协议本来就是 LF 分隔的 ASCII 行，readLine 正好一位对一位。
 *
 * 用法（只在 App 端有效）：
 *   SPP.listBonded()        -> [{name, mac}]
 *   SPP.connect(mac)        -> true / 抛异常（带中文原因）
 *   SPP.send('MOV,50,0\n')  -> true/false
 *   SPP.poll()              -> ['T,...', 'BST,...']   非阻塞，取当前所有整行
 *   SPP.close()
 */

const SPP_UUID = '00001101-0000-1000-8000-00805F9B34FB'
const BOND_BONDED = 12

let socket = null
let reader = null
let writer = null
let connected = false
let lastError = ''

export function isSupported() {
  return typeof plus !== 'undefined' && plus.android != null
}

export function getLastError() {
  return lastError
}

export function isConnected() {
  return connected
}

function fail(msg) {
  lastError = msg
  const err = new Error(msg)
  err.userMessage = msg
  throw err
}

function getAdapter() {
  if (!isSupported()) { fail('只有打包成 App 的手机端才支持蓝牙（网页版不行）') }
  const BluetoothAdapter = plus.android.importClass('android.bluetooth.BluetoothAdapter')
  const adapter = BluetoothAdapter.getDefaultAdapter()
  if (!adapter) { fail('这台手机没有经典蓝牙模块，无法连接 HC-05') }
  plus.android.importClass(adapter)
  if (!adapter.isEnabled()) {
    fail('手机蓝牙未开启，请先打开系统蓝牙开关')
  }
  return adapter
}

/* 列出「已配对」设备。SPP 必须先配对再连接，扫描是扫不到的。 */
export function listBonded() {
  const out = []
  try {
    const adapter = getAdapter()
    const set = adapter.getBondedDevices()
    if (!set) { return out }
    plus.android.importClass(set)
    const it = set.iterator()
    plus.android.importClass(it)
    while (it.hasNext()) {
      const d = it.next()
      plus.android.importClass(d)
      out.push({
        name: String(d.getName() || ''),
        mac: String(d.getAddress() || ''),
        bonded: d.getBondState() === BOND_BONDED
      })
    }
  } catch (e) {
    lastError = e.userMessage || String(e)
  }
  return out
}

function closeQuietly() {
  try { if (socket) { socket.close() } } catch (e) { /* ignore */ }
  socket = null
  reader = null
  writer = null
  connected = false
}

/* 同步阻塞，最长几秒（连接超时）。调用方应先显示「连接中」。 */
export function connect(mac) {
  closeQuietly()
  lastError = ''
  try {
    const adapter = getAdapter()
    const device = adapter.getRemoteDevice(mac)
    if (!device) { fail('找不到这个设备，可能已被取消配对') }
    plus.android.importClass(device)

    if (device.getBondState() !== BOND_BONDED) {
      fail('还没配对：请先在「系统设置 → 蓝牙」里与 HC-05 配对（PIN 1234）')
    }

    adapter.cancelDiscovery()

    const UUID = plus.android.importClass('java.util.UUID')
    const uuid = UUID.fromString(SPP_UUID)
    socket = device.createRfcommSocketToServiceRecord(uuid)
    if (!socket) { fail('无法创建 SPP 通道') }
    plus.android.importClass(socket)

    try {
      socket.connect()
    } catch (e) {
      closeQuietly()
      fail('连接失败：' + (e.message || e) +
           '｜检查模块是否已上电、是否被电脑占用')
    }

    const ins = socket.getInputStream()
    plus.android.importClass(ins)
    const InputStreamReader = plus.android.importClass('java.io.InputStreamReader')
    const isr = new InputStreamReader(ins, 'US-ASCII')
    plus.android.importClass(isr)
    const BufferedReader = plus.android.importClass('java.io.BufferedReader')
    reader = new BufferedReader(isr)
    plus.android.importClass(reader)

    const outs = socket.getOutputStream()
    plus.android.importClass(outs)
    const DataOutputStream = plus.android.importClass('java.io.DataOutputStream')
    writer = new DataOutputStream(outs)
    plus.android.importClass(writer)

    connected = true
    return true
  } catch (e) {
    closeQuietly()
    if (!e.userMessage) { lastError = String(e.message || e) }
    throw e
  }
}

export function send(text) {
  if (!connected || !writer) { return false }
  try {
    writer.writeBytes(text)     // ASCII 为主，低 8 位即可
    writer.flush()
    return true
  } catch (e) {
    lastError = '发送失败：' + (e.message || e)
    closeQuietly()
    return false
  }
}

/* 非阻塞取行：ready() 为假就立刻返回，绝不卡界面。 */
export function poll() {
  const lines = []
  if (!connected || !reader) { return lines }
  try {
    let guard = 0
    while (guard++ < 64) {
      if (!reader.ready()) { break }
      const line = reader.readLine()
      if (line === null) { connected = false; break }
      lines.push(String(line))
    }
  } catch (e) {
    lastError = '读取失败：' + (e.message || e)
    closeQuietly()
  }
  return lines
}

export function close() {
  closeQuietly()
}

export default {
  isSupported, isConnected, getLastError,
  listBonded, connect, send, poll, close
}
