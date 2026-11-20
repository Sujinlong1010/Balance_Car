<script>
import SPP from './common/spp.js'

export default {
  onLaunch() {
    // Android 12+ 访问蓝牙必须拿到运行时权限，拿不到就永远连不上。
    // 放在启动时申请一次，用户拒绝的话连接时会明确提示。
    if (typeof plus !== 'undefined' && plus.android) {
      try {
        plus.android.requestPermissions(
          [
            'android.permission.BLUETOOTH_CONNECT',
            'android.permission.BLUETOOTH_SCAN'
          ],
          function () { /* 结果在连接时再校验 */ },
          function () { /* 老系统没有这些权限名，忽略 */ }
        )
      } catch (e) { /* ignore */ }
    }
    // 冷启动就断开，避免上一次残留的 socket 占着通道
    SPP.close()
  },
  onHide() {
    // 切后台：让车的失联保护生效（心跳停 600 ms 自动停车）
    SPP.close()
  }
}
</script>

<style>
/* 全局深色主题：车上/实验室都看得清，也省电 */
page {
  background-color: #0b1016;
  color: #e6edf3;
  font-family: -apple-system, "Helvetica Neue", "PingFang SC", "Microsoft YaHei", sans-serif;
}
</style>
