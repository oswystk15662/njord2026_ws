# ブロッカー: ZED SDK が GPU/CUDA 未検出で起動不可（ホスト側/要sudo）

日時: 2026-07-12 / 記録者: メインClaude（実機起動確認中に発見）

## 事象
`ros2 launch zed2i_driver zed2i.launch.py`(既定=SDKモード)で ZED SDK ノードがクラッシュ。
```
libnvrm_gpu.so: NvRmGpuLibOpen failed, error=4
sl::Mat::alloc : Err [100]: no CUDA-capable device is detected.
[ZED][ERROR] No NVIDIA graphics card detected.
[ZED][ERROR] NO GPU DETECTED in sl::Camera::open()
[FATAL] Failed to open ZED camera: NO GPU DETECTED   → process exit -6
```

## 診断(計測結果)
- `nvidia-smi` / `nvidia-smi -L` も `NvRmGpuLibOpen failed, error=4` → `No devices found`。**userspace から iGPU が全く見えない**。
- `/proc/driver/nvidia/gpus/` が **空**（登録GPUゼロ）。NVIDIA Open Kernel Module r595 はロード済み。
- `/dev/nvgpu/igpu0/` に **`power` ノードのみ**存在。GPU計算ノード（channel/ctrl/as/dbg/tsg 等）が無い。
- カーネルモジュール: `nvidia`(open, refcount5) と 旧 `nvgpu`(refcount0) が **両方ロード**（混在/移行不整合の疑い）。
- 環境: bare-metal Jetson Orin Nano Super、L4T **R39 (JetPack 7 相当)**, CUDA **13.2**, ZED SDK **5.4.0**。コンテナではない(systemd PID1)。uptime 8.5h。
- 関連worktree存在: `zed-sdk-jetpack-installer`, `zed2i-cuda-auto` → 同種のGPU/CUDA/JetPack整備が進行中の可能性。

## 判定
- **ドライバ package のコード/launch は正常**（back_cam は同手順で正常配信、SDKノードのビルド・パラメータ・トピック生成も正常）。
- ボトルネックは **ホストの GPU ドライバが iGPU を登録できていない**こと。ROS/コード側では解決不能。**要sudo/ホスト作業**。
- ユーザ仮説「本体の設定が悪い」は **USB ではなく GPU ドライバ**に該当。

## CPUモードも即戦力にならない(補足)
`mode:=cpu`(zed2i_cpu_node)は `/dev/video0` は開けるが `/dev/video1` の open に失敗:
`CPU stereo mode needs both left_device and right_device to be readable`。
理由: ZED 2i はステレオ対を**単一UVCデバイス(左右横並び1フレーム)**として公開し、左右別デバイス前提の cpu_stereo_node と不一致。
→ CPUフォールバックをZEDで使うには単一デバイスを開いてフレーム分割する node 改修が必要(別タスク)。当面は SDK モード復旧が本線。

## ユーザ向け sudo リスト(実行して出力を共有 → メインClaudeが次手を判断)
### 手順1: まずGPU登録失敗の理由を採取(診断)
```
sudo dmesg | grep -iE "nvgpu|nvidia|nvrm|gsp|firmware|igpu|gpu" | tail -80
```
### 手順2: 一番安いリセットを試す(移行時のモジュール混在解消を期待)
```
sudo reboot
```
再起動後、sudo無しで以下を確認し結果共有:
```
nvidia-smi -L
ls -l /dev/nvgpu/igpu0/
cat /proc/driver/nvidia/gpus/*/information 2>/dev/null || echo "still no GPU registered"
```
### 手順3: 手順2で復旧しない場合(dmesgの出力を見て判断)
- iGPU が `nvidia`(open) と `nvgpu`(legacy) のどちらで駆動されるべきかを確定し、正しい L4T GPU ドライバ package 構成へ是正(JetPack 7 のドキュメント/`zed-sdk-jetpack-installer` worktree の手順に依存)。具体是正コマンドは手順1の dmesg 結果を見てから提示。

## 確定根本原因 (2026-07-12, dmesg採取 + Codex Sol 相談)

kernel ログ(journalctl -k, **毎ブート決定的に再現**)で原因が確定:
```
nvgpu: 17000000.gpu ga10b_acr_patch_wpr_info_to_ucode: invalid mem acr_falcon2_sysmem_desc
nvgpu: 17000000.gpu nvgpu_acr_bootstrap_hs_ucode_riscv: RISCV ucode patch wpr info failed
nvgpu: 17000000.gpu ga10b_bootstrap_hs_acr / nvgpu_acr_bootstrap_hs_acr: ACR bootstrap failed
nvgpu: 17000000.gpu nvgpu_acr_construct_execute: Bootstrap HS ACR failed
nvgpu: 17000000.gpu nvgpu_finalize_poweron: Failed initialization for: g->ops.acr.acr_construct_execute
```
= **iGPU(ga10b/Orin)のセキュア初期化 firmware(ACR HS/RISCV ucode)のブートストラップ失敗**。
GPUが poweron を finalize できず、計算ノード生成・CUDA登録に至らない → `NO GPU DETECTED`。

原因層 = **ZED/CUDA/ROS ユーザ空間ではなく、kernel driver ↔ nvgpu firmware ↔ OpenRM/legacy nvgpu ↔ DTB/boot の整合不良**(GPU初期化層)。
BSPは dpkg 上すべて 39.2.0 で揃うが、実行 kernel `6.8.12-1021-tegra`(oot variant)と module/firmware の実体整合が崩れている疑い。

## Codex Sol 判定 (gpt-5.5, read-only)
- **再起動だけでは直らない**(毎ブート決定的に失敗)。ただし是正後の反映には reboot 必須。
- **非sudo/コード側でできる対処は無い**。ZED/CUDA/ROS を触っても無意味。**打ち切り妥当**。
- 是正方向(リスク低→高, すべて要sudo):
  1. boot実体確認: `uname -a`, `cat /etc/nv_tegra_release`, `cat /boot/extlinux/extlinux.conf`, `ls -l /boot/Image /boot/initrd /boot/*.dtb`, `lsmod | grep -E 'nvidia|nvgpu'`
  2. module 提供元確認: `modinfo nvidia; modinfo nvgpu; dpkg -S $(modinfo -n nvidia); dpkg -S $(modinfo -n nvgpu)`
  3. L4T 再導入 + initramfs + reboot:
     `sudo apt install --reinstall nvidia-l4t-core nvidia-l4t-kernel nvidia-l4t-kernel-dtbs nvidia-l4t-firmware nvidia-l4t-firmware-nvgpu nvidia-l4t-firmware-openrm && sudo update-initramfs -u -k all && sudo reboot`
  4. firmware path 確認: cmdline に `firmware_class.path=/etc/firmware` あり → `ls -l /etc/firmware`, `ls -l /lib/firmware/nvidia/ga10b`, `journalctl -k -b | grep -i firmware`
  5. 最終手段: JetPack 7 / L4T R39.2 **reflash**(破壊的, rootfsバックアップ後)。
- 是正後に `/proc/driver/nvidia/gpus/` と `nvidia-smi` が復活してから ZED 作業へ戻る。

## ステータス (最終)
- back_cam: **完了(実機配信確認済み)**。カメラプロセス全停止済 → ユーザが単独確認可能。
- ZED: **コード側完了。実機はGPU firmware(ACR)ブロッカーで、コード/非sudo層では対処不能 → 打ち切り**。
  復旧は上記sudo是正(reinstall→reboot、最悪reflash)を要する **ホスト/BSP工程**。復旧後にメインClaudeがZED実配信確認を実施する。
