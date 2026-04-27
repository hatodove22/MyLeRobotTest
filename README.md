# MyLeRobotTest

Windows PC 上で Cluster から OSC 送信されるエンドポイント座標を受け取り、SO101 follower arm の IK を解いて動かすための LeRobot カスタム環境です。

ベースは Hugging Face LeRobot で、SO101 の Windows 実機運用向けに以下を追加しています。

- `lerobot-cluster-ik`: Cluster OSC `/ik/target` を受信して SO101 follower を動かす CLI
- Windows の COM ポート確認用スクリプト
- SO101 follower/leader 周辺の Windows 対応変更
- `placo` が Windows で入らない場合の URDF 直読み IK フォールバック
- OSC joint teleop 関連の追加

## 環境

- Windows
- Python 3.12
- `uv`
- LeRobot local checkout
- SO101 follower arm
- Cluster から OSC で `/ik/target` を送信

## セットアップ

```powershell
Set-Location C:\Users\tesul\LeRobot
uv sync --locked --extra core_scripts --extra hardware
```

開発ツール込みで確認する場合:

```powershell
uv sync --locked --extra test --extra dev
```

## SO101 URDF

IK には SO101 の URDF が必要です。このリポジトリには URDF 本体は含めず、TheRobotStudio の SO-ARM100 repo を別途 clone して使います。

```powershell
Set-Location C:\Users\tesul
git clone https://github.com/TheRobotStudio/SO-ARM100.git
```

使用する URDF:

```text
C:\Users\tesul\SO-ARM100\Simulation\SO101\so101_new_calib.urdf
```

## Cluster OSC → SO101 IK

Cluster 側は以下の形式で送信します。

- Address: `/ik/target`
- Values: `float x, float y, float z`
- Port: デフォルト `9000`

起動コマンド:

```powershell
Set-Location C:\Users\tesul\LeRobot

uv run lerobot-cluster-ik `
  --robot.type=so101_follower `
  --robot.port=COM5 `
  --robot.id=so101_cluster `
  --robot.max_relative_target=10 `
  --urdf_path=C:/Users/tesul/SO-ARM100/Simulation/SO101/so101_new_calib.urdf `
  --target_frame_name=gripper_frame_link `
  --host=127.0.0.1 `
  --recv_port=9000
```

初回起動時に SO101 のキャリブレーションが走ります。保存先は通常:

```text
C:\Users\tesul\.cache\huggingface\lerobot\calibration\robots\so_follower\so101_cluster.json
```

同じ `--robot.id=so101_cluster` を使う限り、次回以降は保存済みキャリブレーションを使います。

## 座標系

デフォルトの変換は Cluster/Unity 想定です。

- Cluster: `x=右`, `y=上`, `z=前`
- SO101 URDF: `x=前`, `y=左`, `z=上`

そのためデフォルトは:

```text
axis_map = [z, -x, y]
```

向きが逆に感じる場合は起動オプションで調整します。

```powershell
--axis_map='[z,x,y]'
```

動きが大きすぎる場合:

```powershell
--scale=0.5
```

1 フレームの最大 EE 移動量を抑える場合:

```powershell
--max_ee_step_m=0.01
```

## COM ポート確認

```powershell
.\scripts\windows\Show-SerialPorts.ps1
```

または LeRobot 標準の対話式検出:

```powershell
uv run lerobot-find-port
```

## テスト

今回追加した Cluster IK 周辺のテスト:

```powershell
uv run pytest tests\scripts\test_lerobot_cluster_ik.py -q
```

Ruff:

```powershell
uv run --extra dev ruff check src\lerobot\scripts\lerobot_cluster_ik.py tests\scripts\test_lerobot_cluster_ik.py
```

## 注意

- `.venv` やローカルキャリブレーションキャッシュは Git 管理対象ではありません。
- `placo` は Windows で依存ビルドに失敗することがあります。この環境では `lerobot-cluster-ik` が URDF 直読み IK にフォールバックします。
- 実機を動かす前に、SO101 が机や人に干渉しない位置にあることを確認してください。
