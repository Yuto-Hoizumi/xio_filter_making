# CanSat 姿勢フィルター試作

Raspberry Pi Zero 2 W 上で動かす姿勢推定の最小試作です。公式の
[x-io Fusion](https://github.com/xioTechnologies/Fusion) Python版
`imufusion`を使用します。独自アルゴリズムを作るのではなく、実績のある
フィルターをセンサー取得部分から分離して組み込みます。

一次成果物は[こちら](docs/班2_フィルタ実装_一次成果物.md)です。

## 現時点で実装した範囲

- ジャイロ＋加速度＋地磁気による9軸姿勢推定
- 地磁気が欠損したときの6軸姿勢推定への自動退避
- 外部方位入力の入口（GNSS進行方向を使う場合のため。現時点では未調整）
- ジャイロの静止時バイアス推定
- 実測したサンプル間隔の反映
- 長時間データが途切れた場合の再初期化
- 無効値、衝撃、加速度・地磁気除外、オーバーレンジ等の状態フラグ
- CSVログ処理、疑似データ生成、フィルター単体ベンチマーク
- 疑似データの真値保存と、ヨー角の平均・最大誤差の自動計算

## まだ決定していないこと

- 自作基板で使用するIMU・地磁気センサーの型番
- センサー軸から機体軸への並べ替えと符号
- 各センサー固有の感度、オフセット、ハードアイアン・ソフトアイアン補正値
- GNSS進行方向を利用する最低速度と測位品質条件
- 落下衝撃後の再初期化条件
- 最終的なゲイン・外乱除外閾値

これらは実機ログなしでは決められません。コード中の既定値は採用値ではなく、
公式推奨値を用いた試験開始値です。

## 入出力

入力単位は次のとおりです。

| 値 | 単位 |
| --- | --- |
| ジャイロ | degree/s |
| 加速度 | g |
| 地磁気 | 較正後の同一単位（通常は µT） |
| 時刻 | s、単調増加 |

座標系は暫定でNWU（X：前、Y：左、Z：上）です。センサーを基板へ載せる向きが
決まったら、入力前に機体座標へ軸変換します。

主な出力はクォータニオン、ロール・ピッチ・ヨー、重力除去後加速度、動作モード、
各種異常フラグです。制御ではオイラー角だけでなく、異常フラグも同時に確認します。

## PCでの実行

```powershell
cd xio_filter_making
python -m pip install -r requirements.txt
python generate_demo_data.py demo_raw.csv
python run_csv.py demo_raw.csv demo_filtered.csv
python -m unittest -v test_cansat_ahrs.py
python benchmark.py --samples 100000
```

疑似データのノイズ強度は変更できます。

```powershell
python generate_demo_data.py demo_raw.csv `
  --gyro-noise-std 0.03 `
  --accel-noise-std 0.002 `
  --mag-noise-std 0.005 `
  --seed 42
```

`run_csv.py`は入力に`true_yaw_deg`列がある場合、各時刻のヨー誤差と、平均絶対誤差、
二乗平均平方根誤差、最大絶対誤差を自動計算します。ここで得られる値は疑似条件に対する
精度であり、実機精度ではありません。

## Raspberry Pi Zero 2 Wでの実行

64 bit Raspberry Pi OSを第一候補とします。公式PyPIにはARM64向けwheelがあるため、
通常は次で導入できます。

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python benchmark.py --samples 100000
```

`uname -m` が `aarch64` なら64 bit、`armv7l` なら32 bitです。32 bit環境でwheelを
取得できない場合は、OS変更または公式C版のビルドを検討します。

## 実センサーへ接続するとき

`CanSatAhrs.update()`へ100 Hzを目標に1サンプルずつ渡します。センサードライバーは
この試作には含めていません。センサー型番の決定後、SPI/I2C取得処理と次の変換を
追加します。

1. 生カウントからdegree/s、g、µTへ変換
2. オフセットと感度補正
3. 基板座標から機体座標へ軸変換
4. `update()`を呼び出す
5. 入力生データと出力を同じ時刻でCSVへ保存する

## 9月24日の最小負荷試験

まず `run_filter_100hz_minimal.py`を実行し、別ターミナルの `top`でPythonプロセスの
CPU使用率とメモリ使用量を確認します。その後、AI Cameraの既存プログラムも同時に
動かし、フィルター追加前後の変化を比較します。

```bash
python run_filter_100hz_minimal.py
```

`benchmark.py`と`run_realtime.py`は、時間に余裕がある場合または詳しい調査が必要に
なった場合に使用します。
