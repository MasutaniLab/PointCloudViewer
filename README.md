# RTC:PCL 仕様の点群ビューア

大阪電気通信大学  
升谷 保博  
2021年3月25日

## 概要

- RTC:PCLのPointCloud型（`PointCloudTypes::PointCoud`）のデータを入力しビューアで表示するRTコンポーネントです．
- 入力される点群を表す座標系は，y軸は上向きが正，z軸は後ろ向きが正です．
- 各点の色の並び順のデフォルトはRGBです（2021年3月25日にBGRからRGBに変更）．
- [`pointcloud.idl`](idl/pointcloud.idl) は，Geoffrey Biggs (gbiggs)氏の
[RT-Components for the Point Cloud Library](https://github.com/gbiggs/rtcpcl/)
に[含まれているもの](https://github.com/gbiggs/rtcpcl/blob/master/pc_type/pointcloud.idl)
をそのまま使っています．
- 以下の環境で開発，動作確認しています．
  - Windows 10 64bit版
  - Visual Studio 2019 x64
  - OpenRTM-aist 1.2.2 64bit版
  - Point Cloud Library 1.11.1

## インストール（Windowsの場合）

- [OpenRTM-aist](https://www.openrtm.org/)をインストール．
- [Point Cloud Library](https://github.com/PointCloudLibrary/pcl)をインストール．
  - [GitHubのpclのRelease](https://github.com/PointCloudLibrary/pcl/releases)の中のWindows用AllInOne `PCL-X.X.X-AllInOne-msvcXXXX-winXX.exe`をダウンロードし実行．
- [PointCloudViewer](https://github.com/MasutaniLab/PointCloudViewer)をクローン．
- CMake GUI
  - Configure
  - Generate
  - Open Project
- Visual StudioでRelease x64でソリューションをビルド．

## 仕様

### 入力データポート

- pc
  - 型： PointCloudTypes::PointCoud
  - 意味： 点群データ

### 出力データポート

なし

### サービスポート

なし

### コンフィギュレーション

- rotX
  - 型： double
  - デフォルト値：0.0
  - 意味： センサ座標系のx軸回転角度 [deg]              

- rotY
  - 型： double
  - デフォルト値：0.0
  - 意味： センサ座標系のy軸回転角度 [deg]              

- rotZ 
  - 型： double
  - デフォルト値：0.0
  - 意味： センサ座標系のz軸回転角度 [deg]              

- transX
  - 型： double
  - デフォルト値：0.0
  - 意味： センサ座標系のx成分 [m]              

- transY
  - 型： double
  - デフォルト値：0.0
  - 意味： センサ座標系のy成分 [m]              

- transZ
  - 型： double
  - デフォルト値：0.0
  - 意味： センサ座標系のz成分 [m]              

- colorOrder
  - 型：string
  - デフォルト値：RGB
  - 意味： 各点のデータの色の並び順（RGBまたはBGR）

## 履歴

- 2021年3月25日 色の並び順のデフォルトをBGRからRGBへ変更．コンフィギュレーションcolorOrderの追加
