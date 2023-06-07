# 開発環境構築

## インストール

- OpenOCD  
  https://openocd.org/  
  インストール後、bin ディレクトリを PATH に追加
- Visual Studio Code  
  https://code.visualstudio.com/
- Docker  
  https://hub.docker.com/editions/community/docker-ce-desktop-windows/

## ファイルのコピー

- gcc-arm-11.2-2022.02-x86_64-arm-none-eabi.tar.xz
  ルートディレクトリにコピーする
  https://developer.arm.com/-/media/Files/downloads/gnu/11.2-2022.02/binrel/gcc-arm-11.2-2022.02-x86_64-arm-none-eabi.tar.xz
- STM32CubeMX_Generate_20220405.zip
  Generate ディレクトリ以下に展開する
  共有サーバにあり

## ビルド

- taniho を Linux 向けにビルド

```
cmake . -B build -DCMAKE_BUILD_TYPE=Debug -DPLATFORM=Linux
make
```
