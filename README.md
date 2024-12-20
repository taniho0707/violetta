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
cmake . -B build -DCMAKE_BUILD_TYPE=Debug -DPLATFORM=Linux -DMOUSE=Violetta
make
```

- taniho を Zirconia2kai STM32F411xE 向けにビルド

```
cmake . -B build -DCMAKE_BUILD_TYPE=Debug -DPLATFORM=STM32 -DMOUSE=Zirconia2kai -DTARGET_GROUP=taniho -GNinja -DCMAKE_EXPORT_COMPILE_COMMANDS=on
ninja
```

- taniho を Lazuli STM32L4P5CET 向けにビルド

```
cmake . -B build -DCMAKE_BUILD_TYPE=Debug -DPLATFORM=STM32 -DMOUSE=Lazuli -DTARGET_GROUP=taniho -GNinja -DCMAKE_EXPORT_COMPILE_COMMANDS=on
ninja
```

- taniho を LazuliSensor STM32C0 向けにビルド

```
cmake . -B build -DCMAKE_BUILD_TYPE=Debug -DPLATFORM=STM32 -DMOUSE=LazuliSensor -DTARGET_GROUP=taniho -GNinja -DCMAKE_EXPORT_COMPILE_COMMANDS=on
ninja
```

- UnitTest を Zirconia2kai Linux 向けにビルド

```
cmake . -B build -DCMAKE_BUILD_TYPE=Debug -DPLATFORM=Linux -DMOUSE=Zirconia2kai -DTARGET_GROUP=Test -GNinja -DCMAKE_EXPORT_COMPILE_COMMANDS=on
ninja
```

- PlantUML の生成

```
hpp2plantuml -i "./Library/act/Inc/*" -i "./Library/cmd/Inc/*" -i "./Library/misc/Inc/*" -i "./Library/mll/Inc/*" -i "./Library/mpl/Inc/*" -i "./Library/msg/Inc/msg_server.h" -o out.puml
```
