#!/bin/sh
SCRIPT_DIR=`dirname $0`
cd $SCRIPT_DIR

if [ "$1" = "" ]; then
	echo "Argument required: Lazuli/LazuliSensor/Zirconia2kai"
elif [ "$1" = "Lazuli" ]; then
	echo "Config switched to Lazuli"
	cp ../.vscode/c_cpp_properties.json.lazuli ../.vscode/c_cpp_properties.json
	cp ../.vscode/launch.json.lazuli ../.vscode/launch.json
	rm -R ../build
elif [ "$1" = "LazuliSensor" ]; then
	echo "Config switched to LazuliSensor"
	cp ../.vscode/c_cpp_properties.json.lazulisensor ../.vscode/c_cpp_properties.json
	cp ../.vscode/launch.json.lazulisensor ../.vscode/launch.json
	rm -R ../build
elif [ "$1" = "Zirconia2kai" ]; then
	echo "Config switched to Zirconia2kai"
	cp ../.vscode/c_cpp_properties.json.zirconia2kai ../.vscode/c_cpp_properties.json
	cp ../.vscode/launch.json.zirconia2kai ../.vscode/launch.json
	rm -R ../build
else
	echo "Argument required: Lazuli/LazuliSensor/Zirconia2kai"
fi

