#!/usr/bin/env bash
# Construit et emballe un plugin EMX-1 : bash emx1/make.sh test2
set -euo pipefail
P="emx1/${1:?usage: bash emx1/make.sh test2}"
SHADOW_SKIN_MPC_OS=2 bash tools/build_port.sh "$P/vst.json" armv7
VENDOR=$(sed -n 's/.*"vendor": *"\([^"]*\)".*/\1/p' "$P/vst.json")
NAME=$(sed -n 's/.*"name": *"\([^"]*\)".*/\1/p' "$P/vst.json")
SO=$(sed -n 's/.*"so": *"\([^"]*\)".*/\1/p' "$P/vst.json")
rm -rf "$P/dist"
python3 tools/release.py --so "$P/build/$SO" --skin "$P/build/skin/$VENDOR - VST - $NAME" --entry "$P/build/pluginlist-entry.xml" --version 4.0.0 -o "$P/dist"
ls -l "$P/dist"
