#!/bin/bash

mkdir -p ROOT/tmp/efa_X2/
cp "../efa.ui" ROOT/tmp/efa_X2/
cp "../focuserlist efa.txt" ROOT/tmp/efa_X2/
cp "../build/Release/libefa.dylib" ROOT/tmp/efa_X2/

if [ ! -z "$installer_signature" ]; then
# signed package using env variable installer_signature
pkgbuild --root ROOT --identifier org.rti-zone.efa_X2 --sign "$installer_signature" --scripts Scripts --version 1.0 efa_X2.pkg
pkgutil --check-signature ./efa_X2.pkg
else
pkgbuild --root ROOT --identifier org.rti-zone.efa_X2 --scripts Scripts --version 1.0 efa_X2.pkg
fi

rm -rf ROOT
