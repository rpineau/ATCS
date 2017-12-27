#!/bin/bash

mkdir -p ROOT/tmp/ATCS_X2/
cp "../ATCS.ui" ROOT/tmp/ATCS_X2/
cp "../mountlist ATCS.txt" ROOT/tmp/ATCS_X2/
cp "../Astrometric.png" ROOT/tmp/ATCS_X2/
cp "../build/Release/libATCS.dylib" ROOT/tmp/ATCS_X2/

if [ ! -z "$installer_signature" ]; then
# signed package using env variable installer_signature
pkgbuild --root ROOT --identifier org.rti-zone.ATCS_X2 --sign "$installer_signature" --scripts Scripts --version 1.0 ATCS_X2.pkg
pkgutil --check-signature ./ATCS_X2.pkg
else
pkgbuild --root ROOT --identifier org.rti-zone.ATCS_X2 --scripts Scritps --version 1.0 ATCS_X2.pkg
fi

rm -rf ROOT
