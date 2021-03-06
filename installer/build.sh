#!/bin/bash

PACKAGE_NAME="ATCS_X2.pkg"
BUNDLE_NAME="org.rti-zone.ATCSX2"

if [ ! -z "$app_id_signature" ]; then
    codesign -f -s "$app_id_signature" --verbose ../build/Release/libATCS.dylib
fi

mkdir -p ROOT/tmp/ATCS_X2/
cp "../ATCS.ui" ROOT/tmp/ATCS_X2/
cp "../mountlist ATCS.txt" ROOT/tmp/ATCS_X2/
cp "../Astrometric.png" ROOT/tmp/ATCS_X2/
cp "../build/Release/libATCS.dylib" ROOT/tmp/ATCS_X2/

if [ ! -z "$installer_signature" ]; then
	# signed package using env variable installer_signature
	pkgbuild --root ROOT --identifier $BUNDLE_NAME --sign "$installer_signature" --scripts Scripts --version 1.0 $PACKAGE_NAME
	pkgutil --check-signature ./${PACKAGE_NAME}
else
	pkgbuild --root ROOT --identifier $BUNDLE_NAME --scripts Scripts --version 1.0 $PACKAGE_NAME
fi

rm -rf ROOT
