#!/bin/bash

TheSkyX_Install=`/usr/bin/find ~/Library/Application\ Support/Software\ Bisque/ -name TheSkyXInstallPath.txt`
echo "TheSkyX_Install = $TheSkyX_Install"

if [ ! -f "$TheSkyX_Install" ]; then
    echo TheSkyXInstallPath.txt not found
    TheSkyX_Path=`/usr/bin/find ~/ -maxdepth 3 -name TheSkyX`
    if [ -d "$TheSkyX_Path" ]; then
		TheSkyX_Path="${TheSkyX_Path}/Contents"
    else
	   echo TheSkyX application was not found.
    	exit 1
	 fi
else
	TheSkyX_Path=$(<"$TheSkyX_Install")
fi

echo "Installing to $TheSkyX_Path"


if [ ! -d "$TheSkyX_Path" ]; then
    echo TheSkyX Install dir not exist
    exit 1
fi

cp "./mountlist ATCS.txt" "$TheSkyX_Path/Resources/Common/Miscellaneous Files/"
cp "./ATCS.ui" "$TheSkyX_Path/Resources/Common/PlugInsARM32/MountPlugIns/"
cp "./Astrometric.png" "$TheSkyX_Path/Resources/Common/PlugInsARM32/MountPlugIns/"
cp "./libATCS.so" "$TheSkyX_Path/Resources/Common/PlugInsARM32/MountPlugIns/"

app_owner=`/usr/bin/stat -c "%u" "$TheSkyX_Path" | xargs id -n -u`
if [ ! -z "$app_owner" ]; then
	chown $app_owner "$TheSkyX_Path/Resources/Common/Miscellaneous Files/mountlist ATCS.txt"
	chown $app_owner "$TheSkyX_Path/Resources/Common/PlugInsARM32/MountPlugIns/ATCS.ui"
	chown $app_owner "$TheSkyX_Path/Resources/Common/PlugInsARM32/MountPlugIns/Astrometric.png"
	chown $app_owner "$TheSkyX_Path/Resources/Common/PlugInsARM32/MountPlugIns/libATCS.so"
fi
chmod  755 "$TheSkyX_Path/Resources/Common/PlugInsARM32/MountPlugIns/libATCS.so"

