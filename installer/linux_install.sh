#!/bin/bash

TheSkyX_Install=~/Library/Application\ Support/Software\ Bisque/TheSkyX\ Professional\ Edition/TheSkyXInstallPath.txt
echo "TheSkyX_Install = $TheSkyX_Install"

if [ ! -f "$TheSkyX_Install" ]; then
    echo TheSkyXInstallPath.txt not found
    exit 1
fi


TheSkyX_Path=$(<"$TheSkyX_Install")
echo "Installing to $TheSkyX_Path"

if [ ! -d "$TheSkyX_Path" ]; then
    echo TheSkyX Install dir not exist
    exit 1
fi

cp "./mountlist ATCS.txt" "$TheSkyX_Path/Resources/Common/Miscellaneous Files/"
cp "./ATCS.ui" "$TheSkyX_Path/Resources/Common/PlugIns/MountPlugIns/"
cp "./libATCS.so" "$TheSkyX_Path/Resources/Common/PlugIns/MountPlugIns/"

app_owner=`/usr/bin/stat -c "%u" "$TheSkyX_Path" | xargs id -n -u`
if [ ! -z "$app_owner" ]; then
	chown $app_owner "$TheSkyX_Path/Resources/Common/Miscellaneous Files/mountlist ATCS.txt"
	chown $app_owner "$TheSkyX_Path/Resources/Common/PlugIns/MountPlugIns/ATCS.ui"
	chown $app_owner "$TheSkyX_Path/Resources/Common/PlugIns/MountPlugIns/libATCS.so"
fi
chmod  755 "$TheSkyX_Path/Resources/Common/PlugIns/MountPlugIns/libATCS.so"
