#!/usr/bin/env bash
#install toolkit

function exit_for {
    echo $1
    exit
}



dataSourceIP=syh-tj-ubuntu.local
scriptDir=$(cd `dirname $0`;pwd)
workspaceDir=$scriptDir/workspace
buildDir=$scriptDir/build
dataDir=$scriptDir/data
mapDir=$dataDir/map
GPSDir=$dataDir/GPS
trajDir=$dataDir/Traj
nprocessor=$(cat /proc/cpuinfo | grep processor | wc -l)

cd $scriptDir
[ -d $workspaceDir ] || mkdir -p $workspaceDir
[ -d $trajDir ] || mkdir -p $trajDir
[ -d $GPSDir ] || mkdir -p $GPSDir
[ -d $buildDir ] || mkdir -p $buildDir
if ! [ -d $mapDir ]
then
    mkdir -p $mapDir
    cd $mapDir
    wget $dataSourceIP/map.tar.bz2 -O map.tar.bz2 || exit_for 'unable to download map'
    tar -xvf map.tar.bz2
fi

sudo umount $GPSDir
sudo umount $trajDir
sudo mount -o ro -t nfs $dataSourceIP://var/GPS $GPSDir
sudo mount -o rw -t nfs $dataSourceIP://var/Traj $trajDir
