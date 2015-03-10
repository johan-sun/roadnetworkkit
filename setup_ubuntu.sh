#!/usr/bin/env bash
#install toolkit

dataSourceIP=syh-tj-ubuntu.local

which cmake || sudo apt-get install -y cmake
which showmount || sudo apt-get install -y nfs-common
find /usr/include/shapefil.h || sudo apt-get install -y libshp-dev
[ -d /usr/local/include/boost ] || sudo apt-get install -y libboost1.55-all-dev || sudo apt-get install -y libboost-all-dev

git submodule update --init

#mkdirs
scriptDir=$(cd `dirname $0`;pwd)
workspaceDir=$scriptDir/workspace
dataDir=$scriptDir/data
GPSDir=$dataDir/GPS
buildDir=$(mktemp -d /var/tmp/roadnetworkbuild.XXXXXX)
trajDir=$dataDir/traj
[ -d $workspaceDir ] || mkdir -p $workspaceDir
[ -d $trajDir ] || mkdir -p $trajDir
[ -d $GPSDir ] || mkdir -p $GPSDir

cd $buildDir
cmake $scriptDir
nprocessor=$(cat /proc/cpuinfo | grep processor | wc -l)
make install -j$nprocessor
sudo mount -o ro -t nfs $dataSourceIP://var/GPS $GPSDir
sudo mount -o rw -t nfs $dataSourceIP://var/Traj $trajDir
