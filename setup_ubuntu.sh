#!/usr/bin/env bash
#install toolkit

function exit_for {
    echo $1
    exit
}

dataSourceIP=syh-tj-ubuntu.local
which clang || sudo apt-get install -y clang || exit_for 'unable to install clang'
which cmake || sudo apt-get install -y cmake || exit_for 'unable to install cmake'
which showmount || sudo apt-get install -y nfs-common || exit_for 'unable to'
find /usr/include/shapefil.h || sudo apt-get install -y libshp-dev || exit_for 'unbale to install libshp'
[ -d /usr/local/include/boost ] || sudo apt-get install -y libboost1.55-all-dev || sudo apt-get install -y libboost-all-dev || exit_for 'unable to install boost'

git submodule update --init || exit_for 'unable to update submodule'

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
CC=clang CXX=clang++ cmake $scriptDir -DCMAKE_BUILDT_YPE=Release
nprocessor=$(cat /proc/cpuinfo | grep processor | wc -l)
make install -j$nprocessor
cd $workspaceDir

sudo mount -o ro -t nfs $dataSourceIP://var/GPS $GPSDir
sudo mount -o rw -t nfs $dataSourceIP://var/Traj $trajDir
