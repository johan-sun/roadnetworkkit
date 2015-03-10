#!/usr/bin/env bash
#install toolkit

function exit_for {
    echo $1
    exit
}

dataSourceIP=syh-tj-ubuntu.local
scriptDir=$(cd `dirname $0`;pwd)
nprocessor=$(cat /proc/cpuinfo | grep processor | wc -l)
which clang || sudo apt-get install -y clang || exit_for 'unable to install clang'
which cmake || sudo apt-get install -y cmake || exit_for 'unable to install cmake'
which showmount || sudo apt-get install -y nfs-common || exit_for 'unable to install nfs client'
find /usr/include/shapefil.h || sudo apt-get install -y libshp-dev || exit_for 'unbale to install libshp'
if ! [ -d /usr/local/include/boost ]
then
    boostTmp=$(mktemp -d /var/tmp/boost1.57.XXXXXX)
    cd $boostTmp
    wget syh-tj-ubuntu.local/boost_1_57_0.tar.bz2 || exit_for 'unable to download boost 1.57'
    tar -xvf boost_1_57_0.tar.bz2
    cd boost_1_57_0
    ./bootstrap.sh --prefix=/usr/local/
    ./b2 -j$nprocessor
    sudo ./b2 install
    cd $scriptDir
fi

git submodule update --init || exit_for 'unable to update submodule'
#mkdirs
workspaceDir=$scriptDir/workspace
dataDir=$scriptDir/data
GPSDir=$dataDir/GPS
buildDir=$(mktemp -d /var/tmp/roadnetworkbuild.XXXXXX)
trajDir=$dataDir/traj
[ -d $workspaceDir ] || mkdir -p $workspaceDir
[ -d $trajDir ] || mkdir -p $trajDir
[ -d $GPSDir ] || mkdir -p $GPSDir

cd $buildDir
CC=clang CXX=clang++ cmake $scriptDir -DCMAKE_BUILD_TYPE=Release
make install -j$nprocessor
cd $workspaceDir

sudo mount -o ro -t nfs $dataSourceIP://var/GPS $GPSDir
sudo mount -o rw -t nfs $dataSourceIP://var/Traj $trajDir
