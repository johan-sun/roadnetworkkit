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

git submodule update --init || exit_for 'unable to update submodule'

which clang || sudo apt-get install -y clang || exit_for 'unable to install clang'
which cmake || sudo apt-get install -y cmake || exit_for 'unable to install cmake'
which showmount || sudo apt-get install -y nfs-common || exit_for 'unable to install nfs client'
find /usr/include/shapefil.h || sudo apt-get install -y libshp-dev || exit_for 'unbale to install libshp'

if ! [ -d /usr/local/include/boost ]
then
    boostTmp=$buildDir
    cd $boostTmp
    wget $dataSourceIP/boost_1_57_0.tar.bz2 -O boost_1_57_0.tar.bz2 || exit_for 'unable to download boost 1.57' 
    tar -xvf boost_1_57_0.tar.bz2
    cd boost_1_57_0
    ./bootstrap.sh --prefix=/usr/local/
    ./b2 -j$nprocessor
    sudo ./b2 install
    cd $scriptDir
fi


cd $buildDir
CC=clang CXX=clang++ cmake $scriptDir -DCMAKE_BUILD_TYPE=Release
make install -j$nprocessor
cd $workspaceDir

sudo mount -o ro -t nfs $dataSourceIP://var/GPS $GPSDir
sudo mount -o rw -t nfs $dataSourceIP://var/Traj $trajDir
