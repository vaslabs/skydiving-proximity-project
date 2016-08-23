#!/bin/bash
set -e
[ -f build.conf ] || {
    echo "dependencies file is missing"
    exit 1
}
work_dir=$(mktemp -d)
cd $work_dir
git clone --depth=1 git@github.com:vaslabs/arduino_repo.git
[ -d $work_dir/arduino_repo ] || { 
        echo "Failed to download dependency repo in $work_dir/arduino_repo"
        exit 1 
    }
rm -rf libraries/*
cd -
[ -d libraries ] || mkdir libraries
source build.conf
for dependency in $dependencies; do
    cp $work_dir/arduino_repo/${dependency}.zip libraries/${dependency}.zip
done
