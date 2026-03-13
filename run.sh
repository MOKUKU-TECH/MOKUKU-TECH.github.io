#!/usr/bin/env bash
set -e


# amd or arm
if [ $# -ne 1 ]; then
    echo "Build Incremental"
    bundle exec jekyll serve --incremental --host ${HOST}
else
    ARCH=$1
    echo $ARCH
    if [ $ARCH == "rebuild" ]; then
        echo "Build Rebuild"
        bundle exec jekyll serve --host ${HOST}
    else
        bundle exec jekyll serve --incremental --host ${HOST}
    fi
fi
