#!/bin/bash

# Uninstall all packages.

LIBNAME=pyhackrf
HACKRFDIR="$(pwd)/hackrf"
HACKRFHOSTDIR="${HACKRFDIR}/host"

CURRENTDIR=${PWD##*/}
if [ "${CURRENTDIR}" = ${LIBNAME} ]; then
    echo "Preparing to uninstall '${LIBNAME}'..."
else
    echo "ERROR: Must perform uninstall from 'pip install' location."
    exit -1
fi

sudo pip uninstall ${LIBNAME}

echo ${CURRENTDIR}

echo $(pwd)

if [ -d "$HACKRFHOSTDIR" ]; then
    echo "    Removing 'libhackrf' scripts, objects, and configurations."
    cd ${HACKRFHOSTDIR}
    if [ -d "$HACKRFHOSTDIR/build" ]; then
        echo "        Found build folder. Uninstalling..."
        cd "${HACKRFHOSTDIR}/build"
        sudo make uninstall
        cd ../
        sudo rm -rf build
    fi

    cd ${HACKRFDIR}/..
fi
