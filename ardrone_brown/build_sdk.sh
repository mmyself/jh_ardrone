#!/bin/sh

cd ARDroneLib/Soft/Build && make clean && make USE_LINUX=yes && cp -f targets_versions/*/*.a ../../../lib/
cd ../../../
