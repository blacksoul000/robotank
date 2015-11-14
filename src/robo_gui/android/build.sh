#!/bin/bash

export ANDROID_SDK_ROOT=~/workspace/android/android-sdk-linux
export ANDROID_NDK_ROOT=~/workspace/android/android-ndk-r10e

/opt/Qt/5.5/android_armv7/bin/qmake -spec android-g++ android.pro
make -j3 install INSTALL_ROOT=android
/opt/Qt/5.5/android_armv7/bin/androiddeployqt --output android --verbose --input android-librobotank.so-deployment-settings.json