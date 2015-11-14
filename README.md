# robotank
install opencv-3.0.0 with contrib modules
rebuild cv_bridge and libcompressed_image_transport with opencv 3.0.0

http://wiki.ros.org/android_ndk/Tutorials/BuildingNativeROSPackages

export ANDROID_SDK_ROOT=~/workspace/android/android-sdk-linux
export ANDROID_NDK_ROOT=...
/opt/Qt/5.5/android_armv7/bin/qmake -spec android-g++
make -j3
make install INSTALL_ROOT=/home/blacksoul/workspace/robotank/src/robo_gui/jni/android/
/opt/Qt/5.5/android_armv7/bin/androiddeployqt --output android --verbose --input android-librobotank.so-deployment-settings.json