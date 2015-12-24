# robotank
install opencv-3.0.0 with contrib modules
rebuild cv_bridge and libcompressed_image_transport with opencv 3.0.0

compile ros using manual
http://wiki.ros.org/android_ndk/Tutorials/BuildingNativeROSPackages

connect DualShock4 via bluetooth using manual
https://pypi.python.org/pypi/ds4drv

trust device
bluetoothctl -a
[bluetooth]# scan on
[bluetooth]# trust <MAC>
[bluetooth]# pair <MAC>
