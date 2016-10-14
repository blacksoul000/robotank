# robotank

Rpi:
* Enable i2c and camera in raspi-config

* Disable serial console.
  In file /boot/cmdline.txt remove options referring to the serial port options referring to the serial port.
  So, this
    dwc_otg.lpm_enable=0 console=ttyAMA0,115200 kgdboc=ttyAMA0,115200 console=tty1 root=/dev/mmcblk0p2 rootfstype=ext4 elevator=deadline rootwait
  becomes this
    dwc_otg.lpm_enable=0 console=tty1 root=/dev/mmcblk0p2 rootfstype=ext4 elevator=deadline rootwait

  And disable service: sudo systemctl disable serial-getty@ttyAMA0.service
  Reboot.

* Speedupt i2c interface and make it permanent
  sudo bash -c "echo options i2c_bcm2708 baudrate=400000 > /etc/modprobe.d/i2c.conf"

* Connect DualShock4 via bluetooth
  sudo apt-get install bluetooth bluez-utils blueman bluez python-gobject python-gobject-2
  run blueman-manager and connect device

* Install opencv-3.0.0 with contrib modules https://github.com/Itseez/opencv_contrib

* Compile ros using manual http://wiki.ros.org/android_ndk/Tutorials/BuildingNativeROSPackages via opencv3.0.0 or
  rebuild cv_bridge and libcompressed_image_transport image_transport and compressed_depth_image_transport with opencv 3.0.0

TODO
- add KCF tracker

