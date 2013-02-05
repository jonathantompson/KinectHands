 #!/bin/sh -e
unzip drivers_mac.zip

NITE_INST="NITE-Bin-MacOSX-v1.4.2.4"
OPENNI_INST="OpenNI-Bin-Dev-MacOSX-v1.4.0.2"
SENSOR_INST="Sensor-Bin-MacOSX-v5.1.0.25"

# uninstall first, just in case
cd ./${NITE_INST}
./uninstall.sh
cd ../${SENSOR_INST}
./install.sh -u
cd ../${OPENNI_INST}
./install.sh -u

#now install
cd ../${OPENNI_INST}
./install.sh
echo "******************************************************************"
echo "   If asked for a license key use: 0KOIk2JeIBYClPWVnMoRKn5cdY4="
echo "******************************************************************"
echo
cd ../${NITE_INST}
./install.sh
cd ../${SENSOR_INST}
./install.sh

cd ../

sudo port install libusb-devel +universal

echo "   Installing zmq... "
cp ./zmq/include/* /usr/local/include
cp ./zmq/lib/*   /usr/local/lib
