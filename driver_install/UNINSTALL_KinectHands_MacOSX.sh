 #!/bin/sh -e
NITE_INST="NITE-Bin-MacOSX-v1.4.1.2"
OPENNI_INST="OpenNI-Bin-MacOSX-v1.3.2.3"
SENSOR_INST="SensorKinect-Bin-MacOSX-v5.0.3.4"

# uninstall
cd ./${NITE_INST}
./uninstall.sh
cd ../${SENSOR_INST}
./install.sh -u
cd ../${OPENNI_INST}
./install.sh -u

cd ../..