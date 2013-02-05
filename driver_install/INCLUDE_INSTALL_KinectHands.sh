 #!/bin/sh -e
cp /usr/lib/libOpenNI.dylib ../lib/
cp /usr/lib/libXnVNite* ../lib/

rm -f -r ../include/ni
rm -f -r ../include/nite
cp -r /usr/include/ni ../include/
cp -r /usr/include/nite ../include/nite
