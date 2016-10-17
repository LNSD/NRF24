Build instructions
---------------

The following instructions are for **\*nix** type systems, specifically this is a Linux example.

In short you can get up and running using the following commands::

    mkdir build
    cd build
    cmake ..
    make
    make upload              # to upload all firmware images             [optional]
    make blink-serial  # to get a serial terminal to wire_serial   [optional]

For a more detailed explanation, please read on...


1. Creating a build directory

   The second order of business is creating a build directory. CMake has a great feature called out-of-source builds, what this means is the building is done in a completely separate directory from where the sources are. The benefit of this is you don't have any clutter in you source directory and you won't accidentally commit something that is auto-generated.

   So let's create that build directory::

        mkdir build
        cd build

2. Creating the build system

   Now let's create the build system that will create our firmware::

        cmake ..

   To specify the build system type, use the ``-G`` option, for example::

        cmake -G"Eclipse CDT4 - Unix Makefiles" ..

   If you rather use a GUI, use::

        cmake-gui ..

3. Building

   Next we will build everything::

        make

4. Uploading (optional)

   Once everything built correctly we can upload. Depending on your Arduino you will have to update the serial port used for uploading the firmware. To change the port please edit the following variable in *CMakeLists.txt*::

        set(${FIRMWARE_NAME}_PORT /path/to/device)

   Ok lets do a upload of all firmware images::

        make upload

   If you have an upload sync error then try resetting/ power cycling the board before starting the upload process.

5. Serial output (optional)

   If you have some serial output, you can launch a serial terminal from the build system. The command used for executing the serial terminal is user configurable by the following setting::

        set(${FIRMWARE_NAME}_SERIAL serial command goes here)

   In order to get access to the serial port use the following in your command::

        @SERIAL_PORT@

   That constant will get replaced with the actual serial port used (see uploading). In the case of our example configuration we can get the serial terminal by executing the following::

        make blink-serial
