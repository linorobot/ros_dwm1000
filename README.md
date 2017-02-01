#uwb_localization

ROS Indoor/Outdoor Positioning System framework(ROS) based on Decawave's DWM1000 Ultra Wide Band transceivers. 
The whole system was tested indoors with 3 anchors and 1 tag. Technically this should work outdoors as well. 

[![IMAGE ALT TEXT](http://img.youtube.com/vi/BuBnmrkJ9BY/maxresdefault.jpg)](https://www.youtube.com/embed/BuBnmrkJ9BY "ROS Indoor Localization using Decawave's UWB Transceivers ")

###Hardware
Both anchors and tags use the same circuit board. The board houses the UWB transceiver, an Arduino Pro Mini 3.3V and a built-in USB 
Male so you can plug it straight to a power bank and deploy the anchors remotely.

1. If you wish to fabricate the PCB, the Gerber files are included /hardware/pcb/fabricate. 
Seeedstudio's DRU for 2-layer board was used to verify the design.

2. Download the DWM1000 Arduino library https://github.com/thotro/arduino-dw1000 .

3. The current DWM1000 Arduino library  uses random address for the anchors and doesn't support arbitrary addresses. 
You need to edit https://github.com/thotro/arduino-dw1000/blob/master/src/DW1000Ranging.cpp#L164-L165 every time you upload the 
anchor codes to have unique address per anchor from:

    ```c++
    _currentShortAddress[0] = random(0, 256);
    _currentShortAddress[1] = random(0, 256);
    ```
    to the address you want (ie. address: '01'):

    ```c++
    _currentShortAddress[0] = 1;
    _currentShortAddress[1] = 0;
    ```

4. Save DW1000Ranging.cpp and upload the codes from /hardware/arduino to your tag and anchors.

###Installation
1. Install dependencies

    ```sh
    $ pip install localization
    $ pip install scipy
    $ pip install shapely
    ```

2. Install map_server

    ```sh
    $ sudo apt-get install ros-indigo-map-server
    ```

3. Build the package

    ```sh
    $ cd ~/catkin_ws
    $ catkin_make
    ```

###Defining TF
1. You need to define the transforms from map to each anchor's frame. You can run map_server and load your map to identify 
the exact points you want your anchors placed. Open RVIZ and click on the location of each anchor by using "Publish Points".

2. Once you get the exact location of each anchor, add the transforms in /launch/tf.launch file. Take note that the frame_id of 
each anchor must be the same as the address defined in Arduino (_currentShortAddress[]).

###Usage
1. Run lips.launch:			

    ```sh
    $ roslaunch uwb_localization localize.launch
    
    ```

###Parameters
#####serial_port(default: '/dev/ttyUSB0')
Tag's serial port.

#####frame_id(default: 'uwb_tag')
Tag's frame_id when transform's published from map to tag.

#####min_anchor(default: 3)
The mininum number of anchors the system must find before performing trilateration between anchors. Increase this if you want to improve the accuracy
at the expense of computational cost.

#####min_range(default: 0.5)
The minimum probable distance from an anchor to a tag to prevent false reading. For example, if your anchor is hanging 1 m away from 
the floor, it makes sense to put 1 meter as minimum range since there's no way it will range less than 1 meter.

#####max_range(default: 10.0)
The maximum probable distance from an anchor to a tag to prevent false reading. For example, if your anchors are placed in 8 meter x 8 meter room, 
you can set the max_range less than or equal the hypotenuse of the room since ranges won't get any longer than that.

