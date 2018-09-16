# Running the pipelines on the NUC

***IF ANY ONE PIPELINE NEEDS TO RUN USING SUDO, ALL PIPELINES MUST USE SUDO;
   OTHERWISE, ZCM MESSAGES WILL NOT WORK***

Execute these from `/home/maav` in the order they are presented here. Do this 
in tmux; otherwise, you will need a new terminal for each if you ssh into the 
NUC.


## IMU Pipeline
Reads from Microstrain over USB and publishes imu\_t messages.

`sudo nav.build/atomcore/src/imu/maav-imu`


## TANFAN
Reads from Tiva over `/dev/ttyUSB0` serial port and publishes lidar\_t and 
emergency\_t messages. Reads from NUC pipelines and publishes dji\_t messages 
to the Tiva.

`sudo nav.build/atomcore/tanfan/tanfan`


## Vision Driver
Reads from cameras, runs the line and roombda detection pipelines, publishes 
messages for localization.

`sudo nav.build/src/vision/driver/maav-vision`


## ZCM Spy
Spy on the messages sent over ipc.

`sudo zcm-spy -u ipc -p nav.build/atomcore/src/zcmtypes/libatomcore-msg.so`


## ZCM Logger
Logs ZCM messages sent over ipc during the flight. In the command below, 
replace `<log_file>` with an appropriate file name.

`sudo zcm-logger -u ipc <log_file>`
