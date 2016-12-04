WebTurtle

An ESP8266 version of the Arduino Drawing Robot (http://www.instructables.com/id/Arduino-Drawing-Robot/)
allowing control via WiFi and storage of logo procedures in its own (SPIFFS) file system.

Hardware used:
- See the original instructable. Instead of an Arduino UNO, a NodeMCU is used
  (https://www.amazon.com/HiLetgo-Version-NodeMCU-Internet-Development/dp/B010O1G1ES)
- 4 AA batteries with holder
- You'll also need a WiFi enabled computer capable of displaying web pages to control the robot

Software used:
- Arduino development environment for ESP8266

Work Flow

1. Install the development environment (see https://github.com/esp8266/Arduino). I used
   version 1.6.11 in this project.

   a. set the development board to NodeMCU 1.0 (Tools -> Board -> NodeMCU 1.0 (ESP-12E Module)
   
   b. Upload the contents of the data directory to the module
     (see https://github.com/esp8266/Arduino/blob/master/doc/filesystem.md#uploading-files-to-file-system)

2. Load webturtle4.ino into the development environment and change the file accordingly.
   The robot acts as its own wireless access point with the following parameters:
   
   a. ssid (currently set to "Turtle")
   
   b. password (currently set to "LogoTurtle")

   c. host (currently set to "turtle")
   
3. Program the NodeMCU module.
 
   a. Connect the NodeMCU to the development computer via the micro usb connector
   
   b. Hold down the flash button on the NodeMCU while pushing its rest button to put it into
      programming mode
      
   c. Upload the program to the module.
   
4. Build the chassis according to the instructable (Steps 1, 3-13)
 
   a. In step 7, this version uses the breadboard to mount the NodeMCU according to
      board_layout.jpg, so all other parts are shifted to accommodate it.
   
   b. In steps 8-12, the wiring is changed according to schematic.pdf
    
5. Turn on the robot. Then, with your computer, look for a WiFi access point with the ssid
   you assigned to your robot (e.g. Turtle). Connect to that ssid with the password you
   assigned (e.g. LogoTurtle).

6. In your computer's web browser, go to page http://turtle/ (or the host name you assigned the robot).
   If your browser fails to recognize the host name, go to http://192.168.4.1/ where you
   should see a text box and a Submit button.

7. In steps 14 and 15 of the instructable, you would have flashed firmware that drew a series of
   squares to aid you in calibration. You can draw the same four calibration boxes at this
   point by entering the following logo commands and then clicking the Submit button:
   
   pd rpt 16 [fd 100 lt 90] pu
   
   The commands do the following:
   * pd - Move the pen down
   * rpt 16 - Repeat the commands enclosed in brackets 16 times
   * fd 100 - Move forward 100 mm
   * lt 90 - Turn lt 90 degrees
   * pu - Move the pen up
   
8. Step 16 of the instructable calibrates the raising and lowering of the pen. As you saw
   previously, you can enter pu for pen up and pd for pen down to control the servo.
   
The robot implements a very restricted set of logo commands. They are:
* fd <value> - Move forward <value> mm, where <value> can be integer or decimal
* bk <value> - Move backward <value> mm, where <value> can be integer or decimal
* rt <value> - Turn right <value> degrees, where <value> can be integer or decimal
* lt <value> - Turn left <value> degrees, where <value> can be integer or decimal
* pu - Move the pen up
* pd - Move the pen down
* rpt <value> [cmd1 cmd2..] - Repeat the bracketed commands <value> times, where <value> is an integer
  (e.g. rpt 16 [fd 100 rt 90])
* to <function_name> <:variable_name1 :variable_name2..> cmd1 cmd2.. end - Define a function
  (e.g to square :side rpt 4 [fd :side rt 90] end)
  (The function is called like this: square 100)
* file <file_name> - Read input from file <file_name> in the robot's file system
    
Using the robot's file system

There is a 500 character limit on the commands that can be entered into the text box. That
may seem like a lot, but it's not. The ability to upload, list, and delete files is provided
by going to http://host/edit (or http://192.168.4.1/edit). Once you've uploaded a file, you
can have the robot read from it by including the file command. For example, typing

pd fd 50.5 file myinput pu

into the text box and clicking Submit will move the pen down, move forward 50.5 mm, and then
read all the commands in the file. Lastly, the pen is moved up

Webturtle uses the file system even when you don't upload files. For example, if you had typed

to circle rpt 24 [fd 20 rt 15] end pd circle pu

into the text box and clicked Submit, you would see a file called circle in directory p.
Every function you define will be stored in the p directory (in a tokenized form) until
you delete it.