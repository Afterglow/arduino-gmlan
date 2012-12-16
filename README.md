Summary
=======

This code is for use in sending and recieving GMLAN messages from the single wire canbus of a GM vehicle using an arduino with canbus shield. Tested on a Vauxhall VXR8 (29 bit send) and a Vauxhall Vectra-C (11 bit send).

Licensing
=========

Files are licensed individually, some are included from other sources, other parts are original work.

Usage
=====

Code is designed to run on an arduino uno with a sparkfun canbus shield. It will log data to an SD card if present. There are also a couple of examples of sending messages by using the joystick to inject messages. You would need to make sure the messages were appropriate for your vehicle, mine are to adjust the volume on a Vauxhall VXR8/Pontiac G8/HSV Clubsport R8.
