High Level Interface (HLI)
==========================

The HLI is another computer on AAUSHIP that is providing the intelligent
interfacer for the ship, in other words it is the main brain, while the LLI
is just an universal interface that is ship hardware dependent.

It is written in python for the ease of use.

Source files and other files
----------------------------
`42-aauship.rules` is udev rules that ensures that consitent device
names is assigned for the USB devides such as the second GPS,
echosounder, LLI, and radio interface. It is to be copied into
`/etc/udev/rules.d`. (The radio interface is used on the GRS to
communicate with the LLI directly, not on the ship.)
