## OSSM-Stroke

New Codebase for OSSM - Open Source Sex Machine 
Integration of theelims StrokeEngine https://github.com/theelims/StrokeEngine

When your came here Looking for the M5 Version its in a Sub Tree right now: 

https://github.com/ortlof/OSSM-Stroke/tree/OSSM_ESP_Remote

Flash with Platform IO. 

First Version Local Control with Remote & M5 Remote. 
For Code and Hardware for the M5 Remote itself look here: https://github.com/ortlof/OSSM-M5-Remote

You need to set you mac adresses in both codes right now Manual:

After building both codes they display MAC's in Serial Console.
Find Line and Change to your OSSM MAC in OSSM code you have to Set the mac of your Remote. 

MAC in Serial Looks like: 34:86:5D:57:F5:84

Change to this:

"uint8_t Remote_Address[] = {0x34, 0x86, 0x5d, 0x57, 0xf5, 0x84};"

# Display Menü 

Left Knob Speed Setting

Start -> Shortpress -> Home 

Home:
Right Knob Sensation From -100 to 100
Shortpress Setup Menue  | Longpress start & stop of Motion

Menue:
Right Knob Select Settings
On Press Set settings with Right Knob Press again back in Menue

Home            Goes Back to Home with Speed
Set Depth       Sets Depth after this Motion
Set Stroke      Sets Stroke after this Motion
Set Pattern     Sets New Pattern after Leavin with Knob Press
Inter. Depth    Setup Optimal Depth Interactively 
Depth Fancy     Setup Optimal Depth & Stroke Interactively Left knob for Stroke and right knob for Depth