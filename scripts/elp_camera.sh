#!/bin/bash
#  - brightness (int) : min=0 max=255 step=1 default=128 value=128
#  - contrast (int) : min=0 max=255 step=1 default=128 value=128
#  - saturation (int) : min=0 max=255 step=1 default=128 value=128
#  - white_balance_temperature_auto (bool) : default=1 value=1 
#  - gain (int) : min=0 max=255 step=1 default=0 value=0 
#  - power_line_frequency (menu) : min=0 max=2 default=2 value=2
#  - white_balance_temperature (int) : min=2000 max=6500 step=1 default=4000 value=4000  
#  - sharpness (int) : min=0 max=255 step=1 default=128 value=128 
#  - backlight_compensation (int) : min=0 max=1    step=1 default=0 value=0 
#  - exposure_auto (menu) : min=0 max=3 default=3    value=3  (0: Auto Mode 1: Manual Mode
#            2: Shutter Priority Mode
#            3: Aperture Priority Mode) 
#  - exposure_absolute (int) : min=3 max=2047 step=1 default=250 value=250  
#  - exposure_auto_priority (bool) :    default=0 value=1 
#  - focus_absolute (int) : min=0 max=250 step=5    default=0 value=0 
#  - focus_auto (bool) : default=1 value=1 
#  - zoom_absolute    (int) : min=100 max=500 step=1 default=100 value=100

v4l2-ctl \
	--set-ctrl=exposure_auto=3\
	--device=/dev/video6
	# --set-ctrl=exposure_absolute=100 \
	# --set-ctrl=gain=20 \
