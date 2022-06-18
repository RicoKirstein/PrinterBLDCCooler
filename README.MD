BLDC driver using B-G431B-ESC1 eval board
and WM7040-24V CPAP blower
currently only PWM input works from 0-99% power level on the pwm pin

I strongly advise adding a large capacitor to the ESC board.
Right now only way to get rpm feedback is using the ST MotorPilot tool over the usb serial.
pinout for the hallsensor can be found here
https://www.st.com/resource/en/user_manual/um2516-electronic-speed-controller-discovery-kit-for-drones-with-stm32g431cb-stmicroelectronics.pdf

Parameters are set up with 24V 3A ~30k rpm maximum

Easiest to compile or change things seems to be the stm32CubeIDE for this generated project.
It should build with open source GCC packages but your mileage may vary.