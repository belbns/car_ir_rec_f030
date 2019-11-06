# The steering wheel buttons imitator
<br>
based on STM32F030F4P6 demo board with using digitally-controlled
potentiometer X9C103P for imitating resistive matrix
of real steering wheel buttons.
<br>
The device allow imitate up to 16 buttons with 400 Ohm (4 pulses) steps.
<br>
Any IR remote control with NEC protocol can be used as a steering wheel buttons.
I'm using the "Steering Wheel Remote Control" (11 buttons) from Aliexpress (rem_ctrl.png).
<br>
The Device must be connected to KEY1 or KEY2 input of car multimedia device,
Android based multimedia player in my case.
<br>
The Lerning mode (the Learn button) allow to detect and to remember remote codes 
and to save them to the internal flash memory.
<br>
The Test button is using to manual connect the potentiometer to multimedia player
during setting buttons procedure.
<br>

# Имитатор резистивной матрицы кнопок руля
<br>
на базе демо-платы STM32F030F4P6 и цифрового потенциометра X9C103P.
<br>
Устройство позволяет имитировать нажатие до 16 кнопок руля с шагом 400 Ом (4 импульса).
Я использую блок кнопок руля с инфракрасным излучателем (11 кнопок) с Алиэкспресс (rem_ctrl.png).
<br>
Устройство подключается ко входу KEY1 или ко входу KEY2 радио/плеера на базе Андроид.
<br>
Режим обучения (кнопка Learn) позволяет распознать и запомнить во внутреннюю память
 коды IR излучателя.
<br>
Кнопка Test используется при обучении радио/плеера.
<br>
<br>
* Tools: Linux Mint, gcc-arm-none-eabi, OpenOCD, LIBOPENCM3, Sublime Text.
<br><br>
NEC IR protocol detecting idea:
<br>
https://blog.csdn.net/u011303443/article/details/76945003
<br>
https://programmer.help/blogs/stm32-timer-for-infrared-remote-control-data-reception.html
<br>
Waveform generation idea: General-purpose timer cookbook (AN4776, STMicroelectronics).


