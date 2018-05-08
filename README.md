# MSP430_Smart_Lamp

A smart desk lamp implemented with MSP430 MCU. The lamp is turned on by pressing the button at Pin 1.3 and subsequent presses will cycle the light through three discrete brightness levels. If the button is not pressed after 3 seconds, the next press will switch the light off. When switching the light back on, the last used brightness level is returned. The user interface is provided via UART. Users can monitor the current brightness level and send strings ( i.e. 1, 2, 3, off ) to control the lamp. 

The programme is compiled and uploaded with Energia, a dedicated IDE for TI-branded embedded devices: www.ti.com/tool/ENERGIA.
