# STM32_encoder_simulator_project
#STM32F0DISCOVERY #STM32 UART DMA Rx and Tx example #encoder simulator example

##  Example 1 - STM32 UART DMA || Receive, Compare and Transmit unknown length DATA

### <u>Features</u> :
1.) Support using serial monitor of Arduino IDE or CP2102 UART to USB module to receive and Transmit data;

2.) STM32 will keep sending the messge(counter value) to Arduino;

3.) Using "stop", "resume","forward" and "backward" to control the STM32's counter from the Arduino serial monitor

4.) STM32 will filter out invalid data and blank data sent by Arduino

5.) STM32 blue and green LEDs will blink in a specific pattern

### <u>Hardware and Wirings</u>:
1.) STM32F0DISCOVERY

2.) Support Arduino or CP2102 UART to USB module to Rx and Tx data

| Wiring ||
|:---:|:---:|
| STM32 | Arduino Uno |
| PA2 Pin |  Pin 1 |
| PA3 Pin | Pin 0 |



### <u>Initial setting in STM32CubdeMX</u>:
1. Setup connectivity USART2

    a. Mode >> Asynchronous;
    
    b. Configuration >> Parameter Settings >> Basic Parameters >> set Baud Rate e.g. 115200 (8N1);
    
    c. in DMA Setting, add "USART2_RX" and "USART2_TX";

    b. DMA Request Settings keep default settings >> Mode : Normal, tick Memory, Data Width = Byte;
    
    d. in NVIC settings, tick USART2 global interrupt

2. Setup RCC

    a. Configuration >> NVIC Settings >> Enable RCC global interrupt



##  Example 2 - STM32 Encoder simulator
### <u>Features</u> :

### <u>Hardware and Wirings</u>:

### <u>Initial setting</u>: