Frequency:  <br />
	&nbsp;&nbsp;&nbsp;1. `Base,A4`: 4000 Hz <br />
 	&nbsp;&nbsp;&nbsp;2. `A2, A3`:  2000 Hz <br />
  	&nbsp;&nbsp;&nbsp;3. `A1`: 1000 Hz <br />
The data you have to send is: 
   - For coordinates: <br />
            -   `B=XXXXXX,AX=XXXXXX,AY=XXXXXX,AZ=XXXXXX,AT=XXXXXX` <br />
            - To change direction of each axis, you can put a character '-' after character '=' like this: <br />
	    &nbsp; `B=-XXXXX,AX=-XXXXX,AY=-XXXXX,AZ=-XXXXX,AT=-XXXXX`
# Pin configuration
|Label  |Pin | Description |
| :--- | :--- | :---|
| `DIR(B,A1,A2,A3,A4)` | PA3, PA4, PA5, PA6, PA2|Connect with **Direction (DIR+/-)** in the driver|
| `TIM2_CH2` | PA1 |Connect to Base axis in **PUL** pins in the driver|
| `TIM3_CH2` | PB5 |Connect to Axis 1 in **PUL** pins in the driver|
| `TIM4_CH4` | PB9 |Connect to Axis 2 in **PUL** pins in the driver|
| `TIM3_CH4` | PB1 |Connect to Axis 3 in **PUL** pins in the driver|
| `TIM2_CH1` | PB0 |Connect to Axis 4 in **PUL** pins in the driver|
| `USART1_TX` | PB6 |Connect to **TX** of module UART|
| `USART1_RX` | PB7 |Connect to **RX** of module UART|
	
