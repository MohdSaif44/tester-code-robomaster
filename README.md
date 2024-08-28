# mainboard-usb

added usb and updated the HAL library and the freeRTOS library

the new HAL library has some changes that might lead to unexpected behavior with our library. will try to find them and document them here:

### 1. SystemClock_Config

In our system clock configuration we would setup in the following way[^1]:
```C
RCC_OscInitStruct.PLL.PLLM = 8;
RCC_OscInitStruct.PLL.PLLN = 336;
RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
RCC_OscInitStruct.PLL.PLLQ = 4;
```
However in order for the usb to function properly you need 48Mhz to usb clock(labeled 48Mhz clocks in ioc) in order to achieve that we to change the PLLQ to 7:
```C
RCC_OscInitStruct.PLL.PLLM = 8;
RCC_OscInitStruct.PLL.PLLN = 336;
RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
RCC_OscInitStruct.PLL.PLLQ = 7;
```
As you can see from the following image changing the PLLQ only affect the 48MHz clocks and nothing else. So in theory it should not affect any other timers:

![Screenshot of the new clock configuration in the ioc.](https://github.com/amer-adam/mainboard-usb/assets/47683779/9bf505f8-9d10-4665-affe-e45b05033f77)


### 2. Timers

In timer init we would start the timer in both normal and interupt modes in the following way:
```C
  HAL_TIM_Base_Start(htimx);
  HAL_TIM_Base_Start_IT(htimx);
```
However in the new hal library this behavior is not supported and it will only start in one mode only and ignore the second line. So for now i will start them only in interrupt mode like this[^2]:
```C
  /*HAL_TIM_Base_Start(htimx);*/
  HAL_TIM_Base_Start_IT(htimx);
```

[^1]: When i tried to replicate our configuration in the ioc i got errors because the PLLQ set to 4 will not give the 48MHz clocks 48MHz. So i have no idea why we use this configuration. if you know please explain to me :pray:.
[^2]: I asked the supersenior who wrote the library and he said in his code he only starts in interupt mode. So i don't know who added the normal mode or why but for now i will assume it is not necessary.
