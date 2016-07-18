# carloop-obd-publisher

Firmware for Particle Electron, publishes OBD data

- https://www.carloop.io/
- https://www.particle.io/
- https://en.wikipedia.org/wiki/OBD-II_PIDs#Standard_PIDs

# decoding

Using the above Wikipedia page listing OBD-II PIDs, I decoded the following by hand
from data received on my laptop (sitting at home) using the Particle CLI subscribing
to the event stream while I was driving around the block.

Obvious next step: decode on laptop/server with a program instead of by hand.

```
645.07    034104    58        engine load 34.5%
645.25    034105    73        coolant temp 75˚C
645.43    034106    81        short term fuel trim 0.78%
645.67    034107    7b        long term fuel trim -3.9%
645.85    04410c    1ca6      engine rpm 1833.5
646.03    03410d    35        vehicle speed 53 km/h
646.21    03410e    b6        timing advance 27˚ before top dead centre (TDC)
646.39    03410f    3f        intake air temperature 23˚C
646.57    044110    037b      MAF air flow rate 8.91 g/s
646.75    034111    39        throttle 22.4%
646.93    044115    9bff      oxygen sensor 2: 0.775V, sensor not used in trim calculation
647.11    04411f    0105      run time 261s
647.35    044121    2a47      distance traveled with MIL on: 10823 km
647.53    03412e    ff        Commanded evaporative purge: 100%
647.71    03412f    b3        fuel tank level 70%
647.89    034130    a8        warm-ups since codes cleared: 168
648.07    044131    2a0f      distance traveled since codes cleared: 10767 km
648.25    034133    65        Absolute Barometric Pressure: 101 kPa
648.43    064134    7f707ffb  oxygen sensor 1: 99.56% fuel-air equivalence ratio, -0.01953125 mA
648.61    04413c    1809      Catalyst Temperature, Bank 1, Sensor 1: 575.3˚C
648.79    064140    fed00000  pids supported 41-60 = 1111 1110 1101 = 41, 42, 43, 44, 45, 46, 47, 49, 4a, 4c

649.02    034104    67        engine load 40.4%
649.20    034105    74        coolant temp 76˚C
649.38    034106    80        short term fuel trim 0%
649.56    034107    7b        long term fuel trim -3.9%
649.74    04410c    1c8c      engine rpm 1827
649.92    03410d    35        vehicle speed 53 km/h
650.10    03410e    b1        timing advance 24.5˚ before top dead centre (TDC)
650.28    03410f    3f        intake air temperature 23˚C
650.46    044110    03e5      MAF air flow rate 9.97 g/s
650.70    034111    3b        throttle 23.1%
650.88    044115    97ff      oxygen sensor 2: 0.755V, sensor not used in trim calculation
651.06    04411f    0109      run time 265s
651.24    044121    2a47      distance traveled with MIL on: 10823 km
651.42    03412e    ff        Commanded evaporative purge: 100%
651.60    03412f    b3        fuel tank level 70%
651.78    034130    a8        warm-ups since codes cleared: 168
651.96    044131    2a0f      distance traveled since codes cleared: 10767 km
652.14    034133    65        Absolute Barometric Pressure: 101 kPa
652.38    064134    7ed87ffa  oxygen sensor 1: 99.097% fuel-air equivalence ratio, -0.0234375 mA
652.56    04413c    1873      Catalyst Temperature, Bank 1, Sensor 1: 585.9˚C
652.74    064140    fed00000  pids supported 41-60 = 1111 1110 1101 = 41, 42, 43, 44, 45, 46, 47, 49, 4a, 4c
```
