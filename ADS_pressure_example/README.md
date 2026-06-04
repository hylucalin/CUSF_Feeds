# ADS_pressure_example

Minimal Arduino IDE example for reading two pressure-transducer voltage dividers using an ESP32-S3 and an ADS1115.

## Arduino library required

Install this from Arduino IDE Library Manager:

```text
Adafruit ADS1X15
```

## Pin connections

| ADS1115 pin | Connect to |
|---|---|
| VIN / VDD | ESP32-S3 3V3 |
| GND | ESP32-S3 GND / common GND |
| SDA | ESP32-S3 GPIO1 |
| SCL | ESP32-S3 GPIO2 |
| ADDR | GND |
| A3 | Voltage-divider midpoint for pressure sensor 1 |
| A1 | Voltage-divider midpoint for pressure sensor 2 |

With ADDR tied to GND, the ADS1115 I2C address is `0x48`.

## Measured voltage dividers used in this version

Circuit for each channel:

```text
Pressure sensor green ---- R_TOP ----+---- ADS1115 A3 or A1
                                      |
                                    R_BOTTOM
                                      |
                                     GND
```

This version uses these measured resistor values:

| Channel | R_TOP | R_BOTTOM | Divider ratio |
|---|---:|---:|---:|
| A3 | 4300 ohm | 8160 ohm | 0.654896 |
| A1 | 4310 ohm | 8300 ohm | 0.658208 |

The code computes:

```text
V_ads = V_sensor * R_BOTTOM / (R_TOP + R_BOTTOM)
V_sensor = V_ads / divider_ratio
```

## Important voltage-margin note

These measured dividers are fine if your pressure transducer is the common `0.5 V to 4.5 V output` type:

```text
A3: 4.5 V * 0.654896 = 2.947 V
A1: 4.5 V * 0.658208 = 2.962 V
```

That is safely below 3.3 V.

If the sensor can truly output 5.0 V, the ADS1115 would see approximately:

```text
A3: 5.0 V * 0.654896 = 3.274 V
A1: 5.0 V * 0.658208 = 3.291 V
```

That is below 3.3 V but has very little safety margin. For a true 0-5 V output sensor, a divider ratio nearer 0.60 is better.

## Pressure sensor wiring

Typical 3-wire 5 V pressure transducer:

| Sensor wire | Function | Connect to |
|---|---|---|
| Red | +5 V supply | 5 V regulator |
| Black | Ground | Common GND |
| Green | Analogue output | Voltage divider, then ADS1115 A3/A1 |

Do **not** connect the pressure sensor green wire directly to the ADS1115 when the ADS1115 is powered from 3.3 V. Use a divider.

## Serial monitor

Use baud rate `115200`.

Expected startup output:

```text
ESP32-S3 ADS1115 pressure example
---------------------------------
I2C SDA GPIO: 1
I2C SCL GPIO: 2
A3 divider ratio: 0.654896
A1 divider ratio: 0.658208
A3 voltage at ADS if sensor output is 5.0 V: 3.274 V
A1 voltage at ADS if sensor output is 5.0 V: 3.291 V
Initialising ADS1115...
ADS1115 found at 0x48.
```
