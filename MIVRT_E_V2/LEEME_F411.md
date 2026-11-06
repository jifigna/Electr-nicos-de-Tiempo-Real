# MIVRT_E_V2 — versión para la placa STM32F411E-DISCO

Copia de MIVRT_E adaptada a la **32F411EDISCOVERY** (chip STM32F411VET6).
El proyecto que se entrega sigue siendo **MIVRT_E** (F407 + LIS3DSH).

## Qué cambia respecto a MIVRT_E

| | MIVRT_E (F407) | MIVRT_E_V2 (F411) |
|---|---|---|
| Reloj CPU | 168 MHz | 84 MHz (máx. F411: 100 MHz) |
| APB1 / APB2 | 42 / 84 MHz | 42 / 84 MHz |
| Flash wait states | 5 | 2 |
| Base de tiempo HAL | TIM6 | TIM11 (la F411 no tiene TIM6) |
| Acelerómetro | LIS3DSH por SPI1 (CS = PE3) | LSM303DLHC/AGR por I2C1 (PB6/PB9, dir. 0x19) |
| Conversión a mg | 0,06 mg/LSB | 0,0625 mg/LSB (1/16, modo HR ±2 g) |
| DWT → µs | /168 | /CICLOS_US (= SystemCoreClock/1e6 = 84) |
| Flash en el .ld | 1024K | 512K |

La lógica de tareas, periodos, prioridades, máquina de estados, vibración,
números de secuencia y medidas de C es **la misma** que en MIVRT_E.

## Variables para Live Expressions

- `init_err` = 0, `acc_ack` = 1 → el acelerómetro responde.
- `ctrl1` = 0x57 (87), `ctrl4` = 0x88 (136) → configuración aplicada.
- `who` = 0x33 (51) en placas con LSM303AGR; en las antiguas (LSM303DLHC) puede dar otro valor, y no importa.
- `i2c_errores` debería quedarse en 0.
- `SystemCoreClock` = 84000000.

## Ojo

- **No abrir el .ioc ni regenerar código**: volvería a la configuración del F407.
- Los C medidos aquí (a 84 MHz y con I2C) **no** son los de la F407: no usarlos en la memoria.
- Los ejes del LSM303 pueden tener otra orientación o signo que los del LIS3DSH.
