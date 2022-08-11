# ojousima.acceleration_broadcaster.c

This firmware runs 3 separate measurements in one-minute cycle.

* First, official Ruuvi RAWv2 measurement
* Second, unofficial RMS, P2P acceleration format
* Third, FFT data of acceleration

These measurements are broadcasted once per cycle with 5 duplications for total of 15 advertisements.