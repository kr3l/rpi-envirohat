# Enviro pHAT nodejs library #

This package aims to create a similar interface to the [official Python library](https://github.com/pimoroni/enviro-phat).

## Usage

```js
const I2C = require('raspi-i2c').I2C;
const Enviro = require('rpi-envirohat');

const enviro = new Enviro({
    i2c: new I2C()
});

console.log({
	accelero: enviro.motion.accelerometer(),
	heading: enviro.motion.heading(),
	rawHeading: enviro.motion.raw_heading(),
	magneto: enviro.motion.magnetometer(),
	temperature: enviro.motion.temperature() //units of 1/8th degrees C.
});

```
