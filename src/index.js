/*
leds = leds()
light = tcs3472(bus)
weather = bmp280(bus)
analog = ads1015(bus)
motion = lsm303d(bus)
 */

const BusAdapter = require('./i2c-bus-adapter');
const lsm303d = require('./lsm303d');
const I2C = require('raspi-i2c').I2C;

class Enviro {
    constructor(theOptions) {
        const options = theOptions || {};
        this.i2c = options.i2c || new I2C();
        this.busPythonInterface = new BusAdapter(this.i2c);
        this.motion = new lsm303d(this.busPythonInterface);
        this.leds = null;       // TODO
        this.light = null;      // TODO
        this.weather = null;    // TODO
        this.analog = null;     // TODO
    }
}

module.exports = Enviro;
