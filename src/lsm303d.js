/**
 * This is a port to javascript of Pimoroni's python code
 *
 * https://github.com/pimoroni/enviro-phat/blob/159542218732e1cef79b561a5ff799d04cb8c8e7/library/envirophat/lsm303d.py
 *
 */

// LSM303 Address
const ADDR = 0x1D; // Assuming SA0 grounded

// LSM303 Register definitions
const TEMP_OUT_L      = 0x05;
const TEMP_OUT_H      = 0x06;
const STATUS_REG_M    = 0x07;
const OUT_X_L_M       = 0x08;
const OUT_X_H_M       = 0x09;
const OUT_Y_L_M       = 0x0A;
const OUT_Y_H_M       = 0x0B;
const OUT_Z_L_M       = 0x0C;
const OUT_Z_H_M       = 0x0D;
const WHO_AM_I        = 0x0F;
const INT_CTRL_M      = 0x12;
const INT_SRC_M       = 0x13;
const INT_THS_L_M     = 0x14;
const INT_THS_H_M     = 0x15;
const OFFSET_X_L_M    = 0x16;
const OFFSET_X_H_M    = 0x17;
const OFFSET_Y_L_M    = 0x18;
const OFFSET_Y_H_M    = 0x19;
const OFFSET_Z_L_M    = 0x1A;
const OFFSET_Z_H_M    = 0x1B;
const REFERENCE_X     = 0x1C;
const REFERENCE_Y     = 0x1D;
const REFERENCE_Z     = 0x1E;
const CTRL_REG0       = 0x1F;
const CTRL_REG1       = 0x20;
const CTRL_REG2       = 0x21;
const CTRL_REG3       = 0x22;
const CTRL_REG4       = 0x23;
const CTRL_REG5       = 0x24;
const CTRL_REG6       = 0x25;
const CTRL_REG7       = 0x26;
const STATUS_REG_A    = 0x27;
const OUT_X_L_A       = 0x28;
const OUT_X_H_A       = 0x29;
const OUT_Y_L_A       = 0x2A;
const OUT_Y_H_A       = 0x2B;
const OUT_Z_L_A       = 0x2C;
const OUT_Z_H_A       = 0x2D;
const FIFO_CTRL       = 0x2E;
const FIFO_SRC        = 0x2F;
const IG_CFG1         = 0x30;
const IG_SRC1         = 0x31;
const IG_THS1         = 0x32;
const IG_DUR1         = 0x33;
const IG_CFG2         = 0x34;
const IG_SRC2         = 0x35;
const IG_THS2         = 0x36;
const IG_DUR2         = 0x37;
const CLICK_CFG       = 0x38;
const CLICK_SRC       = 0x39;
const CLICK_THS       = 0x3A;
const TIME_LIMIT      = 0x3B;
const TIME_LATENCY    = 0x3C;
const TIME_WINDOW     = 0x3D;
const ACT_THS         = 0x3E;
const ACT_DUR         = 0x3F;

// Mag scales
const MAG_SCALE_2     = 0x00; // full-scale is +/- 2 Gauss
const MAG_SCALE_4     = 0x20; // +/- 4 Guass
const MAG_SCALE_8     = 0x40; // +/- 8 Guass
const MAG_SCALE_12    = 0x60; // +/- 12 Guass

const ACCEL_SCALE     = 2; // +/- 2g

const X = 0;
const Y = 1;
const Z = 2;

/**
 * Calculate the 2s complement of int:val
 */
function twos_comp(val, bits) {
    if (val & (1 << (bits - 1))) {
        val = (val & ((1 << bits) - 1)) - (1 << bits);
    }
    return val;
}

class Vector {
    constructor(x, y, z) {
        if (x instanceof Array && x.length === 3) {
            y = x[1];
            z = x[2];
            x = x[0];
        }
        this.x = x;
        this.y = y;
        this.z = z;
    }

    getItem(index) {
        return [this.x, this.y, this.z][index];
    }

    str() {
        return [this.x, this.y, this.z].toString()
    }
}

class lsm303d {
    constructor(i2c_bus, addr) {
        this._mag = [0,0,0];
        this._accel = [0,0,0];
        this._tiltcomp = [0,0,0];
        this._heading=0;
        this._heading_degrees=0;
        this._tilt_heading=0;
        this._tilt_heading_degrees=0;
        if (addr === undefined) {
            addr = ADDR;
        }
        this.i2c_bus = i2c_bus;

        if (!i2c_bus.write_byte_data || !i2c_bus.read_byte_data) {
            throw TypeError("Object given for i2c_bus must implement write_byte_data and read_byte_data methods");
        }
        this.addr = addr;
        this._is_setup = false;
    }

    setup() {
        if (this._is_setup) {
            return;
        }

        this._is_setup = true;

        let whoami = this.i2c_bus.read_byte_data(this.addr, WHO_AM_I);

        if (whoami === 0x49) {
            this.i2c_bus.write_byte_data(this.addr, CTRL_REG1, 0x57); // 0x57 = ODR=50hz, all accel axes on ## maybe 0x27 is Low Res?
            this.i2c_bus.write_byte_data(this.addr, CTRL_REG2, (3<<6)|(0<<3)); // set full scale +/- 2g
            this.i2c_bus.write_byte_data(this.addr, CTRL_REG3, 0x00); // no interrupt
            this.i2c_bus.write_byte_data(this.addr, CTRL_REG4, 0x00); // no interrupt
            this.i2c_bus.write_byte_data(this.addr, CTRL_REG5, 0x80|(4<<2)); // 0x10 = mag 50Hz output rate. 0x80 = enable temperature sensor
            this.i2c_bus.write_byte_data(this.addr, CTRL_REG6, MAG_SCALE_2); // Magnetic Scale +/1 1.3 Guass
            this.i2c_bus.write_byte_data(this.addr, CTRL_REG7, 0x00); // 0x00 continuous conversion mode

        } else {
            throw Error("No lsm303d detected");
        }
    }

    /**
     * Read the temperature sensor and return the raw value in units of 1/8th degrees C.
     *
     * This is an uncalibrated relative temperature.
     */
    temperature() {
        this.setup();

        // in order to read multiple bytes the high bit of the sub address must be asserted
        return twos_comp(this.i2c_bus.read_word_data(this.addr, TEMP_OUT_L|0x80), 12);
    }

    /**
     * Read the magnetomter and return the raw x, y and z magnetic readings as a vector.
     *
     * The returned vector will have properties x, y and z.
     */
    magnetometer() {
        this.setup();

        //
        // taken from https://github.com/pimoroni/enviro-phat/pull/31/commits/78b1058e230c88cc5969afa210bb5b97f7aa3f82
        // as there is no real counterpart to struct.unpack in js
        //
        // in order to read multiple bytes the high bit of the sub address must be asserted
        this._mag[X] = twos_comp(this.i2c_bus.read_word_data(this.addr, OUT_X_L_M|0x80), 16);
        this._mag[Y] = twos_comp(this.i2c_bus.read_word_data(this.addr, OUT_Y_L_M|0x80), 16);
        this._mag[Z] = twos_comp(this.i2c_bus.read_word_data(this.addr, OUT_Z_L_M|0x80), 16);

        return new Vector(this._mag)
    }

    /**
     * Read the accelerometer and return the x, y and z acceleration as a vector in Gs.
     *
     * The returned vector will have properties x, y and z.
     */
    accelerometer() {
        this.setup();

        let accel = [0, 0, 0];

        //
        // taken from https://github.com/pimoroni/enviro-phat/pull/31/commits/78b1058e230c88cc5969afa210bb5b97f7aa3f82
        // as there is no real counterpart to struct.unpack in js
        //
        // in order to read multiple bytes the high bit of the sub address must be asserted
        accel[X] = twos_comp(this.i2c_bus.read_word_data(this.addr, OUT_X_L_A|0x80), 16);
        accel[Y] = twos_comp(this.i2c_bus.read_word_data(this.addr, OUT_Y_L_A|0x80), 16);
        accel[Z] = twos_comp(this.i2c_bus.read_word_data(this.addr, OUT_Z_L_A|0x80), 16);

        for (let i = X; i < Z + 1; i += 1) {
            this._accel[i] = accel[i] / Math.pow(2, 15) * ACCEL_SCALE;
        }

        return new Vector(this._accel);
    }

    /**
     * Return a raw compas heading calculated from the magnetometer data.
     */
    raw_heading() {
        this._heading = Math.atan2(this._mag[X], this._mag[Y]);

        if (this._heading < 0) {
            this._heading += 2 * Math.PI;
        }

        if (this._heading > 2 * Math.PI) {
            this._heading -= 2 * Math.PI;
        }

        this._heading_degrees = Math.round(100 * 180 * this._heading / Math.PI) / 100;

        return this._heading_degrees;
    }

    /**
     * Return a tilt compensated heading calculated from the magnetometer data.
     *
     * Returns None in the case of a calculation error.
     */
    heading() {
        this.update();
        let truncate = [0, 0, 0];
        for (let i = X; i < Z + 1; i += 1) {
            let theSign = this._accel[i] >= 0 ? 1 : -1;
            truncate[i] = theSign * Math.abs(Math.min(Math.abs(this._accel[i]), 1.0));
        }

        try {
            let pitch = Math.asin(-1*truncate[X]);
            let roll = 0; //set roll to zero if pitch approaches -1 or 1
            if (Math.abs(Math.cos(pitch)) >= Math.abs(truncate[Y])) {
                roll = Math.asin(truncate[Y]/Math.cos(pitch));
            }


            this._tiltcomp[X] = this._mag[X] * Math.cos(pitch) + this._mag[Z] * Math.sin(pitch);
            this._tiltcomp[Y] = this._mag[X] * Math.sin(roll) * Math.sin(pitch)
                + this._mag[Y] * Math.cos(roll) - this._mag[Z] * Math.sin(roll) * Math.cos(pitch);
            this._tiltcomp[Z] = this._mag[X] * Math.cos(roll) * Math.sin(pitch) +
                               this._mag[Y] * Math.sin(roll) +
                               this._mag[Z] * Math.cos(roll) * Math.cos(pitch);
            this._tilt_heading = Math.atan2(this._tiltcomp[Y], this._tiltcomp[X]);

            if (this._tilt_heading < 0) {
                this._tilt_heading += 2 * Math.PI;
            }

            if (this._tilt_heading > 2*Math.PI) {
                this._heading -= 2*Math.PI;
            }

            this._tilt_heading_degrees = Math.round(100 * 180 * this._tilt_heading / Math.PI) / 100;
            return this._tilt_heading_degrees;
        } catch (ex) {
            return null;
        }
    }

    is_mag_ready() {
        return (this.i2c_bus.read_byte_data(this.addr, STATUS_REG_M) & 0x03) > 0;
    }

    /**
     * Update both the accelerometer and magnetometer data.
     */
    update() {
        this.accelerometer();
        this.magnetometer();
    }
}

module.exports = lsm303d;
