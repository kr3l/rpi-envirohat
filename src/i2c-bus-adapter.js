class I2CAdapter {
    constructor(i2c) {
        this.i2c = i2c;
    }

    write_byte_data(address, register, byte) {
        return this.i2c.writeByteSync(address, register, byte);
    }

    read_byte_data(address, register) {
        return this.i2c.readByteSync(address, register);
    }

    read_word_data(address, register) {
        return this.i2c.readWordSync(address, register);
    }

    read_i2c_block_data(address, register, length) {
        return this.i2c.readSync(address, register, length);
    }
}

module.exports = I2CAdapter;
