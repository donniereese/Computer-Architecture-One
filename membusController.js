class BUS {
    constructor(busLocations) {
        this.banks = new Array(4);
        this.banks.fill(0);

        this.reg = new Array(8);
        this.reg.fill(0);

        // Read Write Bank Switching value.
        // READ  WRITE < 4 possible options for each
        // 4321  4321  < sets for read and write
        // 0001  0001  < read from bank 1, write to bank 1
        this.bankRW = 0b00010001;

        this.addressValue = 0b00000000;
        this.address = 0b00000000;

        this.data = null;

        this.flags = {
            WRITE: false,
            READ: false,
            FAULT: false
        };

        for (let i = 0; i < busLocations.length; i++) {
            this.banks[i] = busLocations[i];
        }
    }

    v(t, m) {
        // console.log(t, m);
    };

    setBANK(bank) {
        if (!this.flags.FAULT) this.bankRW = bank;
    }

    setREADBANK(bank) {
        const read = bank << 4;
        const write = this.bankRW & 0b00001111;
        this.bankRW = read | write;

    }

    setWRITEBANK(bank) {
        const write = bank;
        const read = this.bankRW & 0b11110000;
        this.bankRW = read | write;
    }

    privateSWAP() {
        if (!this.flags.FAULT) {
            // perform action bassed on flag.
            const flag = this.flags.WRITE ? 0b00000001 : this.flags.READ ? 0b00000010 : 0b00000000;
            switch (flag) {
                case 0b00000010:
                    // READ
                    break;
                case 0b00000001:
                    // WRITE
                    break;
                default:
                    // nothing
            }
        }
    }

    READ() {
        if (!this.flags.FAULT) {
            // set read flag
            this.flags.READ = true;
            // unset write flag
            // this.flags.WRITE = false;

            let readBank = 1;
            let bank = this.bankRW & 0b11110000;
            bank >>= 4;
            for (let i = 0; i < 4; i ++) {
                const shift = bank >> i;
                const bit = shift & 0b00000001;
                if (bit === 1) readBank += i;
            }
            this.data = this.banks[readBank - 1].read([this.address]);
            // Unset read flag at the end.
            this.flags.READ = false;
        }
    }

    WRITE() {
        if (!this.flags.FAULT) {
            // set write flag
            this.flags.WRITE = true;
            // unset read flag
            // this.flags.READ = false;

            let writeBank = 1;
            for (let i = 0; i < 4; i ++) {
                const shift = this.bankRW >> i;
                const bit = shift & 0b00000001;
                if (bit === 1) writeBank += i;
            }
            this.banks[writeBank - 1].write([this.address], this.addressValue);

            // Unset the Write flag.
            this.flags.WRITE = false;
        }
    }

    set ADDR(byte) {
        if (!this.flags.FAULT) this.address = byte;
        this.v('SET ADDR: ', byte);
    }

    get ADDR() {
        if (!this.flags.FAULT) return this.address;
        this.v('GET ADDR: ', this.address);
    }

    set ADDRVAL(byte) {
        if (!this.flags.FAULT) this.addressValue = byte;
        this.v('SET ADDRVAL: ', byte);
    }

    get ADDRVAL() {
        if (!this.flags.FAULT) return this.addressValue;
        this.v('GET ADDR: ', this.addressValue);
    }

    DATA() {
        if (!this.flags.FAULT) return this.data;
        this.v('READ DATA: ', this.data);
    }
}

module.exports = BUS;
