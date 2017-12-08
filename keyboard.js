const stdin = process.stdin;
stdin.setRawMode(true);
const util = require('util');

class KEYBOARD {
    constructor() {
        // this.buffer = new Array(127);
        // this.reg = new Array(16);
        // this.reg.fill(0);
        // this.curReg = 0b00000000;
        // this.reg.PC = 0b00000000;
        // this.reg.SP = 0b11111111;
        stdin.resume();
        // i don't want binary, do you?
        stdin.setEncoding( 'utf8' );
        // on any data into stdin
        stdin.on( 'data', this.input.bind(this));
        // interupt set
        this.setInterrupt = null;
        // memory block allocation
        this.memAccess = null;
    }

    input(key) {
        // ctrl-c ( end of text )
        // if ( key === '\u0003' ) {
        //     process.exit();e
        // }
        // write the key to stdout all normal like
        // process.stdout.write( key );
        const keyBinary = key.charCodeAt(0).toString(2).padStart(8, '0');
        // this.curReg = 0b00000000;
        // this.reg[this.curReg] = keyBinary;
        // this.PUSH();
        // TODO: Shortcircuit real function and just output straight to buffer.
        // this.buffer.push(keyBinary);
        this.memAccess(keyBinary);
        this.setInterrupt();
    }


    hook(f, e = null) {
        this.setInterrupt = f;
        this.memAccess = e;
    }



    READIN() {
        // return this.buffer.shift();
    }

    POP() {

    }

    PUSH() {

    }

    done() {
        // process.exit();
    }
}

module.exports = KEYBOARD;
