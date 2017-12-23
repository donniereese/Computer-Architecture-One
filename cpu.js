const INIT  = 0b00000001;   // Initialize
const SETR  = 0b00000010;   // Set Register
const GETR  = 0b01101111;   // Get Register Value
const SAVE  = 0b00000100;   //
const LOAD  = 0b00000111;   //
const MUL   = 0b00000101;   //
const PRN   = 0b00000110;   //
const HALT  = 0b00000000;   //

// Output Extension
const PRA   = 0b01000001;   //

// Universal Ext. Commands  //
const EXTDO = 0b11000000;   // Issue {extension#} a {extension Command}
const EXTRT = 0b11000001;   // Get {extension#} to return

// Load and store extensions
const LD    = 0b00001000;   //
const ST    = 0b00001001;   //
const LDRI  = 0b00010010;   //
const STRI  = 0b00010011;   //
const STOR  = 0b11110101;   //
const LODM  = 0b11110111;   //

// Math Extension
const ADD   = 0b00001100;   // add two registers
const SUB   = 0b00001101;   // subtract two registers
const DIV   = 0b00001110;   // Divide two registers

// push / pop
const PUSH  = 0b00001010;   // Push onto stack
const POP   = 0b00001011;   // pop off stack

// call and return extensions
const CALL  = 0b00001111;   // Call subroutine
const RET   = 0b00010000;   // Return from Call

// Logic Extension
const JMP   = 0b00010001;   // Jump to memory
const JTL   = 0b00011110;   // Jump to previous label
const JEQ   = 0b00010011;   // Jump if equal
const JNE   = 0b00010100;   // Jump if not equal
const CMP   = 0b00010110;   // Compare

// Logical Structuring extensions
const LBL   = 0b01100000;   // Set a Label

// Memory Control
const ADR   = 0b11011000;   // (ADR)ess a memory block for read and write.
const RAD   = 0b11011001;   // Set (R)ead (AD)dress block
const WAD   = 0b11011010;   // Set (W)rite (AD)dress block
const RADR  = 0b11011011;   // (R)ead (ADR)ess for block

// Memory Management
const CPYR  = 0b11000011;   // Copy Recursive from range to another range

// Interupts
const SETI  = 0b00100000;   // Set Interrupt Address
const GETI  = 0b00100001;   // Get Interrupt Address
const RETI  = 0b11101110;   // (RET)urn from {I}nterupt
// Display Extension


const SP = 240; // stack poointer in register 1015, 8 - max 1023
// 240  241  242  243  244  245  246  247  248  249  250  251  252  253  254
// stak                     IM   IS   ad8  ad7  ad6  ad5  ad4  ad3  ad2  ad1


class CPU {
    constructor(ext, bus) {
        // // Set Rom
        // this.rom = new Array(511);
        // this.rom.fill(0);
        // Set Memory
        // this.mem = new Array(254);
        // this.mem.fill(0);
        // Set Registry
        this.reg = new Array(254);
        this.reg.fill(0);
        // Set SRM - (S)witch between (R)om and (M)emory program
        // first 4 are READ and second 4 are WRITE
        // 0 = 0b00010001 = Rom
        // 1 = 0b00100010 = Memory
        // 2 = 0b01000100 = Cartridge
        this.reg.SRM = 0b00000000;
        // Set Rom Program Counter
        this.reg.RC = 0;
        // Set Mem Program Counter
        this.reg.PC = 0;
        // stack pointer
        this.reg.SP = 0;
        // Set current Registry token
        this.curReg = 0;
        // Interupt Flag
        this.reg.IS = 246;
        this.reg[this.reg.IS] = 0b11111111;
        // Interrupt Mask Register (IM)
        this.reg.IM = 245;
        this.reg[this.reg.IM] = 0b00000000;
        // Flags
        this.flags = {
            WAIT: false,
            INTR: false,
            EQLS: false,
            BOOL: false,

        };
        // Set carryover flag
        this.CO = 0;

        // Set SRM to rom
        this.reg.SRM = 0b00010001;
        // memory data register
        this.reg.MDR = 0b00000000;
        // memory address register
        this.reg.MAR = 0b00000000;

        // Bus
        this.membus = bus;
        // set bus read/write banks
        this.membus.setBANK(this.reg.SRM);

        this.EXT = Array(8);
        this.EXT.fill(null);
        this.EXT.reg = Array(24);
        this.EXT.reg.fill(0);

        // Extensions..
        // I don't know if this is like a bus but ugh....
        // load extension hooks into EXT array
        for (let i = 1; i <= ext.length; i++) {
            // this.mem[this.reg.EP] = this.alu('ADD', this.reg.EP, 0b00001000);
            this.reg[this.reg.IM] |= i;
            // get output function from giving input functions in hook
            ext[i - 1](
                () => {
                    // set interupt
                    this.reg[this.reg.IS] |= i;
                    this.reg[this.reg.IS] = 0b00000001;
                    this.flags.INTR = true;
                },
                (byte) => {

                    // set ext mem block location.
                    // this.mem[241] = byte;
                    // set input register
                    this.EXT.reg[i] = byte;
                }
            );
        }
        // Build branch table
        this.buildBranchTable();
    }

    poll() {

    }

    buildBranchTable() {
        let bt = {
            [INIT]: this.INIT,
            [SETR]: this.SETR,
            [GETR]: this.GETR,
            [SAVE]: this.SAVE,
            [MUL]: this.MUL,
            [PRN]: this.PRN,
            [HALT]: this.HALT
        };

        // Look for Output extensions
        if (PRA) bt[PRA] = this.PRA;
        // Look for Math extension
        if (ADD) bt[ADD] = this.ADD;
        if (SUB) bt[SUB] = this.SUB;
        if (DIV) bt[DIV] = this.DIV;
        // Look for Logic extension
        if (JMP) bt[JMP] = this.JMP;
        if (JEQ) bt[JEQ] = this.JEQ;
        if (JNE) bt[JNE] = this.JNE;
        if (CMP) bt[CMP] = this.CMP;
        // Look for Load and store extensions
        if (LD) bt[LD] = this.LD;
        if (ST) bt[ST] = this.ST;
        if (LDRI) bt[LDRI] = this.LDRI;
        if (STRI) bt[STRI] = this.STRI;
        // Look for push&pop extension
        if (PUSH) bt[PUSH] = this.PUSH;
        if (POP) bt[POP] = this.POP;
        // Look for Call & Return extension
        if (CALL) bt[CALL] = this.CALL;
        if (RET) bt[RET] = this.RET;
        // Mem store save extension
        if (STOR) bt[STOR] = this.MEMSTORE;
        if (LODM) bt[LODM] = this.MEMLOAD;
        // Interrupt control extension
        if (SETI) bt[SETI] = this.SETI;
        if (GETI) bt[GETI] = this.GETI;
        // if (RETI) bt[RETI] = this.RETI;

        this.branchTable = bt;

        const loadscreen = [
            `00001000 00001000 00001000 00001000 00001000 00001000 00001000`,
            `00001000 00001000 00001000 00001000 00001000 00001000 00001000`,
            `00001000 000/    /0000100/           \\001000 00001000 00001000`,
            `00001000 00/    / 000010/    ________/001000 00001000 00001000`,
            `00001000 0/    /0 00001/    /001000 00001000 00001000 00001000`,
            `00001000 /    /00 0000/     \\001000 00001000 00001000 00001000`,
            `00001000/    /000 0000\\_______    \\ 00001000 00001000 00001000`,
            `0000100/    /1000 00001000 0/     / 00001000 00001000 00001000`,
            `000010/    /01000 00001000 /     /0 00001000 00001000 00001000`,
            `00001/           / /            /00 00001000 00001000 00001000`,
            `0000|___________/ 0\\___________/000 00001000 00001000 00001000`,
            `00001000 00001000 00001000 00001000 00001000 00001000 00001000`,
            `00001000 00001000 00001000 00001000 00001000 00001000 00001000`
        ];

        loadscreen.forEach(line => console.log(line));
    }

    /**
     * Poke values into memory
     */
    poke (address, value) {
        this.membus.ADDR = address;                                 // set memory read/write address
        this.membus.ADDRVAL = value;                                // set memory value
        this.membus.WRITE();                                        // write to memory address
        // this.rom[address] = value;
    }

    /**
     * Peek value from memory
     */
    peek (address) {
        this.membus.ADDR = address;                                 // set memory read/write address
        this.membus.READ();                                         // read from memory address
        return this.membus.DATA();                                  // read memory data;
        // return this.rom[address];
    }

    /**
     * startClock
     */
    startClock() {
        this.clock = setInterval(() => { this.tick(); }, 200);
    }

    /**
     * stop the clock
     *
     */
    stopClock() {
        clearInterval(this.clock);
    }

    /**
     * tick
     *
     */
    tick() {

        // Is there a multi instruction waiting?
        // if (this.CO != 0) {
        //
        // }
        if (this.flags.INTR === true) {
            // Mask the binary
            const masked = this.reg[this.reg.IS] & this.reg[this.reg.IM];
            // check interupts
            for (let i = 0; i < 8; i++) {
                // If this active
                if (((masked >> i) & 0x01) === 1) {
                    // interupt cleaner
                    this.reg.IS &= ~i;
                    // look up the mem address for the interrupt
                    this.reg.MAR = 255 - (i * 2);
                    this.membus.ADDR = 255;
                    this.membus.READ();
                    // this.reg.PC = this.reg.MAR;
                    this.reg.PC = this.membus.DATA();
                }
            }
            this.flags.INTR = false;
        }

        // run instructions...
        // const currentInstruction = this.mem[this.reg.PC];
        this.membus.ADDR = this.reg.PC;                             // set read/write memory address
        this.membus.READ();                                         // read memory location
        const currentInstruction = this.membus.DATA();              // read data from membus
        // console.log(currentInstruction);
        const handler = this.branchTable[currentInstruction];

        if (handler === undefined) {
            console.error('ERROR: invalid instruction ' + currentInstruction);
            console.log('MEMORY STACK:\n');
            console.log('ROM:\n', this.membus.banks[0].bank);
            console.log('MEM:\n', this.membus.banks[1].bank);
            this.stopClock();
            return;
        }
        handler.call(this); // set this explicitly in handler
    }

    /**
     * arithmatic logic unit
     * @method alu
     * @param  {[type]} func [description]
     * @param  {[type]} r0   [description]
     * @param  {[type]} r1   [description]
     * @return {[type]}      [description]
     */
    alu(func, r0, r1, sv = false) {
        switch (func) {
            case 'INC':
                // increment number
                this.reg[r0]++;
                // check if out of bounds
                if (this.reg[r0] > 255)
                    this.reg[r0] = 255;
                    break;
            case 'DEC':
                // Decrement number
                this.reg[r0]--;
                // check if out of bounds
                if (this.reg[r0] < 0)
                    this.reg[r0] = 255;
                    break;
            case 'ADD':
                if (!sv) return this.reg[r0] + this.reg[r1];
                const a = r0 + r1;
                return (a <= 255) ? r0 + r1 : a - 255;
                break;
            case 'SUB':
                if (!sv) return this.reg[r0] - this.reg[r1];
                const s = r0 - r1;
                return (s >= 0) ? r0 - r1 : 256 + s;
                break;
            case 'MUL':
                if (!sv) return this.reg[r0] * this.reg[r1];
                return r0 * r1;
                break;
            case 'DIV':
                if (!sv) return this.reg[r0] / this.reg[r1];
                return r0 / r1;
                break;
            case 'CMP':
                if (!sv) return this.reg[r0] === this.reg[r1];
                return r0 === r1;
                break;
        }
    }

    /**
     * Init
     *
     */
    INIT() {
        this.flags.INTR = false;
        this.curReg = 0;
        /* set Interupt Mask to 0 so all interupts are blocked. */
        this.membus.ADDR = this.reg.IM;                             // set memory read/write address
        this.membus.ADDRVAL = 0b00000000;                           // set memory value to write
        this.membus.WRITE();                                        // write the memory
        // this.mem[this.reg.IM] = 0b00000000;
        this.reg.PC++;                                              // go to next instruction
    }

    /**
     * (SET) the current (R)egister location
     * @method SET
     */
    SETR() {
        this.membus.ADDR = this.reg.PC + 1;                         // set memory read/write address
        this.membus.READ();                                         // read the memory
        // const reg = this.mem[this.reg.PC + 1];
        // this.curReg = reg;
        this.curReg = this.membus.DATA();                           // read the data from mebus
        this.reg.PC += 2;
    }

    /**
     * (GET) the current (R)egister location
     * @method SET
     * @PRIVATE
     */
    GETR() {
        return this.curReg;
    }

    /**
     * Save the value in the inext instruction line to the current register location
     * @method SAVE
     */
    SAVE() {
        console.log('SAVE...');
        this.membus.ADDR = this.reg.PC + 1;                         // Set memory read/write address
        this.membus.READ();                                         // read memory
        this.reg[this.curReg] = this.membus.DATA();                 // read membus data
        // this.reg[this.curReg] = this.mem[this.reg.PC + 1];
        this.reg.PC += 2;                                           // increment PC by 2
    }

    /**
     * multiply the next two concurrent values in memory and place them into current register location
     * @method MULL
     */
    MUL() {
        this.membus.ADDR = this.reg.PC + 1;                         // Set memory address for first argument
        this.membus.READ();                                         // read memory
        const m1 = this.membus.DATA();                              // get memory read data
        this.membus.ADDR = this.reg.PC + 2;                         // set memory address for next argument
        this.membus.READ();                                         // read memory
        const m2 = this.membus.DATA();                              // get memory read data
        // const m1 = this.mem[this.reg.PC + 1];
        // const m2 = this.mem[this.reg.PC + 2];
        this.reg[this.curReg] = this.alu('MUL', m1, m2);
        this.reg.PC += 3;
    }

    /**
     * Divide two numbers
     * @method DIV
     */
    DIV() {
        this.membus.ADDR = this.reg.PC + 1;                         // Set memory address for first argument
        this.membus.READ();                                         // read memory
        const m1 = this.membus.DATA();                              // get memory read data
        this.membus.ADDR = this.reg.PC + 2;                         // set memory address for next argument
        this.membus.READ();                                         // read memory
        const m2 = this.membus.DATA();                              // get memory read data
        // const m1 = this.mem[this.reg.PC + 1];
        // const m2 = this.mem[this.reg.PC + 2];
        this.reg[this.curReg] = this.alu('DIV', m1, m2);
        this.reg.PC += 3;
    }

    /**
     * add two concurrent numbers together and place in current registry
     * @method ADD
     */
    ADD() {
        this.membus.ADDR = this.reg.PC + 1;                         // Set memory address for first argument
        this.membus.READ();                                         // read memory
        const m1 = this.membus.DATA();                              // get memory read data
        this.membus.ADDR = this.reg.PC + 2;                         // set memory address for next argument
        this.membus.READ();                                         // read memory
        const m2 = this.membus.DATA();                              // get memory read data
        // const m1 = this.mem[this.reg.PC + 1];
        // const m2 = this.mem[this.reg.PC + 2];
        this.reg[this.curReg] = this.alu('ADD', m1, m2);
        this.reg.PC += 3;
    }

    /**
     * subtract one number from another number.
     * @method SUB
     */
    SUB() {
        this.membus.ADDR = this.reg.PC + 1;                         // Set memory address for first argument
        this.membus.READ();                                         // read memory
        const m1 = this.membus.DATA();                              // get memory read data
        this.membus.ADDR = this.reg.PC + 2;                         // set memory address for next argument
        this.membus.READ();                                         // read memory
        const m2 = this.membus.DATA();                              // get memory read data
        // const m1 = this.mem[this.reg.PC + 1];
        // const m2 = this.mem[this.reg.PC + 2];
        this.reg[this.curReg] = this.alu('SUB', m1, m2);
        this.reg.PC += 3;
    }

    /**
     * compare two values
     * @method CMP
     */
    CMP() {
        this.membus.ADDR = this.reg.PC + 1;                         // Set memory address for first argument
        this.membus.READ();                                         // read memory
        const m1 = this.membus.DATA();                              // get memory read data
        this.membus.ADDR = this.reg.PC + 2;                         // set memory address for next argument
        this.membus.READ();                                         // read memory
        const m2 = this.membus.DATA();                              // get memory read data
        // const m1 = this.mem[this.reg.PC + 1];
        // const m2 = this.mem[this.reg.PC + 2];
        this.reg[this.curReg] = this.alu('CMP', m1, m2);
        this.reg.PC += 3;
    }

    /**
     * (PR)int (A)lpha-numeric char
     * @method PRA
     */
    PRA() {
        let mv = this.reg[this.curReg];
        if (typeof mv === 'string') mv = parseInt(mv.padStart(8, '0'), 2);
        process.stdout.write(String.fromCharCode(mv));
        this.reg.PC++;
    }

    /**
     * print the current number.
     * @method PRN
     */
    PRN() {
        console.log(this.reg[this.curReg]);
        this.reg.PC++;
    }

    /**
     * halt the current program in memory.
     * @method HALT
     */
    HALT() {
        console.log('HALT');
        this.stopClock();
    }

    /**
     * Jump to a supplied memory address
     * @method JMP
     */
    JMP() {
        this.membus.ADDR = this.reg.PC + 1;                         // set memory address for read
        this.membus.READ();                                         // read memory
        const m1 = this.membus.DATA();                              // read membus data
        // const m1 = this.mem[this.reg.PC + 1];
        this.reg.PC = m1;
    }

    /**
     * Jump to a supplied memory address if two reg address values are equal
     * @method JEQ
     */
    JEQ() {
        this.membus.ADDR = this.reg.PC + 1;                         // Set memory address for first argument
        this.membus.READ();                                         // read memory
        const m1 = this.membus.DATA();                              // get memory read data
        this.membus.ADDR = this.reg.PC + 2;                         // set memory address for next argument
        this.membus.READ();                                         // read memory
        const m2 = this.membus.DATA();                              // get memory read data
        // const m1 = this.mem[this.reg.PC + 1];
        // const m2 = this.mem[this.reg.PC + 2];
        if (m1 === m2) {
            this.membus.ADDR = this.reg.PC + 3;                     // set memory address for read
            this.membus.READ();                                     // read memory
            this.reg.PC = this.membus.DATA();                       // read membus data
            // this.reg.PC = this.mem[this.reg.PC + 3];
        } else {
            this.reg.PC += 4;
        }
    }

    /**
     * Jump to a supplied mem. address if 2 supplied reg address values are not equal.
     * @method JNE
     */
    JNE() {
        this.membus.ADDR = this.reg.PC + 1;                         // Set memory address for first argument
        this.membus.READ();                                         // read memory
        const m1 = this.membus.DATA();                              // get memory read data
        this.membus.ADDR = this.reg.PC + 2;                         // set memory address for next argument
        this.membus.READ();                                         // read memory
        const m2 = this.membus.DATA();                              // get memory read data
        // const m1 = this.mem[this.reg.PC + 1];
        // const m2 = this.mem[this.reg.PC + 2];
        if (m1 === m2) {
            this.reg.PC += 4;
        } else {
            this.membus.ADDR = this.reg.PC + 3;                     // set memory address for read
            this.membus.READ();                                     // read memory
            this.reg.PC = this.membus.DATA();                       // read membus data
            // this.reg.PC = this.mem[this.reg.PC + 3];
        }
    }

    /**
     * load the value at MAR address in memory into MDR
     * @method MEMLOAD
     */
    MEMLOAD() {
        // set membus address
        this.membus.ADDR = this.reg.MAR;
        // instruct memory to read
        this.membus.READ();
        // set MDR
        this.reg.MDR = this.membus.DATA();
        // this.reg.MDR = this.mem[this.reg.MAR];
    }

    /**
     * store the value in MDR in the mem location in MAR
     * @method MEMSTORE
     */
    MEMSTORE() {
        // Set memory address
        this.membus.ADDR = this.reg.MAR;
        // Set memory write value
        this.membus.ADDRVAL = this.reg.MDR;
        // instruct memory to write
        this.membus.WRITE();
        // this.mem[this.reg.MAR] = this.reg.MDR;
    }

    /**
     * Loa(d) directly from memory
     * @method LD
     */
    LD() {
        // set memory address
        this.membus.ADDR = this.reg.PC + 1;
        // Read memory
        this.membus.READ();
        // set register with memory
        this.reg[this.curReg] = this.membus.DATA();
        // const ml = this.mem[this.reg.PC + 1];
        // this.reg[this.curReg] = m1;
        this.reg.PC += 2;
    }

    /**
     * (St)ore Directly to Memory
     * @method ST
     */
    ST() {
        this.membus.ADDR = this.reg.PC + 1;                         // set memory read/write address
        this.membus.READ();                                         // read memory location
        const ml = this.membus.DATA();                              // read memory data
        // const ml = this.mem[this.reg.PC + 1];
        this.membus.ADDR = ml;                                      // set memory read/write address
        this.membus.ADDRVAL = this.reg[this.curReg];                // set memory write value
        this.membus.WRITE();                                        // write value to mem address
        // this.mem[ml] = this.reg[this.curReg];
        this.reg.pc += 2;                                           // increment counter 2
    }

    /**
     * (L)oa(d)-(R)egister-(I)ndirect
     * @method LDRI
     */
    LDRI() {
        this.membus.ADDR = this.reg.PC + 1;                         // set memory read/write address
        this.membus.READ();                                         // read memory location
        const rl = this.membus.DATA();                              // read memory data
        // const rl = this.mem[this.reg.PC + 1];
        this.membus.ADDR = rl;                                      // set memory read/write address
        this.membus.READ();                                         // read memory from address
        this.reg[this.curReg] = this.membus.DATA();                 // read memory data
        // this.reg[this.curReg] = this.mem[rl];
        this.reg.PC += 2;                                           // increment counter 2
    }

    /**
     * (St)ore (R)egister (I)ndirect
     * @method STRI
     */
    STRI() {
        this.membus.ADDR = this.reg.PC + 1;                         // set memory read/write address
        this.membus.READ();                                         // read memory location
        const rl = this.membus.DATA;                                // read memory data
        // const rl = this.mem[this.reg.PC + 1];
        const ml = this.reg[rl];
        this.membus.ADDR = ml;                                      // set memory read/write address
        this.membus.ADDRVAL = this.reg[this.curReg];                // set memory data to write
        this.membus.WRITE();                                        // write data to memory location
        // this.mem[ml] = this.reg[this.curReg];
        this.reg.PC += 2;                                           // increment counter 2
    }

    /**
     * Set the PC.  if out of bounds, set to neutral number.
     * @method SPC
     */
    SPC() {
        // TODO: set bounds of max min;
        // TODO: make sure it allows for a little before for terminal running.
        // TODO: need a good memory set for this.
    }

    /**
     * Copy Range (inclusive)
     * @method CPYR
     */
    CPYR() {
        this.membus.ADDR = this.reg.PC + 1;
        this.membus.READ();
        const from = this.membus.DATA();
        this.membus.ADDR = this.reg.PC + 2;
        this.membus.READ();
        const to = this.membus.DATA();

        for(let i = from; i <= to; i++) {
            this.membus.ADDR = i;
            this.membus.READ();
            const data = this.membus.DATA();
            this.membus.ADDRVAL = data;
            this.membus.WRITE();
        }

        this.reg.PC += 3;
        // TODO: this should take more than 1 cycle if more than x but we're going to do that later.
    }

    /**
     * address a memory block for read and write
     * @method ADR
     */
    ADR() {
        this.reg[this.curReg] = this.reg.SRM;
    }

    /**
     * set read address block
     * @method RAD
     */
    RAD() {
        // shift bits left by 4
        // const read = this.reg[this.curReg] << 4;
        // switching from looking for value in reg to value in next mem location
        this.membus.ADDR = this.reg.PC + 1;                         // set memory read/write address
        this.membus.READ();                                         // read memory location
        const read = this.membus.DATA();                            // read data from memory
        read <<= 4;
        // get right 4 bits
        const write = this.reg.SRM & 0b00001111;
        // assign the SMR the or of both.  (combine)
        this.reg.SMR = read | write;
        this.membus.setBANK(this.reg.SMR);                          // set mem banks
        this.reg.PC += 2;
    }

    /**
     * set read address block from register
     * @method RADFR
     */
    RADFR() {
        // shift bits left by 4
        const read = this.reg[this.curReg] << 4;
        // get right 4 bits
        const write = this.reg.SRM & 0b00001111;
        // assign the SMR the or of both.  (combine)
        this.reg.SMR = read | write;
        this.reg.PC += 1;
    }

    /**
     * set write address block
     * @method WAD
     */
    WAD() {
        // get write bits
        // // switching from looking for value in reg to value in next mem location
        // const write = this.reg[this.curReg];
        this.membus.ADDR = this.reg.PC + 1;                         // set memory read/write address
        this.membus.READ();                                         // read memory location
        const write = this.membus.DATA();                           // read data from memory
        // get current read bits and mask out write bits
        const read = this.reg.SRM &0b11110000;
        // assign read and write
        this.reg.SMR = read | write;
        this.reg.PC += 2;
    }

    /**
     * set write address block from register
     * @method WADFR
     */
    WADFR() {
        // get write bits
        const write = this.reg[this.curReg];
        // get current read bits and mask out write bits
        const read = this.reg.SRM &0b11110000;
        // assign read and write
        this.reg.SMR = read | write;
        this.reg.PC += 1;
    }

    /**
     * read address for block
     * @method RADR
     */
    RADR() {

    }

    /**
     * Read SRM value and place into current register
     * @method RSRM
     */
    rsrm() {
        this.reg[this.curReg] = this.reg.SRM;
    }

    /**
     * (SET) (I)nterrupt Vector (address)
     * @method SETI
     */
    SETI() {
        this.membus.ADDR = this.alu('ADD', this.reg.PC, 0b00000001, true);
        this.membus.READ();
        const interruptNum = this.membus.DATA();                            // Read interrupt index                     EX: 0   /   1   /   2

        const intVertex = this.alu('MUL', interruptNum, 0b00000010, true);  // Interrupt Vertex, not interrupt store    EX: 0   /   2   /   4
        const lastIndex = this.alu('SUB', 0b00000000, 0b00000001, true);    // Get Last memory address                  Ex: 255

        console.log('last index: ', lastIndex);

        const interLoc = this.alu('SUB', lastIndex, intVertex, true);       // Interrupt memory location                EX: 255 /   253 /   251

        console.log('interrupt location address: ', interLoc);

        this.membus.ADDR = this.alu('ADD', this.reg.PC, 0b00000010, true);  // Get second argument for
        this.membus.READ();                                                 // location of interrupt in memory
        const interruptDataLoc = this.membus.DATA();

        console.log(`Interrupt Memory Jump Location: ${interruptDataLoc}`);

        this.membus.ADDR = interLoc;
        this.membus.ADDRVAL =   interruptDataLoc;
        this.membus.WRITE();

        this.reg.PC += 2;
    }

    /**
     * (GET) (I)nterrupt Vector (address)
     * @method GETI
     */
    GETI() {
        this.membus.ADDR = this.alu('ADD', this.reg.PC, 0b00000001, true);
        this.membus.READ();
        const interruptNum = this.membus.DATA();                        // Read interrupt index
        const intVertex = this.alu('MUL', interruptNum, 0b00000010, true);    // Interrupt Vertex, not interrupt store
        const lastIndex = this.alu('SUB', 0b00000000, 0b00000001, true);      // Get Last memory address
        const interLoc = this.alu('SUB', lastIndex, intVertex);         // Interrupt memory location
        this.membus.ADDR = interLoc;
        this.membus.READ();
        this.reg[this.curReg] = interLoc;
    }

    /**
     * (RET)urn from (I)nterrupt
     * @method RETI
     */
    RETI() {
        this.POP
    }

    /**
     * Call a subroutine at a specific location on next instruction
     * @method CALL
     */
    CALL() {
        this.curReg = 254;
        this.reg[this.curReg] = this.reg.PC + 2;
        this.PUSH();
        // this.reg.PC = this.mem[this.reg.PC + 1];
        this.membus.ADDR = this.reg.PC + 1;                         // set memory read/write address
        this.membus.READ();                                         // read from memory address
        this.reg.PC = this.membus.DATA();                           // read memory data
    }

    /**
     * Return to the last line in the stack
     * @method RET
     */
    RET() {
        this.curReg = 254;
        this.POP();
        this.reg.PC = this.reg[this.curReg];
    }

    /**
     * Pop from stack
     * @method POP
     */
    POP() {
        // decremenet the SP holder
        this.alu('INC', SP);
        // store current Register at the location of the SP
        this.membus.ADDR = SP;                                      // set memory read/write address
        this.membus.READ();                                         // read memory location
        this.reg[this.curReg] = this.membus.DATA();                 // read memory data
        // this.reg[this.curReg] = this.mem[SP];
    }

    /**
     * Push to stack
     * @method PUSH
     */
    PUSH() {
        // store the current Register at the SP location in memory
        this.membus.ADDR = SP;                                      // set memory read/write address
        this.membus.ADDRVAL = this.reg[this.curReg];                // set memory data
        this.membus.WRITE();                                        // write to memory
        // this.mem[SP] = this.reg[this.curReg];
        // Incremeent the SP holder
        this.alu('DEC', SP)
    }
}

module.exports = CPU;

/*
             _____        __________ ________________
            /    /       /           \               \
           /    /       /    ________/   ________     \
          /    /       /    /     |      |      |      |
         /    /       /     \____  \     \______/     /
        /    /        \_______    \  >              <
       /    /                /    //     /------\     \
      /    /______  ________/    /|      |______|      |
     /           / /            /  \                  /
    |___________/  \___________/    \________________/



             _____        __________ ________________
            /    /       /           \           ^*&#\
           /    /       /    ________/   ________  *&#\
          /    /       /    /     |     #|      |   *&#|
         /    /       /     \____  \    #\______/  *&#/
        /    /        \_______    \  >              <
       /    /                /    //    #/------\  *&#\
      /    /______  ________/    /|     #|______|   *&#|
     /           / /            /  \     #######   *&#/
    |___________/  \___________/    \_____________&##/




00001000 00001000 00001000 00001000 00001000 00001000 00001000
00001000 00001000 00000100 00001000 00001000 00001000 00001000
00001000 000/    / 000010/           \001000 00001000 00001000
00001000 00/    /0 00001/    ________/001000 00001000 00001000
00001000 0/    /00 0000/    /001000 00001000 00001000 00001000
00001000 /    /000 000/     \001000 00001000 00001000 00001000
00001000/    /1000 000\_______    \ 00001000 00001000 00001000
0000100/    /01000 00001000 0/    / 00001000 00001000 00001000
000010/    /001000 00001000 /    /0 00001000 00001000 00001000
00001/           / /            /00 00001000 00001000 00001000
0000|___________/ 0\___________/000 00001000 00001000 00001000
00001000 00001000 00001000 00001000 00001000 00001000 00001000
00001000 00001000 00001000 00001000 00001000 00001000 00001000



+------------------------------------------------------------------------------+
|                                CPU CYCLE MAP                                 |
|------------------------------------------------------------------------------|
|                                                                              |
|  +-TICK                                                                      |
|  |                                                                           |
|  +---------------+  +-----+  +-------------+  +--------                      |
|  | IS INTERRUPT? |--| yes |--| STORE STATE |--|                              |
|  +---------------+  +-----+  | TO STACK    |                                 |
|               |              +-------------+                                 |
|             +----+                                                           |
|  +----------| no |                                                           |
|  |          +----+                                                           |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|------------------------------------------------------------------------------|
|                                    FLAGS                                     |
|------------------------------------------------------------------------------|
| LABEL | ADDR | DEFAULT  |                    DESCRIPTION                     |
|-------|------|----------|----------------------------------------------------|
| WAIT  |  000 |        0 | CPU Wait for Instruction / Interrupt               |
| INTR  |  001 |        0 | Interrupt                                          |
| EQLS  |  010 |        0 | Equals Comparison                                  |
| BOOL  |  011 |        0 | Boolean Comparison                                 |
| OVFL  |  100 |        0 | Overflow Value                                     |
| ARNX  |  101 |        0 | Arithmatic Logic Unit Next Instruction             |
| MRNX  |  110 |        0 | Memory Read Next Instruction                       |
| MWNX  |  111 |        0 | Memory Write Next Instruction                      |
|       |      |          |                                                    |
|       |      |          |                                                    |
|       |      |          |                                                    |
|       |      |          |                                                    |
|------------------------------------------------------------------------------|
|                                  REGISTERS                                   |
|------------------------------------------------------------------------------|
| LABEL |   ADDR   | HEX  | DEFAULT  | HEX  |           DESCRIPTION            |
|-------|----------|------|----------|------|----------------------------------|
| rar   | 00000000 | 0x00 | 00000000 | 0x00 | Arithmatic Result Register       |
| rao   | 00000001 | 0x01 | 00000000 | 0x00 | Arithmatic Argument One          |
| rat   | 00000010 | 0x02 | 00000000 | 0x00 | Arithmatic Argument Two          |
| rsp   | 00000011 | 0x03 | 11111111 | 0xff | Stack Pointer Register           |
| mde   | 00000100 | 0x04 | 00000000 | 0x00 | Software Mode Register           |
| pc    | 00000101 | 0x05 | 00000000 | 0x00 | Program Counter Register         |
|       | 00000110 | 0x06 | 00000000 |      |                                  |
|       | 00000111 | 0x07 | 00000000 |      |                                  |
| rs0   | 00001000 | 0x08 | 00000000 | 0x00 | Register Storage 0               |
| rs1   | 00001001 | 0x09 | 00000000 | 0x00 | Register Storage 1               |
| rs2   | 00001010 | 0x0a | 00000000 | 0x00 | Register Storage 2               |
| rs3   | 00001011 | 0x0b | 00000000 | 0x00 | Register Storage 3               |
| rs4   | 00001100 | 0x0c | 00000000 | 0x00 | Register Storage 4               |
| rs5   | 00001101 | 0x0d | 00000000 | 0x00 | Register Storage 5               |
| rs6   | 00001110 | 0x0e | 00000000 | 0x00 | Register Storage 6               |
| rs7   | 00001111 | 0x0f | 00000000 | 0x00 | Register Storage 7               |
|------------------------------------------------------------------------------|
|                                   ROM MAP                                    |
|------------------------------------------------------------------------------|
|   ADDR   | HEX  |   LABEL   | INSTR |      DESCRIPTION                       |
|----------|------|-----------|-------|----------------------------------------|
| 00000000 | 0x00 |           |       |                                        |
| 00000001 | 0x01 |           |       |                                        |
| 00000010 | 0x02 |           |       |                                        |
| 00000011 | 0x03 |           |       |                                        |
| 00000100 | 0x04 |           |       |                                        |
| 00000101 | 0x05 |           |       |                                        |
| 00000110 | 0x06 |           |       |                                        |
| 00000111 | 0x07 |           |       |                                        |
| 00001000 | 0x08 |           |       |                                        |
| 00001001 | 0x09 |           |       |                                        |
| 00001010 | 0x0a |           |       |                                        |
| 00001011 | 0x0b |           |       |                                        |
| 00001100 | 0x0c |           |       |                                        |
| 00001101 | 0x0d |           |       |                                        |
| 00001110 | 0x0e |           |       |                                        |
| 00001111 | 0x0f |           |       |                                        |
| 00010000 | 0x10 |           |       |                                        |
| 00010001 | 0x11 |           |       |                                        |
| 00010010 | 0x12 |           |       |                                        |
| 00010011 | 0x13 |           |       |                                        |
| 00010100 | 0x14 |           |       |                                        |
| 00010101 | 0x15 |           |       |                                        |
| 00010110 | 0x16 |           |       |                                        |
| 00010111 | 0x17 |           |       |                                        |
| 00011000 | 0x18 |           |       |                                        |
| 00011001 | 0x19 |           |       |                                        |
| 00011010 | 0x1a |           |       |                                        |
| 00011011 | 0x1b |           |       |                                        |
| 00011100 | 0x1c |           |       |                                        |
| 00011101 | 0x1d |           |       |                                        |
| 00011110 | 0x1e |           |       |                                        |
| 00011111 | 0x1f |           |       |                                        |
| 00100000 | 0x20 |           |       |                                        |
| 00100001 | 0x21 |           |       |                                        |
| 00100010 | 0x22 |           |       |                                        |
| 00100011 | 0x23 |           |       |                                        |
| 00100100 | 0x24 |           |       |                                        |
| 00100101 | 0x25 |           |       |                                        |
| 00100110 | 0x26 |           |       |                                        |
| 00100111 | 0x27 |           |       |                                        |
| 00101000 | 0x28 |           |       |                                        |
| 00101001 | 0x29 |           |       |                                        |
| 00101010 | 0x2a |           |       |                                        |
| 00101011 | 0x2b |           |       |                                        |
| 00101100 | 0x2c |           |       |                                        |
| 00101101 | 0x2d |           |       |                                        |
| 00101110 | 0x2e |           |       |                                        |
| 00101111 | 0x2f |           |       |                                        |
| 00110000 | 0x30 |           |       |                                        |
| 00110001 | 0x31 |           |       |                                        |
| 00110010 | 0x32 |           |       |                                        |
| 00110011 | 0x33 |           |       |                                        |
| 00110100 | 0x34 |           |       |                                        |
| 00110101 | 0x35 |           |       |                                        |
| 00110110 | 0x36 |           |       |                                        |
| 00110111 | 0x37 |           |       |                                        |
| 00111000 | 0x38 |           |       |                                        |
| 00111001 | 0x39 |           |       |                                        |
| 00111010 | 0x3a |           |       |                                        |
| 00111011 | 0x3b |           |       |                                        |
| 00111100 | 0x3c |           |       |                                        |
| 00111101 | 0x3d |           |       |                                        |
| 00111110 | 0x3e |           |       |                                        |
| 00111111 | 0x3f |           |       |                                        |
| 01000000 | 0x40 |           |       |                                        |
| 01000001 | 0x41 |           |       |                                        |
| 01000010 | 0x42 |           |       |                                        |
| 01000011 | 0x43 |           |       |                                        |
| 01000100 | 0x44 |           |       |                                        |
| 01000101 | 0x45 |           |       |                                        |
| 01000110 | 0x46 |           |       |                                        |
| 01000111 | 0x47 |           |       |                                        |
| 01001000 | 0x48 |           |       |                                        |
| 01001001 | 0x49 |           |       |                                        |
| 01001010 | 0x4a |           |       |                                        |
| 01001011 | 0x4b |           |       |                                        |
| 01001100 | 0x4c |           |       |                                        |
| 01001101 | 0x4d |           |       |                                        |
| 01001110 | 0x4e |           |       |                                        |
| 01001111 | 0x4f |           |       |                                        |
|          |      |                   |                                        |
|          |      |                   |                                        |
|          |      |                   |                                        |
|          |      |                   |                                        |
|          |      |                   |                                        |
| 11111110 | 0xfe |                   |                                        |
| 11111111 | 0xff |                   |                                        |
|------------------------------------------------------------------------------|
|                                  MEMORY MAP                                  |
|------------------------------------------------------------------------------|
|   ADDR   | HEX  |       LABEL       |              DESCRIPTION               |
|----------|------|-------------------|----------------------------------------|
| 00000000 | 0x00 |                   |                                        |
| 00000001 | 0x01 |                   |                                        |
| 00000010 | 0x02 |                   |                                        |
| 00000011 | 0x03 |                   |                                        |
| 00000100 | 0x04 |                   |                                        |
| 00000101 | 0x05 |                   |                                        |
| 00000110 | 0x06 |                   |                                        |
| 00000111 | 0x07 |                   |                                        |
| 00001000 | 0x08 |                   |                                        |
| 00001001 | 0x09 |                   |                                        |
| 00001010 | 0x0a |                   |                                        |
| 00001011 | 0x0b |                   |                                        |
| 00001100 | 0x0c |                   |                                        |
| 00001101 | 0x0d |                   |                                        |
| 00001110 | 0x0e |                   |                                        |
| 00001111 | 0x0f |                   |                                        |
| 00010000 | 0x10 |                   |                                        |
| 00010001 | 0x11 |                   |                                        |
| 00010010 | 0x12 |                   |                                        |
| 00010011 | 0x13 |                   |                                        |
| 00010100 | 0x14 |                   |                                        |
| 00010101 | 0x15 |                   |                                        |
| 00010110 | 0x16 |                   |                                        |
| 00010111 | 0x17 |                   |                                        |
| 00011000 | 0x18 |                   |                                        |
| 00011001 | 0x19 |                   |                                        |
| 00011010 | 0x1a |                   |                                        |
| 00011011 | 0x1b |                   |                                        |
| 00011100 | 0x1c |                   |                                        |
| 00011101 | 0x1d |                   |                                        |
| 00011110 | 0x1e |                   |                                        |
| 00011111 | 0x1f |                   |                                        |
| 00100000 | 0x20 |                   |                                        |
| 00100001 | 0x21 |                   |                                        |
| 00100010 | 0x22 |                   |                                        |
| 00100011 | 0x23 |                   |                                        |
| 00100100 | 0x24 |                   |                                        |
| 00100101 | 0x25 |                   |                                        |
| 00100110 | 0x26 |                   |                                        |
| 00100111 | 0x27 |                   |                                        |
| 00101000 | 0x28 |                   |                                        |
| 00101001 | 0x29 |                   |                                        |
| 00101010 | 0x2a |                   |                                        |
| 00101011 | 0x2b |                   |                                        |
| 00101100 | 0x2c |                   |                                        |
| 00101101 | 0x2d |                   |                                        |
| 00101110 | 0x2e |                   |                                        |
| 00101111 | 0x2f |                   |                                        |
| 00110000 | 0x30 |                   |                                        |
| 00110001 | 0x31 |                   |                                        |
| 00110010 | 0x32 |                   |                                        |
| 00110011 | 0x33 |                   |                                        |
| 00110100 | 0x34 |                   |                                        |
| 00110101 | 0x35 |                   |                                        |
| 00110110 | 0x36 |                   |                                        |
| 00110111 | 0x37 |                   |                                        |
| 00111000 | 0x38 |                   |                                        |
| 00111001 | 0x39 |                   |                                        |
| 00111010 | 0x3a |                   |                                        |
| 00111011 | 0x3b |                   |                                        |
| 00111100 | 0x3c |                   |                                        |
| 00111101 | 0x3d |                   |                                        |
| 00111110 | 0x3e |                   |                                        |
| 00111111 | 0x3f |                   |                                        |
| 01000000 | 0x40 |                   |                                        |
| 01000001 | 0x41 |                   |                                        |
| 01000010 | 0x42 |                   |                                        |
| 01000011 | 0x43 |                   |                                        |
| 01000100 | 0x44 |                   |                                        |
| 01000101 | 0x45 |                   |                                        |
| 01000110 | 0x46 |                   |                                        |
| 01000111 | 0x47 |                   |                                        |
| 01001000 | 0x48 |                   |                                        |
| 01001001 | 0x49 |                   |                                        |
| 01001010 | 0x4a |                   |                                        |
| 01001011 | 0x4b |                   |                                        |
| 01001100 | 0x4c |                   |                                        |
| 01001101 | 0x4d |                   |                                        |
| 01001110 | 0x4e |                   |                                        |
| 01001111 | 0x4f |                   |                                        |
| 01010000 | 0x50 |                   |                                        |
| 01010001 | 0x51 |                   |                                        |
| 01010010 | 0x52 |                   |                                        |
| 01010011 | 0x53 |                   |                                        |
| 01010100 | 0x54 |                   |                                        |
| 01010101 | 0x55 |                   |                                        |
| 01010110 | 0x56 |                   |                                        |
| 01010111 | 0x57 |                   |                                        |
| 01011000 | 0x58 |                   |                                        |
| 01011001 | 0x59 |                   |                                        |
| 01011010 | 0x5a |                   |                                        |
| 01011011 | 0x5b |                   |                                        |
| 01011100 | 0x5c |                   |                                        |
| 01011101 | 0x5d |                   |                                        |
| 01011110 | 0x5e |                   |                                        |
| 01011111 | 0x5f |                   |                                        |
| 01100000 | 0x60 |                   |                                        |
| 01100001 | 0x61 |                   |                                        |
| 01100010 | 0x62 |                   |                                        |
| 01100011 | 0x63 |                   |                                        |
| 01100100 | 0x64 |                   |                                        |
| 01100101 | 0x65 |                   |                                        |
| 01100110 | 0x66 |                   |                                        |
| 01100111 | 0x67 |                   |                                        |
| 01101000 | 0x68 |                   |                                        |
| 01101001 | 0x69 |                   |                                        |
| 01101010 | 0x6a |                   |                                        |
| 01101011 | 0x6b |                   |                                        |
| 01101100 | 0x6c |                   |                                        |
| 01101101 | 0x6d |                   |                                        |
| 01101110 | 0x6e |                   |                                        |
| 01101111 | 0x6f |                   |                                        |
| 01110000 | 0x70 |                   |                                        |
| 01110001 | 0x71 |                   |                                        |
| 01110010 | 0x72 |                   |                                        |
| 01110011 | 0x73 |                   |                                        |
| 01110100 | 0x74 |                   |                                        |
| 01110101 | 0x75 |                   |                                        |
| 01110110 | 0x76 |                   |                                        |
| 01110111 | 0x77 |                   |                                        |
| 01111000 | 0x78 |                   |                                        |
| 01111001 | 0x79 |                   |                                        |
| 01111010 | 0x7a |                   |                                        |
| 01111011 | 0x7b |                   |                                        |
| 01111100 | 0x7c |                   |                                        |
| 01111101 | 0x7d |                   |                                        |
| 01111110 | 0x7e |                   |                                        |
| 01111111 | 0x7f |                   |                                        |
| 10000000 | 0x80 |                   |                                        |
| 10000001 | 0x81 |                   |                                        |
| 10000010 | 0x82 |                   |                                        |
| 10000011 | 0x83 |                   |                                        |
| 10000100 | 0x84 |                   |                                        |
| 10000101 | 0x85 |                   |                                        |
| 10000110 | 0x86 |                   |                                        |
| 10000111 | 0x87 |                   |                                        |
| 10001000 | 0x88 |                   |                                        |
| 10001001 | 0x89 |                   |                                        |
| 10001010 | 0x8a |                   |                                        |
| 10001011 | 0x8b |                   |                                        |
| 10001100 | 0x8c |                   |                                        |
| 10001101 | 0x8d |                   |                                        |
| 10001110 | 0x8e |                   |                                        |
| 10001111 | 0x8f |                   |                                        |
| 10010000 | 0x90 |                   |                                        |
| 10010001 | 0x91 |                   |                                        |
| 10010010 | 0x92 |                   |                                        |
| 10010011 | 0x93 |                   |                                        |
| 10010100 | 0x94 |                   |                                        |
| 10010101 | 0x95 |                   |                                        |
| 10010110 | 0x96 |                   |                                        |
| 10010111 | 0x97 |                   |                                        |
| 10011000 | 0x98 |                   |                                        |
| 10011001 | 0x99 |                   |                                        |
| 10011010 | 0x9a |                   |                                        |
| 10011011 | 0x9b |                   |                                        |
| 10011100 | 0x9c |                   |                                        |
| 10011101 | 0x9d |                   |                                        |
| 10011110 | 0x9e |                   |                                        |
| 10011111 | 0x9f |                   |                                        |
| 10100000 | 0xa0 |                   |                                        |
| 10100001 | 0xa1 |                   |                                        |
| 10100010 | 0xa2 |                   |                                        |
| 10100011 | 0xa3 |                   |                                        |
| 10100100 | 0xa4 |                   |                                        |
| 10100101 | 0xa5 |                   |                                        |
| 10100110 | 0xa6 |                   |                                        |
| 10100111 | 0xa7 |                   |                                        |
| 10101000 | 0xa8 |                   |                                        |
| 10101001 | 0xa9 |                   |                                        |
| 10101010 | 0xaa |                   |                                        |
| 10101011 | 0xab |                   |                                        |
| 10101100 | 0xac |                   |                                        |
| 10101101 | 0xad |                   |                                        |
| 10101110 | 0xae |                   |                                        |
| 10101111 | 0xaf |                   |                                        |
| 10110000 | 0xb0 |                   |                                        |
| 10110001 | 0xb1 |                   |                                        |
| 10110010 | 0xb2 |                   |                                        |
| 10110011 | 0xb3 |                   |                                        |
| 10110100 | 0xb4 |                   |                                        |
| 10110101 | 0xb5 |                   |                                        |
| 10110110 | 0xb6 |                   |                                        |
| 10110111 | 0xb7 |                   |                                        |
| 10111000 | 0xb8 |                   |                                        |
| 10111001 | 0xb9 |                   |                                        |
| 10111010 | 0xba |                   |                                        |
| 10111011 | 0xbb |                   |                                        |
| 10111100 | 0xbc |                   |                                        |
| 10111101 | 0xbd |                   |                                        |
| 10111110 | 0xbe |                   |                                        |
| 10111111 | 0xbf |                   |                                        |
| 11000000 | 0xc0 |                   |                                        |
| 11000001 | 0xc1 |                   |                                        |
| 11000010 | 0xc2 |                   |                                        |
| 11000011 | 0xc3 |                   |                                        |
| 11000100 | 0xc4 |                   |                                        |
| 11000101 | 0xc5 |                   |                                        |
| 11000110 | 0xc6 |                   |                                        |
| 11000111 | 0xc7 |                   |                                        |
| 11001000 | 0xc8 |                   |                                        |
| 11001001 | ERR  | ERR ERR ERR ERR E | ERR ERR ERR ERR ERR ERR ERR ERR ERR ER | ERR ERR ERR ERR ERR
| 11001010 | 0xc9 |                   |                                        |
| 11001011 | 0xd0 |                   |                                        |
| 11001100 | 0xd1 |                   |                                        |
| 11001101 | 0xd2 |                   |                                        |
| 11001110 | 0xd3 |                   |                                        |
| 11001111 | 0xd4 |                   |                                        |
| 11010000 | 0xd5 |                   |                                        |
| 11010001 | 0xd6 |                   |                                        |
| 11010010 | 0xd7 |                   |                                        |
| 11010011 | 0xd8 |                   |                                        |
| 11010100 | 0xd9 |                   |                                        |
| 11010101 | 0xe0 |                   |                                        |
| 11010110 | 0xe1 |                   |                                        |
| 11010111 | 0xe2 |                   |                                        |
| 11011000 | 0xe3 |                   |                                        |
| 11011001 | 0xe4 |                   |                                        |
| 11011010 | 0xe5 |                   |                                        |
| 11011011 | 0xe6 |                   |                                        |
| 11011100 | 0xe7 |                   |                                        |
| 11011101 | 0xe8 |stack              |                                        |
| 11011110 | 0xe9 | stack             |                                        |
| 11011111 | 0xea |  stack            |                                        |
| 11100000 | 0xeb |   stack           |                                        |
| 11100001 | 0xec |    stack          |                                        |
| 11100010 | 0xed |     stack         |                                        |
| 11101110 | 0xee |      stack        |                                        | ERR ERR ERR ERR
| 11101111 | 0xef |       stack       |                                        |
| 11110000 | 0xf0 |        stack      |                                        |
| 11110001 | 0xf1 |         stack     |                                        |
| 11110010 | 0xf2 |          stack    |                                        |
| 11110011 | 0xf3 |           stack   |                                        |
| 11110100 | 0xf4 |            stack  |                                        |
| 11110101 | 0xf5 |             stack |                                        |
| 11110110 | 0xf6 |              stack|                                        |
| 11110111 | 0xf7 | STACK_FLOOR       |                                        |
| 11111000 | 0xf8 | INTER_STORx4      |                                        |
| 11111001 | 0xf9 | INTER_ADDRx4      |                                        |
| 11111010 | 0xfa | INTER_STORx3      |                                        |
| 11111011 | 0xfb | INTER_ADDRx3      |                                        |
| 11111100 | 0xfc | INTER_STORx1      |                                        |
| 11111101 | 0xfd | INTER_ADDRx1      |                                        |
| 11111110 | 0xfe | INTER_STORx0      |                                        |
| 11111111 | 0xff | INTER_ADDRx0      |                                        |
+------------------------------------------------------------------------------+



*/
