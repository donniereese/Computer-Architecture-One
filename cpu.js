const INIT  = 0b00000001;   // Initialize
const SET   = 0b00000010;   //
const STOR  = 0b00000100;   //
const LOAD  = 0b00000111;   //
const MUL   = 0b00000101;   //
const PRN   = 0b00000110;   //
const HALT  = 0b00000000;   //

// Output Extension
const PRA   = 0b01000001;   //

// Load and store extensions
const LD    = 0b00001000;   //
const ST    = 0b00001001;   //
const LDRI  = 0b00010010;   //
const STRI  = 0b00010011;   //

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

// Interupts
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
        };
        // Set carryover flag
        this.CO = 0;

        // Set SRM to rom
        this.reg.SRM = 0b00010001;
        // memory data register
        this.reg.MDR = 0b00000000;
        // memory address register
        this.reg.MAR = 0b00000000;;

        // Bus
        this.membus = bus;

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
            [SET]: this.SET,
            [STOR]: this.SAVE,
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
        if (LOAD) bt[LOAD] = this.MEMLOAD;

        this.branchTable = bt;
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
        if (this.flags.INTR) {
            // Mask the binary
            const masked = this.reg[this.reg.IS] & this.reg[this.reg.IM];
            // check interupts
            for (let i = 0; i < 8; i++) {
                // If this active
                if (((masked >> i) & 0x01) === 1) {
                    // interupt cleaner
                    this.reg.IS &= ~i;

                    // look up the mem address for the interrupt
                    this.reg.MAR = 255 - i;
                }
            }
        }

        // run instructions...
        // const currentInstruction = this.mem[this.reg.PC];
        this.membus.ADDR = this.reg.PC;                             // set read/write memory address
        this.membus.READ();                                         // read memory location
        const currentInstruction = this.membus.DATA();                  // read data from membus
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
    alu(func, r0, r1) {
        switch (func) {
            case 'INC':
                // increment number
                this.reg[r0]++;
                // check if out of bounds
                if (this.reg[r0] > 246)
                    this.reg[r0] = 246;
                    break;
            case 'DEC':
                // Decrement number
                this.reg[r0]--;
                // check if out of bounds
                if (this.reg[r0] < 0)
                    this.reg[r0] = 246;
                    break;
            case 'ADD':
                return this.reg[r0] + this.reg[r1];
                break;
            case 'SUB':
                return this.reg[r0] - this.reg[r1];
                break;
            case 'MUL':
                return this.reg[r0] * this.reg[r1];
                break;
            case 'DIV':
                return this.reg[r0] / this.reg[r1];
                break;
            case 'CMP':
                return this.reg[r0] === this.reg[r1];
                break;
        }
    }

    /**
     * Init
     *
     */
    INIT() {
        this.flags.INTR = true;
        this.curReg = 0;
        /* set Interupt Mask to 0 so all interupts are blocked. */
        this.membus.ADDR = this.reg.IM;                             // set memory read/write address
        this.membus.ADDRVAL = 0b00000000;                           // set memory value to write
        this.membus.WRITE();                                        // write the memory
        // this.mem[this.reg.IM] = 0b00000000;
        this.reg.PC++;                                              // go to next instruction
    }

    /**
     * set the current register location
     * @method SET
     */
    SET() {
        this.membus.ADDR = this.reg.PC + 1;                         // set memory read/write address
        this.membus.READ();                                         // read the memory
        // const reg = this.mem[this.reg.PC + 1];
        // this.curReg = reg;
        this.curReg = this.membus.DATA();                           // read the data from mebus
        this.reg.PC += 2;
    }

    /**
     * Save the value in the inext instruction line to the current register location
     * @method SAVE
     */
    SAVE() {
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
        const read = this.reg[this.curReg] << 4;
        // get right 4 bits
        const write = this.reg.SRM & 0b00001111;
        // assign the SMR the or of both.  (combine)
        this.reg.SMR = read | write;
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
    }

    /**
     * set write address block
     * @method WAD
     */
    WAD() {
        // get write bits
        const write = this.reg[this.curReg];
        // get current read bits and mask out write bits
        const read = this.reg.SRM &0b11110000;
        // assign read and write
        this.reg.SMR = read | write;
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
    }

    /**
     * read address for block
     * @method RADR
     */
    RADR() {

    }

    /**
     * Switch running between Ram or Memory
     * @method SRM
     */
    smr() {
        switch(this.reg.SRM) {
            case 2:
                // Read Catridge
                break;
            case 1:
                // Read Memory
                break;
            case 0:
            default:
                // Read ROM
        }
    }

    /**
     * Read SRM value and place into current register
     * @method RSRM
     */
    rsrm() {
        this.reg[this.curReg] = this.reg.SRM;
    }

    /**
     * Load from location specified by SRM (Bank Switch).
     * You don't specify memory or rom, just get data from switched based on SRM
     * @method LSRM
     */
    lsrm() {
        // TODO: Replace this with bitwise processing.
        switch(this.reg.SRM) {
            case 2:
                // Read Catridge
                break;
            case 1:
                // Read Memory
                break;
            case 0:
            default:
                // Read ROM
        }
    }

    /**
     * Store in location at SRM (Bank Switch) set location.
     * @method SSRM
     */
    ssrm() {
        switch(this.reg.SRM) {
            case 2:
                // Read Catridge
                break;
            case 1:
                // Read Memory
                break;
            case 0:
            default:
                // Read ROM
        }
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
