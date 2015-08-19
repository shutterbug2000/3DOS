// 8086tiny: a tiny, highly functional, highly portable PC emulator/VM
// Copyright 2013-14, Adrian Cable (adrian.cable@gmail.com) - http://www.megalith.co.uk/8086tiny
//
// Revision 1.25
//
// This work is licensed under the MIT License. See included LICENSE.TXT.

#include <time.h>
#include <sys/timeb.h>
#include <assert.h>

#include <3ds.h>
#include "3dos.h"


// Global variable definitions
unsigned char mem[RAM_SIZE], io_ports[IO_PORT_COUNT], *opcode_stream, *regs8, i_rm, i_w, i_reg,
    i_mod, i_mod_size, i_d, i_reg4bit, raw_opcode_id, xlat_opcode_id, extra, rep_mode,
    seg_override_en, rep_override_en, trap_flag, int8_asap, scratch_uchar, io_hi_lo, *vid_mem_base,
    spkr_en, bios_table_lookup[20][256], currKey, keys[3];
unsigned short *regs16, reg_ip, seg_override, file_index, wave_counter;
unsigned int op_source, op_dest, rm_addr, op_to_addr, op_from_addr, i_data0, i_data1, i_data2,
    scratch_uint, scratch2_uint, inst_counter, set_flags_type, GRAPHICS_X, GRAPHICS_Y,
    pixel_colors[16], vmem_ctr;
int op_result, disk[3], scratch_int, key;
time_t clock_buf;
struct timeb ms_clock;


// Helper functions

// Set carry flag
char set_CF(int new_CF) { return regs8[FLAG_CF] = !!new_CF; }

// Set auxiliary flag
char set_AF(int new_AF) { return regs8[FLAG_AF] = !!new_AF; }

// Set overflow flag
char set_OF(int new_OF) { return regs8[FLAG_OF] = !!new_OF; }

// Set auxiliary and overflow flag after arithmetic operations
char set_AF_OF_arith() {
    set_AF((op_source ^= op_dest ^ op_result) & 0x10);
    if (op_result == op_dest)
        return set_OF(0);
    else
        return set_OF(1 & (regs8[FLAG_CF] ^ op_source >> (TOP_BIT - 1)));
}

// Assemble and return emulated CPU FLAGS register in scratch_uint
void make_flags() {
    scratch_uint = 0xF002; // 8086 has reserved and unused flags set to 1
    for (int i = 9; i--;)
        scratch_uint += regs8[FLAG_CF + i] << bios_table_lookup[TABLE_FLAGS_BITFIELDS][i];
}

// Set emulated CPU FLAGS register from regs8[FLAG_xx] values
void set_flags(int new_flags) {
    for (int i = 9; i--;)
        regs8[FLAG_CF + i] = !!(1 << bios_table_lookup[TABLE_FLAGS_BITFIELDS][i] & new_flags);
}

// Convert raw opcode to translated opcode index. This condenses a large number of different
// encodings of similar
// instructions into a much smaller number of distinct functions, which we then execute
void set_opcode(unsigned char opcode) {
    xlat_opcode_id = bios_table_lookup[TABLE_XLAT_OPCODE][raw_opcode_id = opcode];
    extra = bios_table_lookup[TABLE_XLAT_SUBFUNCTION][opcode];
    i_mod_size = bios_table_lookup[TABLE_I_MOD_SIZE][opcode];
    set_flags_type = bios_table_lookup[TABLE_STD_FLAGS][opcode];
}

// Execute INT #interrupt_num on the emulated machine
char pc_interrupt(unsigned char interrupt_num) {
    set_opcode(0xCD); // Decode like INT

    make_flags();
    R_M_PUSH(scratch_uint);
    R_M_PUSH(regs16[REG_CS]);
    R_M_PUSH(reg_ip);
    MEM_OP(REGS_BASE + 2 * REG_CS, =, 4 * interrupt_num + 2);
    R_M_OP(reg_ip, =, mem[4 * interrupt_num]);

    return regs8[FLAG_TF] = regs8[FLAG_IF] = 0;
}

// AAA and AAS instructions - which_operation is +1 for AAA, and -1 for AAS
int AAA_AAS(char which_operation) {
    return (regs16[REG_AX] +=
            262 * which_operation * set_AF(set_CF(((regs8[REG_AL] & 0x0F) > 9) || regs8[FLAG_AF])),
            regs8[REG_AL] &= 0x0F);
}


int main(int argc, char **argv) {
    //*++argv;
    // printf(argv[0]);
    //*++argv;
    // printf(argv[1]);
    //*++argv;
    // printf(argv[2]);
    char **args[2];
    args[1] = "bios";
    args[2] = "fd.img";
    main2(2, args);
}

// Emulator entry point
int main2(int argc, char **argv) {

    currKey = 0x20;

    gfxInitDefault();

    PrintConsole topScreen, bottomScreen;

    // Initialize console for both screen using the two different PrintConsole we have defined
    consoleInit(GFX_TOP, &topScreen);
    consoleInit(GFX_BOTTOM, &bottomScreen);

    printf("\x1b[0;0HABCDEFGHIJKLMNOPQRSTUVWXYZ");

    printf("\x1b[1;0HKEYBOARD!");

    consoleSelect(&topScreen);

    // Flush and swap framebuffers
    gfxFlushBuffers();
    gfxSwapBuffers();

    // Wait for VBlank
    gspWaitForVBlank();

    // regs16 and reg8 point to F000:0, the start of memory-mapped registers. CS is initialised to
    // F000
    regs16 = (unsigned short *)(regs8 = mem + REGS_BASE);
    regs16[REG_CS] = 0xF000;

    // Trap flag off
    regs8[FLAG_TF] = 0;

    // Set DL equal to the boot device: 0 for the FD, or 0x80 for the HD. Normally, boot from the
    // FD.
    // But, if the HD image file is prefixed with @, then boot from the HD
    regs8[REG_DL] = ((argc > 3) && (*argv[3] == '@')) ? argv[3]++, 0x80 : 0;

    // Open BIOS (file id disk[2]), floppy disk image (disk[1]), and hard disk image (disk[0]) if
    // specified
    disk[2] = open("bios", 32898);
    disk[1] = open("fd.img", 32898);

    // Set CX:AX equal to the hard disk image size, if present
    CAST(unsigned)regs16[REG_AX] = *disk ? lseek(*disk, 0, 2) >> 9 : 0;

    // Load BIOS image into F000:0100, and set IP to 0100
    read(disk[2], regs8 + (reg_ip = 0x100), 0xFF00);

    // Load instruction decoding helper table
    for (int i = 0; i < 20; i++)
        for (int j = 0; j < 256; j++)
            bios_table_lookup[i][j] = regs8[regs16[0x81 + i] + j];

    // Instruction execution loop. Terminates if CS:IP = 0:0
    for (; opcode_stream = mem + 16 * regs16[REG_CS] + reg_ip, opcode_stream != mem;) {

        // Scan all the inputs. This should be done once for each frame
        hidScanInput();

        // hidKeysDown returns information about which buttons have been just pressed (and they
        // weren't in the previous frame)
        u32 held = hidKeysDown();

        touchPosition touch;

        // Read the touch screen coordinates
        hidTouchRead(&touch);

        // Set up variables to prepare for decoding an opcode
        set_opcode(*opcode_stream);

        // Extract i_w and i_d fields from instruction
        i_w = (i_reg4bit = raw_opcode_id & 7) & 1;
        i_d = i_reg4bit / 2 & 1;

        // Extract instruction data fields
        i_data0 = CAST(short)opcode_stream[1];
        i_data1 = CAST(short)opcode_stream[2];
        i_data2 = CAST(short)opcode_stream[3];

        // seg_override_en and rep_override_en contain number of instructions to hold segment
        // override and REP prefix respectively
        if (seg_override_en)
            seg_override_en--;
        if (rep_override_en)
            rep_override_en--;

        // i_mod_size > 0 indicates that opcode uses i_mod/i_rm/i_reg, so decode them
        if (i_mod_size) {
            i_mod = (i_data0 & 0xFF) >> 6;
            i_rm = i_data0 & 7;
            i_reg = i_data0 / 8 & 7;

            if ((!i_mod && i_rm == 6) || (i_mod == 2))
                i_data2 = CAST(short)opcode_stream[4];
            else if (i_mod != 1)
                i_data2 = i_data1;
            else // If i_mod is 1, operand is (usually) 8 bits rather than 16 bits
                i_data1 = (char)i_data1;

            DECODE_RM_REG;
        }

        // Instruction execution unit
        switch (xlat_opcode_id) {
        case 0: // Conditional jump (JAE, JNAE, etc.)
                // i_w is the invert flag, e.g. i_w == 1 means JNAE, whereas i_w == 0 means JAE
            scratch_uchar = raw_opcode_id / 2 & 7;
            reg_ip +=
                (char)i_data0 *
                (i_w ^ (regs8[bios_table_lookup[TABLE_COND_JUMP_DECODE_A][scratch_uchar]] ||
                        regs8[bios_table_lookup[TABLE_COND_JUMP_DECODE_B][scratch_uchar]] ||
                        regs8[bios_table_lookup[TABLE_COND_JUMP_DECODE_C][scratch_uchar]] ^
                            regs8[bios_table_lookup[TABLE_COND_JUMP_DECODE_D][scratch_uchar]]));
            break;
        case 1: // MOV reg, imm
            i_w = !!(raw_opcode_id & 8);
            R_M_OP(mem[GET_REG_ADDR(i_reg4bit)], =, i_data0);
            break;
        case 3: // PUSH regs16
            R_M_PUSH(regs16[i_reg4bit]);
            break;
        case 4: // POP regs16
            R_M_POP(regs16[i_reg4bit]);
            break;
        case 2: // INC|DEC regs16
            i_w = 1;
            i_d = 0;
            i_reg = i_reg4bit;
            DECODE_RM_REG;
            i_reg = extra;
        case 5:            // INC|DEC|JMP|CALL|PUSH
            if (i_reg < 2) // INC|DEC
                MEM_OP(op_from_addr, += 1 - 2 * i_reg +, REGS_BASE + 2 * REG_ZERO),
                    op_source = 1, set_AF_OF_arith(),
                    set_OF(op_dest + 1 - i_reg == 1 << (TOP_BIT - 1)),
                    (xlat_opcode_id == 5) && (set_opcode(0x10), 0); // Decode like ADC
            else if (i_reg != 6)                                    // JMP|CALL
                i_reg - 3 || R_M_PUSH(regs16[REG_CS]),              // CALL (far)
                    i_reg & 2 && R_M_PUSH(reg_ip + 2 + i_mod * (i_mod != 3) +
                                          2 * (!i_mod && i_rm == 6)), // CALL (near or far)
                    i_reg & 1 &&
                        (regs16[REG_CS] = CAST(short)mem[op_from_addr + 2]), // JMP|CALL (far)
                    R_M_OP(reg_ip, =, mem[op_from_addr]),
                    set_opcode(0x9A); // Decode like CALL
            else                      // PUSH
                R_M_PUSH(mem[rm_addr]);
            break;
        case 6: // TEST r/m, imm16 / NOT|NEG|MUL|IMUL|DIV|IDIV reg
            op_to_addr = op_from_addr;
            switch (i_reg) {
            case 0:               // TEST
                set_opcode(0x20); // Decode like AND
                reg_ip += i_w + 1;
                R_M_OP(mem[op_to_addr], &, i_data2);
                break;
            case 2: // NOT
                OP(= ~);
                break;
            case 3: // NEG
                OP(= -);
                op_dest = 0;
                set_opcode(0x28); // Decode like SUB
                set_CF(op_result > op_dest);
                break;
            case 4: // MUL
                i_w ? MUL_MACRO(unsigned short, regs16) : MUL_MACRO(unsigned char, regs8);
                break;
            case 5: // IMUL
                i_w ? MUL_MACRO(short, regs16) : MUL_MACRO(char, regs8);
                break;
            case 6: // DIV
                i_w ? DIV_MACRO(unsigned short, unsigned, regs16)
                    : DIV_MACRO(unsigned char, unsigned short, regs8);
                break;
            case 7: // IDIV
                i_w ? DIV_MACRO(short, int, regs16) : DIV_MACRO(char, short, regs8);
            }
            break;
        case 7: // ADD|OR|ADC|SBB|AND|SUB|XOR|CMP AL/AX, immed
            rm_addr = REGS_BASE;
            i_data2 = i_data0;
            i_mod = 3;
            i_reg = extra;
            reg_ip--;
        case 8: // ADD|OR|ADC|SBB|AND|SUB|XOR|CMP reg, immed
            op_to_addr = rm_addr;
            regs16[REG_SCRATCH] = (i_d |= !i_w) ? (char)i_data2 : i_data2;
            op_from_addr = REGS_BASE + 2 * REG_SCRATCH;
            reg_ip += !i_d + 1;
            set_opcode(0x08 *(extra = i_reg));
        case 9: // ADD|OR|ADC|SBB|AND|SUB|XOR|CMP|MOV reg, r/m
            switch (extra) {
            case 0: // ADD
                OP(+= ), set_CF(op_result < op_dest);
                break;
            case 1: // OR
                OP(|= );
                break;
            case 2: // ADC
                ADC_SBB_MACRO(+);
                break;
            case 3: // SBB
                ADC_SBB_MACRO(-);
                break;
            case 4: // AND
                OP(&= );
                break;
            case 5: // SUB
                OP(-= ), set_CF(op_result > op_dest);
                break;
            case 6: // XOR
                OP(^= );
                break;
            case 7: // CMP
                OP(-), set_CF(op_result > op_dest);
                break;
            case 8: // MOV
                OP(= );
            }
            break;
        case 10:      // MOV sreg, r/m | POP r/m | LEA reg, r/m
            if (!i_w) // MOV
                i_w = 1, i_reg += 8, DECODE_RM_REG, OP(= );
            else if (!i_d) // LEA
                seg_override_en = 1, seg_override = REG_ZERO, DECODE_RM_REG,
                R_M_OP(mem[op_from_addr], =, rm_addr);
            else // POP
                R_M_POP(mem[rm_addr]);
            break;
        case 11: // MOV AL/AX, [loc]
            i_mod = i_reg = 0;
            i_rm = 6;
            i_data1 = i_data0;
            DECODE_RM_REG;
            MEM_OP(op_from_addr, =, op_to_addr);
            break;
        case 12: // ROL|ROR|RCL|RCR|SHL|SHR|???|SAR reg/mem, 1/CL/imm (80186)
            scratch2_uint = SIGN_OF(mem[rm_addr]),
            scratch_uint = extra ? // xxx reg/mem, imm
                ++reg_ip,
            (char)i_data1 : i_d ? 31 & regs8[REG_CL] : 1; // xxx reg/mem, CL or xxx reg/mem, 1
            if (scratch_uint) {
                if (i_reg < 4) // Rotate operations
                    scratch_uint %= i_reg / 2 + TOP_BIT, R_M_OP(scratch2_uint, =, mem[rm_addr]);
                if (i_reg & 1) // Rotate/shift right operations
                    R_M_OP(mem[rm_addr], >>=, scratch_uint);
                else // Rotate/shift left operations
                    R_M_OP(mem[rm_addr], <<=, scratch_uint);
                if (i_reg > 3)        // Shift operations
                    set_opcode(0x10); // Decode like ADC
                if (i_reg > 4)        // SHR or SAR
                    set_CF(op_dest >> (scratch_uint - 1) & 1);
            }

            switch (i_reg) {
            case 0: // ROL
                R_M_OP(mem[rm_addr], +=, scratch2_uint >> (TOP_BIT - scratch_uint));
                set_OF(SIGN_OF(op_result) ^ set_CF(op_result & 1));
                break;
            case 1: // ROR
                scratch2_uint &= (1 << scratch_uint) - 1,
                    R_M_OP(mem[rm_addr], +=, scratch2_uint << (TOP_BIT - scratch_uint));
                set_OF(SIGN_OF(op_result * 2) ^ set_CF(SIGN_OF(op_result)));
                break;
            case 2: // RCL
                R_M_OP(mem[rm_addr], += (regs8[FLAG_CF] << (scratch_uint - 1)) +,
                       scratch2_uint >> (1 + TOP_BIT - scratch_uint));
                set_OF(SIGN_OF(op_result) ^ set_CF(scratch2_uint & 1 << (TOP_BIT - scratch_uint)));
                break;
            case 3: // RCR
                R_M_OP(mem[rm_addr], += (regs8[FLAG_CF] << (TOP_BIT - scratch_uint)) +,
                       scratch2_uint << (1 + TOP_BIT - scratch_uint));
                set_CF(scratch2_uint & 1 << (scratch_uint - 1));
                set_OF(SIGN_OF(op_result) ^ SIGN_OF(op_result * 2));
                break;
            case 4: // SHL
                set_OF(SIGN_OF(op_result) ^ set_CF(SIGN_OF(op_dest << (scratch_uint - 1))));
                break;
            case 5: // SHR
                set_OF(SIGN_OF(op_dest));
                break;
            case 7: // SAR
                scratch_uint < TOP_BIT || set_CF(scratch2_uint);
                set_OF(0);
                R_M_OP(mem[rm_addr], +=, scratch2_uint *= ~(((1 << TOP_BIT) - 1) >> scratch_uint));
            }
            break;
        case 13: // LOOPxx|JCZX
            scratch_uint = !!--regs16[REG_CX];

            switch (i_reg4bit) {
            case 0: // LOOPNZ
                scratch_uint &= !regs8[FLAG_ZF];
                break;
            case 1: // LOOPZ
                scratch_uint &= regs8[FLAG_ZF];
                break;
            case 3: // JCXXZ
                scratch_uint = !++regs16[REG_CX];
            }
            reg_ip += scratch_uint * (char)i_data0;
            break;
        case 14: // JMP | CALL short/near
            reg_ip += 3 - i_d;
            if (!i_w) {
                if (i_d) // JMP far
                    reg_ip = 0, regs16[REG_CS] = i_data2;
                else // CALL
                    R_M_PUSH(reg_ip);
            }
            reg_ip += i_d && i_w ? (char)i_data0 : i_data0;
            break;
        case 15: // TEST reg, r/m
            MEM_OP(op_from_addr, &, op_to_addr);
            break;
        case 16: // XCHG AX, regs16
            i_w = 1;
            op_to_addr = REGS_BASE;
            op_from_addr = GET_REG_ADDR(i_reg4bit);
        case 24: // NOP|XCHG reg, r/m
            if (op_to_addr != op_from_addr)
                OP(^= ), MEM_OP(op_from_addr, ^=, op_to_addr), OP(^= );
            break;
        case 17: // MOVSx (extra=0)|STOSx (extra=1)|LODSx (extra=2)
            scratch2_uint = seg_override_en ? seg_override : REG_DS;

            for (scratch_uint = rep_override_en ? regs16[REG_CX] : 1; scratch_uint;
                 scratch_uint--) {
                MEM_OP(extra < 2 ? SEGREG(REG_ES, REG_DI, ) : REGS_BASE, =,
                       extra & 1 ? REGS_BASE : SEGREG(scratch2_uint, REG_SI, )),
                    extra & 1 || INDEX_INC(REG_SI), extra & 2 || INDEX_INC(REG_DI);
            }

            if (rep_override_en)
                regs16[REG_CX] = 0;
            break;
        case 18: // CMPSx (extra=0)|SCASx (extra=1)
            scratch2_uint = seg_override_en ? seg_override : REG_DS;

            if ((scratch_uint = rep_override_en ? regs16[REG_CX] : 1)) {
                for (; scratch_uint; rep_override_en || scratch_uint--) {
                    MEM_OP(extra ? REGS_BASE : SEGREG(scratch2_uint, REG_SI, ), -,
                           SEGREG(REG_ES, REG_DI, )),
                        extra || INDEX_INC(REG_SI), INDEX_INC(REG_DI),
                        rep_override_en && !(--regs16[REG_CX] && (!op_result == rep_mode)) &&
                            (scratch_uint = 0);
                }

                set_flags_type =
                    FLAGS_UPDATE_SZP | FLAGS_UPDATE_AO_ARITH; // Funge to set SZP/AO flags
                set_CF(op_result > op_dest);
            }
            break;
        case 19: // RET|RETF|IRET
            i_d = i_w;
            R_M_POP(reg_ip);
            if (extra) // IRET|RETF|RETF imm16
                R_M_POP(regs16[REG_CS]);
            if (extra & 2) // IRET
                set_flags(R_M_POP(scratch_uint));
            else if (!i_d) // RET|RETF imm16
                regs16[REG_SP] += i_data0;
            break;
        case 20: // MOV r/m, immed
            R_M_OP(mem[op_from_addr], =, i_data2);
            break;
        case 21:                               // IN AL/AX, DX/imm8
            io_ports[0x20] = 0;                // PIC EOI
            io_ports[0x42] = --io_ports[0x40]; // PIT channel 0/2 read placeholder
            io_ports[0x3DA] ^= 9;              // CGA refresh
            scratch_uint = extra ? regs16[REG_DX] : (unsigned char)i_data0;
            scratch_uint == 0x60 && (io_ports[0x64] = 0); // Scancode read flag
            scratch_uint == 0x3D5 && (io_ports[0x3D4] >> 1 == 7) &&
                (io_ports[0x3D5] = ((mem[0x49E] * 80 + mem[0x49D] + CAST(short)mem[0x4AD]) &
                                    (io_ports[0x3D4] & 1 ? 0xFF : 0xFF00)) >>
                                   (io_ports[0x3D4] & 1 ? 0 : 8)); // CRT cursor position
            R_M_OP(regs8[REG_AL], =, io_ports[scratch_uint]);
            break;
        case 22: // OUT DX/imm8, AL/AX
            scratch_uint = extra ? regs16[REG_DX] : (unsigned char)i_data0;
            R_M_OP(io_ports[scratch_uint], =, regs8[REG_AL]);
            scratch_uint == 0x61 && (io_hi_lo = 0, spkr_en |= regs8[REG_AL] & 3); // Speaker control
            (scratch_uint == 0x40 || scratch_uint == 0x42) && (io_ports[0x43] & 6) &&
                (mem[0x469 + scratch_uint - (io_hi_lo ^= 1)] =
                     regs8[REG_AL]); // PIT rate programming

            scratch_uint == 0x3D5 && (io_ports[0x3D4] >> 1 == 6) &&
                (mem[0x4AD + !(io_ports[0x3D4] & 1)] = regs8[REG_AL]); // CRT video RAM start offset
            scratch_uint == 0x3D5 && (io_ports[0x3D4] >> 1 == 7) &&
                (scratch2_uint = ((mem[0x49E] * 80 + mem[0x49D] + CAST(short)mem[0x4AD]) &
                                  (io_ports[0x3D4] & 1 ? 0xFF00 : 0xFF)) +
                                 (regs8[REG_AL] << (io_ports[0x3D4] & 1 ? 0 : 8)) -
                                 CAST(short)mem[0x4AD],
                 mem[0x49D] = scratch2_uint % 80,
                 mem[0x49E] = scratch2_uint / 80); // CRT cursor position
            scratch_uint == 0x3B5 && io_ports[0x3B4] == 1 &&
                (GRAPHICS_X = regs8[REG_AL] * 16); // Hercules resolution reprogramming. Defaults
                                                   // are set in the BIOS
            scratch_uint == 0x3B5 && io_ports[0x3B4] == 6 && (GRAPHICS_Y = regs8[REG_AL] * 4);
            break;
        case 23: // REPxx
            rep_override_en = 2;
            rep_mode = i_w;
            seg_override_en &&seg_override_en++;
            break;
        case 25: // PUSH reg
            R_M_PUSH(regs16[extra]);
            break;
        case 26: // POP reg
            R_M_POP(regs16[extra]);
            break;
        case 27: // xS: segment overrides
            seg_override_en = 2;
            seg_override = extra;
            rep_override_en &&rep_override_en++;
            break;
        case 28: // DAA/DAS
            i_w = 0;
            extra ? DAA_DAS(-=, >=, 0xFF, 0x99)
                  : DAA_DAS(+=, <, 0xF0, 0x90); // extra = 0 for DAA, 1 for DAS
            break;
        case 29: // AAA/AAS
            op_result = AAA_AAS(extra - 1);
            break;
        case 30: // CBW
            regs8[REG_AH] = -SIGN_OF(regs8[REG_AL]);
            break;
        case 31: // CWD
            regs16[REG_DX] = -SIGN_OF(regs16[REG_AX]);
            break;
        case 32: // CALL FAR imm16:imm16
            R_M_PUSH(regs16[REG_CS]);
            R_M_PUSH(reg_ip + 5);
            regs16[REG_CS] = i_data2;
            reg_ip = i_data0;
            break;
        case 33: // PUSHF
            make_flags();
            R_M_PUSH(scratch_uint);
            break;
        case 34: // POPF
            set_flags(R_M_POP(scratch_uint));
            break;
        case 35: // SAHF
            make_flags();
            set_flags((scratch_uint & 0xFF00) + regs8[REG_AH]);
            break;
        case 36: // LAHF
            make_flags(), regs8[REG_AH] = scratch_uint;
            break;
        case 37: // LES|LDS reg, r/m
            i_w = i_d = 1;
            DECODE_RM_REG;
            OP(= );
            MEM_OP(REGS_BASE + extra, =, rm_addr + 2);
            break;
        case 38: // INT 3
            ++reg_ip;
            pc_interrupt(3);
            break;
        case 39: // INT imm8
            reg_ip += 2;
            pc_interrupt(i_data0);
            break;
        case 40: // INTO
            ++reg_ip;
            regs8[FLAG_OF] && pc_interrupt(4);
            break;
        case 41: // AAM
            if (i_data0 &= 0xFF)
                regs8[REG_AH] = regs8[REG_AL] / i_data0, op_result = regs8[REG_AL] %= i_data0;
            else // Divide by zero
                pc_interrupt(0);
            break;
        case 42: // AAD
            i_w = 0;
            regs16[REG_AX] = op_result = 0xFF & regs8[REG_AL] + i_data0 * regs8[REG_AH];
            break;
        case 43: // SALC
            regs8[REG_AL] = -regs8[FLAG_CF];
            break;
        case 44: // XLAT
            regs8[REG_AL] =
                mem[SEGREG(seg_override_en ? seg_override : REG_DS, REG_BX, regs8[REG_AL] + )];
            break;
        case 45: // CMC
            regs8[FLAG_CF] ^= 1;
            break;
        case 46: // CLC|STC|CLI|STI|CLD|STD
            regs8[extra / 2] = extra & 1;
            break;
        case 47: // TEST AL/AX, immed
            R_M_OP(regs8[REG_AL], &, i_data0);
            break;
        case 48: // Emulator-specific 0F xx opcodes
            switch ((char)i_data0) {
            case 0: // PUTCHAR_AL
                write(1, regs8, 1);
                mem[0x4A6] = 0;
                break;
            case 1: // GET_RTC
                time(&clock_buf);
                short ms_clock2 = 1000 * clock_buf;
                memcpy(mem + SEGREG(REG_ES, REG_BX, ), localtime(&clock_buf), sizeof(struct tm));
                CAST(short)mem[SEGREG(REG_ES, REG_BX, 36 + )] = ms_clock2;
                break;
            case 2: // DISK_READ
            case 3: // DISK_WRITE
                regs8[REG_AL] =
                    ~lseek(disk[regs8[REG_DL]], CAST(unsigned)regs16[REG_BP] << 9, 0)
                        ? ((char)i_data0 == 3 ? (int (*)())write : (int (*)())read)(
                              disk[regs8[REG_DL]], mem + SEGREG(REG_ES, REG_BX, ), regs16[REG_AX])
                        : 0;
            }
        }

        // Increment instruction pointer by computed instruction length. Tables in the BIOS binary
        // help us here.
        reg_ip += (i_mod * (i_mod != 3) + 2 * (!i_mod && i_rm == 6)) * i_mod_size +
                  bios_table_lookup[TABLE_BASE_INST_SIZE][raw_opcode_id] +
                  bios_table_lookup[TABLE_I_W_SIZE][raw_opcode_id] * (i_w + 1);

        // If instruction needs to update SF, ZF and PF, set them as appropriate
        if (set_flags_type & FLAGS_UPDATE_SZP) {
            regs8[FLAG_SF] = SIGN_OF(op_result);
            regs8[FLAG_ZF] = !op_result;
            regs8[FLAG_PF] = bios_table_lookup[TABLE_PARITY_FLAG][(unsigned char)op_result];

            // If instruction is an arithmetic or logic operation, also set AF/OF/CF as appropriate.
            if (set_flags_type & FLAGS_UPDATE_AO_ARITH)
                set_AF_OF_arith();
            if (set_flags_type & FLAGS_UPDATE_OC_LOGIC)
                set_CF(0), set_OF(0);
        }

        // Poll timer/keyboard every KEYBOARD_TIMER_UPDATE_DELAY instructions
        if (!(++inst_counter % KEYBOARD_TIMER_UPDATE_DELAY))
            int8_asap = 1;


        /* if( held & KEY_UP){
        mem[0x4A6] = 0x64;
        for(int i = 300; i != 0; i--){

        }
        } */

        /* if( held & KEY_RIGHT ){
            if(currKey == 0x20){
                currKey = 0x7F;
            }
            else{
            currKey++;
            }

            if(currKey == 0x20){
            printf("\x1b[25;50HSpace");
            } else if(currKey == 0x7F){
            printf("\x1b[27;50HDEL");
            } else {

            printf("%c", (currKey - '0'));
            }
        } */

        /* 			if( held & KEY_DOWN){
                    mem[0x4A6] = 0x72;
                    for(int i = 300; i != 0; i--){

                    }
                    } */
        /*
        if( held & KEY_LEFT){
            if(currKey == 0x20){
                currKey = 0x7F;
            }
            else{
            currKey--;
            }

            if(currKey == 0x20){
            printf("\x1b[25;50HSpace");
            } else if(currKey == 0x7F){
            printf("\x1b[27;50HDEL");
            } else {

            printf("%c", (currKey - '0'));
            }
        } */

        // if( held & KEY_A){
        // mem[0x4A6] = currKey;
        // gspWaitForVBlank();
        //}

        if (held & KEY_START) {
            mem[0x4A6] = 13;
            // gspWaitForVBlank();
        }

        if (held & KEY_SELECT) {
            mem[0x4A6] = 8;
            // gspWaitForVBlank();
        }

        hidTouchRead(&touch);

        key = touch.px - '0';

        if (held & KEY_B) {
            printf("%d", key);
        }

        if (key != -48) {
            //	printf("Z");
        }

        if (key <= (signed int)161 & key >= (signed int)153) {
            mem[0x4A6] = 0x7a;
        }
        if (key <= (signed int)153 & key >= (signed int)150) {
            mem[0x4A6] = 0x79;
        }
        if (key <= (signed int)150 & key >= (signed int)145) {
            mem[0x4A6] = 0x78;
        }
        if (key <= (signed int)145 & key >= (signed int)137) {
            mem[0x4A6] = 0x77;
        }
        if (key <= (signed int)137 & key >= (signed int)129) {
            mem[0x4A6] = 0x76;
        }
        if (key <= (signed int)129 & key >= (signed int)121) {
            mem[0x4A6] = 0x75;
        }
        if (key <= (signed int)121 & key >= (signed int)113) {
            mem[0x4A6] = 0x74;
        }
        if (key <= (signed int)113 & key >= (signed int)105) {
            mem[0x4A6] = 0x73;
        }
        if (key <= (signed int)105 & key >= (signed int)97) {
            mem[0x4A6] = 0x72;
        }
        if (key <= (signed int)97 & key >= (signed int)89) {
            mem[0x4A6] = 0x71;
        }
        if (key <= (signed int)89 & key >= (signed int)81) {
            mem[0x4A6] = 0x70;
        }
        if (key <= (signed int)81 & key >= (signed int)73) {
            mem[0x4A6] = 0x6f;
            ;
        }
        if (key <= (signed int)73 & key >= (signed int)65) {
            mem[0x4A6] = 0x6e;
        }
        if (key <= (signed int)65 & key >= (signed int)57) {
            mem[0x4A6] = 0x6d;
        }
        if (key <= (signed int)57 & key >= (signed int)49) {
            mem[0x4A6] = 0x6c;
        }
        if (key <= (signed int)49 & key >= (signed int)41) {
            mem[0x4A6] = 0x6b;
        }
        if (key <= (signed int)41 & key >= (signed int)33) {
            mem[0x4A6] = 0x6a;
        }
        if (key <= (signed int)33 & key >= (signed int)25) {
            mem[0x4A6] = 0x69;
        }
        if (key <= (signed int)25 & key >= (signed int)17) {
            mem[0x4A6] = 0x68;
        }
        if (key <= (signed int)17 & key >= (signed int)9) {
            mem[0x4A6] = 0x67;
        }
        if (key <= (signed int)9 & key >= (signed int)1) {
            mem[0x4A6] = 0x66;
        }
        if (key <= (signed int)1 & key >= (signed int)-7) {
            mem[0x4A6] = 0x65;
        }
        if (key <= (signed int)-7 & key >= (signed int)-15) {
            mem[0x4A6] = 0x64;
        }
        if (key <= (signed int)-15 & key >= (signed int)-23) {
            mem[0x4A6] = 0x63;
        }
        if (key <= (signed int)-23 & key >= (signed int)-31) {
            mem[0x4A6] = 0x62;
        }
        if (key <= (signed int)-31 & key >= (signed int)-39) {
            mem[0x4A6] = 0x61;
        }

        /* 			if( held & KEY_START){
                    break;
                    break;
                    break;
                    break;
                    break;
                    break;
                    break;
                    break;
                    break;
                    break;
                    break;
                    break;
                    } */

        // Application has set trap flag, so fire INT 1
        if (trap_flag)
            pc_interrupt(1);

        trap_flag = regs8[FLAG_TF];

        // If a timer tick is pending, interrupts are enabled, and no overrides/REP are active,
        // then process the tick and check for new keystrokes
        if (int8_asap && !seg_override_en && !rep_override_en && regs8[FLAG_IF] && !regs8[FLAG_TF])
            pc_interrupt(0xA), int8_asap = 0, SDL_KEYBOARD_DRIVER;
    }

    return 0;
}
