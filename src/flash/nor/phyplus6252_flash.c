/*
 * OpenOCD flash driver for PHYPLUS6252 internal SPI NOR flash (AP_SPIF controller)
 * Skeleton version - for research and bring-up
 *
 * Copyright (c) 2024
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define ROM_BASED_WRITE 1

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/target.h>

#define PHYPLUS6252_FLASH_BASE      0x11000000
#define PHYPLUS6252_SPIF_BASE       0x4000C800

// AP_SPIF register offsets (from mcu_phy_bumbee.h)
#define SPIF_CONFIG             0x00  // QSPI Configuration Register
#define SPIF_READ_INSTR         0x04  // Device Read Instruction Register
#define SPIF_WRITE_INSTR        0x08  // Device Write Instruction Register
#define SPIF_DELAY              0x0C  // QSPI Device Delay Register
#define SPIF_RDDATA_CAPTURE     0x10  // Read Data Capture Register
#define SPIF_DEV_SIZE           0x14  // Device Size Register
#define SPIF_SRAM_PART          0x18  // SRAM Partition Register
#define SPIF_IND_AHB_ADDR_TRIG  0x1C  // Indirect AHB Address Trigger Register
#define SPIF_DMA_PERIPHERAL     0x20  // DMA Peripheral Register
#define SPIF_REMAP              0x24  // Remap Address Register
#define SPIF_MODE_BIT           0x28  // Mode Bit Register
#define SPIF_SRAM_FILL_LEVEL    0x2C  // SRAM Fill Level Register
#define SPIF_TX_THRESHOLD       0x30  // TX Threshold Register
#define SPIF_RX_THRESHOLD       0x34  // RX Threshold Register
#define SPIF_WR_COMPLETION      0x38  // Write Completion Control Register
#define SPIF_POLL_EXPIRE        0x3C  // Polling Expiration Register
#define SPIF_INT_STATUS         0x40  // Interrupt Status Register
#define SPIF_INT_MASK           0x44  // Interrupt Mask Register
// 0x48~0x4C: Reserved
#define SPIF_LOW_WR_PROTECTION  0x50  // Lower Write Protection Register
#define SPIF_UP_WR_PROTECTION   0x54  // Upper Write Protection Register
#define SPIF_WR_PROTECTION      0x58  // Write Protection Register
// 0x5C: Reserved
#define SPIF_INDIRECT_RD        0x60  // Indirect Read Transfer Register
#define SPIF_INDIRECT_RD_WM     0x64  // Indirect Read Transfer Watermark Register
#define SPIF_INDIRECT_RD_START  0x68  // Indirect Read Transfer Start Address Register
#define SPIF_INDIRECT_RD_NUM    0x6C  // Indirect Read Transfer Number Bytes Register
#define SPIF_INDIRECT_WR        0x70  // Indirect Write Transfer Register
#define SPIF_INDIRECT_WR_WM     0x74  // Indirect Write Transfer Watermark Register
#define SPIF_INDIRECT_WR_START  0x78  // Indirect Write Transfer Start Address Register
#define SPIF_INDIRECT_WR_CNT    0x7C  // Indirect Write Transfer Count Register
#define SPIF_IND_AHB_TRIG_RANGE 0x80  // Indirect AHB Trigger Address Range Register
// 0x84~0x8C: Reserved
#define SPIF_FCMD               0x90  // Flash Command Register
#define SPIF_FCMD_ADDR          0x94  // Flash Command Address Register
// 0x98~0x9C: Reserved
#define SPIF_FCMD_RDDATA0       0xA0  // Flash Command Read Data Register (low)
#define SPIF_FCMD_RDDATA1       0xA4  // Flash Command Read Data Register (up)
#define SPIF_FCMD_WRDATA0       0xA8  // Flash Command Write Data Register (low)
#define SPIF_FCMD_WRDATA1       0xAC  // Flash Command Write Data Register (up)
#define SPIF_POLL_FSTATUS       0xB0  // Polling Flash Status Register
// 0xFC: Module ID Register (not usually needed)

// JEDEC commands (from flash.h)
#define FCMD_WREN           0x06
#define FCMD_PP             0x02
#define FCMD_SE             0x20
#define FCMD_CE             0x60
#define FCMD_RDID           0x9F
#define FCMD_READ           0x03
#define FCMD_RDSR           0x05

#define JEDEC_ID_CMD        0x9F

#define EXPECTED_MANUFACTURER_ID 0x01

// Helper macros for register access
#define SPIF_REG(target, reg)   (PHYPLUS6252_SPIF_BASE + (reg))

static int phyplus6252_write_reg(struct target *target, uint32_t reg, uint32_t value)
{
    return target_write_u32(target, SPIF_REG(target, reg), value);
}

static int phyplus6252_read_reg(struct target *target, uint32_t reg, uint32_t *value)
{
    return target_read_u32(target, SPIF_REG(target, reg), value);
}

// Wait for controller to be idle (poll fcmd & config)
static int phyplus6252_wait_idle(struct target *target)
{
    uint32_t val;
    int timeout = 10000;
    do {
        phyplus6252_read_reg(target, SPIF_FCMD, &val);
        if ((val & 0x02) == 0)
            break;
    } while (--timeout);
    if (!timeout)
        return ERROR_FAIL;

    timeout = 10000;
    do {
        phyplus6252_read_reg(target, SPIF_CONFIG, &val);
        if ((val & 0x80000000) == 0)
            break;
    } while (--timeout);
    return timeout ? ERROR_OK : ERROR_FAIL;
}

// Issue a JEDEC command (e.g., WREN, SE, PP)
static int phyplus6252_cmd(struct target *target, uint8_t cmd, uint32_t addr, int addrlen, int wrlen, int rdlen)
{
    int res;
    // Write address if needed
    if (addrlen)
        phyplus6252_write_reg(target, SPIF_FCMD_ADDR, addr);

    // Write command to FCMD register
    uint32_t fcmd = (cmd & 0xFF) | ((addrlen & 0x7) << 8) | ((rdlen & 0xF) << 16) | ((wrlen & 0xF) << 20);
    phyplus6252_write_reg(target, SPIF_FCMD, fcmd);

    res = phyplus6252_wait_idle(target);
    return res;
}

// Read status register (RDSR)
static int phyplus6252_read_status(struct target *target, uint8_t *status)
{
    int res = phyplus6252_cmd(target, FCMD_RDSR, 0, 0, 0, 1);
    if (res != ERROR_OK)
        return res;
    uint32_t val;
    res = phyplus6252_read_reg(target, SPIF_FCMD_RDDATA0, &val);
    if (res != ERROR_OK)
        return res;
    *status = val & 0xFF;
    return ERROR_OK;
}

// Wait for WIP=0
static int phyplus6252_wait_ready(struct target *target)
{
    uint8_t status = 0;
    int timeout = 100000;
    do {
        phyplus6252_read_status(target, &status);
        if ((status & 0x01) == 0)
            return ERROR_OK;
    } while (--timeout);
    return ERROR_FAIL;
}

// Add unlock/lock helpers for write enable
static int phyplus6252_flash_unlock(struct target *target)
{
    // Write Enable
    phyplus6252_cmd(target, FCMD_WREN, 0, 0, 0, 0);
    // Write 0x00 to status register (unprotect)
    phyplus6252_write_reg(target, SPIF_FCMD_WRDATA0, 0x00);
    // Write Status Register command (0x01)
    phyplus6252_cmd(target, 0x01, 0, 0, 1, 0);
    return phyplus6252_wait_ready(target);
}

static int phyplus6252_flash_lock(struct target *target)
{
    // Write Enable
    phyplus6252_cmd(target, FCMD_WREN, 0, 0, 0, 0);
    // Write 0x7C to status register (protect all)
    phyplus6252_write_reg(target, SPIF_FCMD_WRDATA0, 0x7C);
    // Write Status Register command (0x01)
    phyplus6252_cmd(target, 0x01, 0, 0, 1, 0);
    return phyplus6252_wait_ready(target);
}

// Helper to disable write protection
static int phyplus6252_disable_write_protection(struct target *target)
{
    // Write 0 to wr_protection register (offset 0x58)
    return phyplus6252_write_reg(target, SPIF_WR_PROTECTION, 0);
}

// Erase sector (4kB)
static int phyplus6252_erase_sector(struct flash_bank *bank, uint32_t sector)
{
    struct target *target = bank->target;
    uint32_t addr = bank->sectors[sector].offset + bank->base;
    int res;

    // Disable write protection before erase
    res = phyplus6252_disable_write_protection(target);
    if (res != ERROR_OK) return res;

    res = phyplus6252_flash_unlock(target);
    if (res != ERROR_OK) return res;
    phyplus6252_cmd(target, FCMD_WREN, 0, 0, 0, 0);
    phyplus6252_cmd(target, FCMD_SE, addr, 3, 0, 0);
    res = phyplus6252_wait_ready(target);
    phyplus6252_flash_lock(target);
    return res;
}

#if ROM_BASED_WRITE

#define ROM_SPIF_WRITE_ADDR 0x100010B0  // <- Adjust if your disassembly shows a different location
#define ROM_STUB_ADDR       0x1FFF7000
#define ROM_BUFFER_ADDR     0x1FFF7400

static const uint8_t rom_spif_write_stub[] = {
    0x00, 0xB5,             // push {lr}
    0x02, 0x4B,             // ldr r3, [pc, #8]
    0x18, 0x47,             // bx  r3
    0x00, 0xBD,             // pop {pc}
    (ROM_SPIF_WRITE_ADDR & 0xFF), 
    ((ROM_SPIF_WRITE_ADDR >> 8) & 0xFF),
    ((ROM_SPIF_WRITE_ADDR >> 16) & 0xFF),
    ((ROM_SPIF_WRITE_ADDR >> 24) & 0xFF),
};

static int phyplus6252_rom_write(struct flash_bank *bank, uint32_t offset, const uint8_t *buffer, uint32_t length)
{
    struct target *target = bank->target;
    int res;

    // Upload the stub
    res = target_write_buffer(target, ROM_STUB_ADDR, sizeof(rom_spif_write_stub), rom_spif_write_stub);
    if (res != ERROR_OK)
        return res;

    // Upload the write buffer
    res = target_write_buffer(target, ROM_BUFFER_ADDR, length, buffer);
    if (res != ERROR_OK)
        return res;

    // Call the stub with (flash_addr, ram_buf, length)
    uint32_t args[3] = { PHYPLUS6252_FLASH_BASE + offset, ROM_BUFFER_ADDR, length };
    res = target_run_algorithm(target, 0, 3, args, ROM_STUB_ADDR, 0, 5000, NULL);

    if (res != ERROR_OK)
        LOG_ERROR("ROM write failed at 0x%08" PRIx32, offset);

    return res;
}

#else

// Program page (256B)
static int phyplus6252_program_page(struct flash_bank *bank, uint32_t addr, const uint8_t *buf, uint32_t len)
{
    struct target *target = bank->target;
    int res;

    // Disable write protection before programming
    res = phyplus6252_disable_write_protection(target);
    if (res != ERROR_OK) return res;

    res = phyplus6252_flash_unlock(target);
    if (res != ERROR_OK) return res;
    uint32_t offset = 0;
    while (offset < len) {
        uint32_t chunk = (len - offset > 256) ? 256 : (len - offset);
        phyplus6252_cmd(target, FCMD_WREN, 0, 0, 0, 0);
        // Write data to FCMD_WRDATA0 (assumes 32-bit writes)
        for (uint32_t i = 0; i < chunk; i += 4) {
            uint32_t w = buf[offset + i] | (buf[offset + i + 1] << 8) |
                         (buf[offset + i + 2] << 16) | (buf[offset + i + 3] << 24);
            phyplus6252_write_reg(target, SPIF_FCMD_WRDATA0, w);
        }
        phyplus6252_cmd(target, FCMD_PP, addr + offset, 3, chunk, 0);
        phyplus6252_wait_ready(target);
        offset += chunk;
    }
    phyplus6252_flash_lock(target);
    return ERROR_OK;
}

#endif

// Read (for verification)
static int phyplus6252_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
    struct target *target = bank->target;
    // Use memory-mapped read
    return target_read_memory(target, PHYPLUS6252_FLASH_BASE + offset, 1, count, buffer);
}

static int phyplus6252_flash_probe(struct flash_bank *bank)
{
    struct target *target = bank->target;
    uint8_t jedec_id[3] = {0};

    // Write JEDEC ID command to controller
    // (You may need to set up the controller for a read, see SDK for details)
    // Example (pseudo-code, adjust for your controller):
    target_write_u32(target, PHYPLUS6252_SPIF_BASE + SPIF_FCMD, JEDEC_ID_CMD);
    // ...trigger the command, wait for completion...

    // Read back the 3 bytes
    uint32_t val;
    target_read_u32(target, PHYPLUS6252_SPIF_BASE + SPIF_FCMD_RDDATA0, &val);
    jedec_id[0] = val & 0xFF;
    jedec_id[1] = (val >> 8) & 0xFF;
    jedec_id[2] = (val >> 16) & 0xFF;

    LOG_INFO("PHYPLUS6252 JEDEC ID: %02X %02X %02X", jedec_id[0], jedec_id[1], jedec_id[2]);

    // Always set up the bank, even if the ID is unexpected
    bank->size = 0x40000; // 256KB
    bank->num_sectors = bank->size / 0x1000; // 4KB sectors
    bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
    for (unsigned i = 0; i < bank->num_sectors; i++) {
        bank->sectors[i].offset = i * 0x1000;
        bank->sectors[i].size = 0x1000;
        bank->sectors[i].is_erased = -1;
        bank->sectors[i].is_protected = 0;
    }

    if (jedec_id[0] != EXPECTED_MANUFACTURER_ID) {
        LOG_ERROR("Unexpected JEDEC manufacturer ID");
        // return ERROR_FAIL; // <-- comment this out for now
    }

    return ERROR_OK;
}

// Add these wrapper functions:
static int phyplus6252_erase(struct flash_bank *bank, unsigned int first, unsigned int last) {
    for (unsigned int s = first; s <= last; s++) {
        int res = phyplus6252_erase_sector(bank, s);
        if (res != ERROR_OK)
            return res;
    }
    return ERROR_OK;
}

static int phyplus6252_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count) {
#if ROM_BASED_WRITE
    return phyplus6252_rom_write(bank, offset, buffer, count);
#else
    return phyplus6252_program_page(bank, offset, buffer, count);
#endif
}

static int phyplus6252_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last) {
    return ERROR_FAIL;
}

static int phyplus6252_auto_probe(struct flash_bank *bank) {
    // Optionally call your probe function or just return ERROR_OK
    return phyplus6252_flash_probe(bank);
}

#include <helper/command.h> // for command_print

#include <inttypes.h> // for PRIx64

static int phyplus6252_info(struct flash_bank *bank, struct command_invocation *cmd)
{
    command_print(cmd, "PHYPLUS6252 custom driver: base=0x%08" PRIx64 ", size=0x%08" PRIx64,
                  (uint64_t)bank->base, (uint64_t)bank->size);
    return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(phyplus6252_flash_bank_command)
{
    return ERROR_OK;
}

// Update your driver struct:
const struct flash_driver phyplus6252_flash = {
    .name = "phyplus6252",
    .erase = phyplus6252_erase,
    .write = phyplus6252_write,
    .read = phyplus6252_read,
    .probe = phyplus6252_flash_probe,
    .auto_probe = phyplus6252_auto_probe,
    .protect = phyplus6252_protect,
    .info = phyplus6252_info,
    .flash_bank_command = phyplus6252_flash_bank_command,
    .free_driver_priv = default_flash_free_driver_priv,
};
