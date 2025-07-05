/*
 * OpenOCD flash driver for PHYPLUS6252 internal SPI NOR flash (AP_SPIF controller)
 * Simplified version
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define ROM_BASED_WRITE 1

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/target.h>
#include <target/target_type.h>
#include <helper/command.h>
#include <inttypes.h>

#define PHYPLUS6252_FLASH_BASE      0x11000000
#define PHYPLUS6252_SPIF_BASE       0x4000C800
#define SPIF_FCMD                   0x90
#define SPIF_FCMD_ADDR              0x94
#define SPIF_FCMD_RDDATA0           0xA0
#define SPIF_FCMD_WRDATA0           0xA8
#define SPIF_WR_PROTECTION          0x58
#define SPIF_CONFIG                 0x00
#define FCMD_WREN                   0x06
#define FCMD_PP                     0x02
#define FCMD_SE                     0x20
#define FCMD_RDSR                   0x05
#define JEDEC_ID_CMD                0x9F
#define EXPECTED_MANUFACTURER_ID    0x01

#define ROM_SPIF_WRITE_ADDR 0x100010B0
#define ROM_STUB_ADDR       0x1FFF7000
#define ROM_BUFFER_ADDR     0x1FFF7400

static int write_reg(struct target *target, uint32_t reg, uint32_t value) {
    return target_write_u32(target, PHYPLUS6252_SPIF_BASE + reg, value);
}
static int read_reg(struct target *target, uint32_t reg, uint32_t *value) {
    return target_read_u32(target, PHYPLUS6252_SPIF_BASE + reg, value);
}
static int wait_idle(struct target *target) {
    uint32_t status; int timeout = 10000;
    do { read_reg(target, SPIF_FCMD, &status); if (!(status & 0x02)) break; } while (--timeout);
    do { read_reg(target, SPIF_CONFIG, &status); if (!(status & 0x80000000)) break; } while (--timeout);
    return timeout ? ERROR_OK : ERROR_FAIL;
}
static int send_cmd(struct target *target, uint8_t cmd_code, uint32_t addr, int addr_len, int write_len, int read_len) {
    if (addr_len) write_reg(target, SPIF_FCMD_ADDR, addr);
    uint32_t fcmd = (cmd_code & 0xFF) | ((addr_len & 0x7) << 8) | ((read_len & 0xF) << 16) | ((write_len & 0xF) << 20);
    write_reg(target, SPIF_FCMD, fcmd);
    return wait_idle(target);
}
static int read_status(struct target *target, uint8_t *status_out) {
    if (send_cmd(target, FCMD_RDSR, 0, 0, 0, 1) != ERROR_OK) return ERROR_FAIL;
    uint32_t reg_value; 
    if (read_reg(target, SPIF_FCMD_RDDATA0, &reg_value) != ERROR_OK) return ERROR_FAIL;
    *status_out = reg_value & 0xFF; 
    return ERROR_OK;
}
static int wait_ready(struct target *target) {
    uint8_t status = 0; int timeout = 100000;
    do { read_status(target, &status); if (!(status & 0x01)) return ERROR_OK; } while (--timeout);
    return ERROR_FAIL;
}
static int flash_unlock(struct target *target) {
    send_cmd(target, FCMD_WREN, 0, 0, 0, 0);
    write_reg(target, SPIF_FCMD_WRDATA0, 0x00);
    send_cmd(target, 0x01, 0, 0, 1, 0);
    return wait_ready(target);
}
static int flash_lock(struct target *target) {
    send_cmd(target, FCMD_WREN, 0, 0, 0, 0);
    write_reg(target, SPIF_FCMD_WRDATA0, 0x7C);
    send_cmd(target, 0x01, 0, 0, 1, 0);
    return wait_ready(target);
}
static int disable_wp(struct target *target) {
    return write_reg(target, SPIF_WR_PROTECTION, 0);
}
static int erase_sector(struct flash_bank *bank, uint32_t sector_idx) {
    struct target *target = bank->target;
    uint32_t addr = bank->sectors[sector_idx].offset + bank->base;
    if (disable_wp(target) != ERROR_OK) return ERROR_FAIL;
    if (flash_unlock(target) != ERROR_OK) return ERROR_FAIL;
    send_cmd(target, FCMD_WREN, 0, 0, 0, 0);
    send_cmd(target, FCMD_SE, addr, 3, 0, 0);
    int result = wait_ready(target);
    flash_lock(target);
    return result;
}
static const uint8_t rom_stub[] = {
    0x00, 0xB5, 0x02, 0x4B, 0x18, 0x47, 0x00, 0xBD,
    (ROM_SPIF_WRITE_ADDR & 0xFF), ((ROM_SPIF_WRITE_ADDR >> 8) & 0xFF),
    ((ROM_SPIF_WRITE_ADDR >> 16) & 0xFF), ((ROM_SPIF_WRITE_ADDR >> 24) & 0xFF),
};
static int write_common(struct flash_bank *bank, uint32_t offset, const uint8_t *buffer, uint32_t length, int use_rom) {
    struct target *target = bank->target;
    int result = ERROR_OK; // Initialize to avoid uninitialized use
    if (disable_wp(target) != ERROR_OK) return ERROR_FAIL;
    if (flash_unlock(target) != ERROR_OK) return ERROR_FAIL;
    if (use_rom) {
        wait_idle(target);
        if ((result = target_write_buffer(target, ROM_STUB_ADDR, sizeof(rom_stub), rom_stub))) return result;
        if ((result = target_write_buffer(target, ROM_BUFFER_ADDR, length, buffer))) return result;
        struct reg_param reg_params[3];
        init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
        init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
        init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
        buf_set_u32(reg_params[0].value, 0, 32, PHYPLUS6252_FLASH_BASE + offset);
        buf_set_u32(reg_params[1].value, 0, 32, ROM_BUFFER_ADDR);
        buf_set_u32(reg_params[2].value, 0, 32, length);
        result = target_run_algorithm(target, 0, NULL, 3, reg_params, ROM_STUB_ADDR, 0, 5000, NULL);
        destroy_reg_param(&reg_params[0]); destroy_reg_param(&reg_params[1]); destroy_reg_param(&reg_params[2]);
        wait_idle(target);
    } else {
        uint32_t addr = PHYPLUS6252_FLASH_BASE + offset, written = 0;
        while (written < length) {
            uint32_t chunk = (length - written > 256) ? 256 : (length - written);
            send_cmd(target, FCMD_WREN, 0, 0, 0, 0);
            for (uint32_t i = 0; i < chunk; i += 4) {
                uint32_t word = buffer[written + i] | (buffer[written + i + 1] << 8) |
                                (buffer[written + i + 2] << 16) | (buffer[written + i + 3] << 24);
                write_reg(target, SPIF_FCMD_WRDATA0, word);
            }
            send_cmd(target, FCMD_PP, addr + written, 3, chunk, 0);
            wait_ready(target);
            written += chunk;
        }
    }
    flash_lock(target);
    if (result != ERROR_OK) LOG_ERROR("Flash write failed at 0x%08" PRIx32, offset);
    return result;
}
static int phyplus6252_flash_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count) {
    return target_read_memory(bank->target, PHYPLUS6252_FLASH_BASE + offset, 1, count, buffer);
}
static int phyplus6252_flash_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count) {
#if ROM_BASED_WRITE
    return write_common(bank, offset, buffer, count, 1);
#else
    return write_common(bank, offset, buffer, count, 0);
#endif
}
static int probe(struct flash_bank *bank) {
    struct target *target = bank->target;
    uint8_t jedec[3] = {0};
    target_write_u32(target, PHYPLUS6252_SPIF_BASE + SPIF_FCMD, JEDEC_ID_CMD);
    uint32_t reg_value; target_read_u32(target, PHYPLUS6252_SPIF_BASE + SPIF_FCMD_RDDATA0, &reg_value);
    jedec[0] = reg_value & 0xFF; jedec[1] = (reg_value >> 8) & 0xFF; jedec[2] = (reg_value >> 16) & 0xFF;
    LOG_INFO("PHYPLUS6252 JEDEC ID: %02X %02X %02X", jedec[0], jedec[1], jedec[2]);
    bank->size = 0x40000; bank->num_sectors = bank->size / 0x1000;
    bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
    for (unsigned int i = 0; i < bank->num_sectors; i++) {
        bank->sectors[i].offset = i * 0x1000;
        bank->sectors[i].size = 0x1000;
        bank->sectors[i].is_erased = -1;
        bank->sectors[i].is_protected = 0;
    }
    if (jedec[0] != EXPECTED_MANUFACTURER_ID)
        LOG_ERROR("Unexpected JEDEC manufacturer ID");
    return ERROR_OK;
}
static int erase(struct flash_bank *bank, unsigned int first, unsigned int last) {
    for (unsigned int sector = first; sector <= last; sector++)
        if (erase_sector(bank, sector) != ERROR_OK) return ERROR_FAIL;
    return ERROR_OK;
}
static int protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last) { return ERROR_FAIL; }
static int auto_probe(struct flash_bank *bank) { return probe(bank); }
static int info(struct flash_bank *bank, struct command_invocation *cmd) {
    command_print(cmd, "PHYPLUS6252 custom driver: base=0x%08" PRIx64 ", size=0x%08" PRIx64,
                  (uint64_t)bank->base, (uint64_t)bank->size);
    return ERROR_OK;
}
FLASH_BANK_COMMAND_HANDLER(flash_bank_command) { return ERROR_OK; }

const struct flash_driver phyplus6252_flash = {
    .name = "phyplus6252",
    .erase = erase,
    .write = phyplus6252_flash_write,
    .read = phyplus6252_flash_read,
    .probe = probe,
    .auto_probe = auto_probe,
    .protect = protect,
    .info = info,
    .flash_bank_command = flash_bank_command,
    .free_driver_priv = default_flash_free_driver_priv,
};
