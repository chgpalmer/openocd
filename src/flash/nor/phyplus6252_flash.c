/*
 * OpenOCD flash driver for PHYPLUS6252 internal SPI NOR flash (AP_SPIF controller)
 * 
 * This driver implements the unlock strategy discovered through boot ROM reverse engineering:
 * 1. Set SRAM override flag at 0x1FFF0801 = 1
 * 2. Perform soft reset to trigger boot ROM unlock check
 * 3. Use ROM's spif_write function which bypasses SPI protection registers
 * 
 * Based on boot ROM analysis: flash lock uses two-layer protection (SPI status + IOMUX gating)
 * but the boot ROM restores IOMUX access on every boot and skips lock when SRAM override is set.
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

/* Boot ROM function addresses from symbol table */
#define ROM_SPIF_WRITE_ADDR         0x10017395  /* spif_write function from ROM symbols */
#define ROM_STUB_ADDR               0x1FFF7000  /* SRAM area for stub code */
#define ROM_BUFFER_ADDR             0x1FFF7400  /* SRAM area for data buffer */

/* Boot ROM flash unlock mechanism addresses */
#define SRAM_OVERRIDE_ADDR          0x1FFF0801  /* SRAM soft override flag */
#define IOMUX_GATE_ADDR             0x40003814  /* IOMUX[0x14] - flash controller gate (corrected) */
#define IOMUX_GATE_ADDR_ALT         0x4000B014  /* Alternative address (old assumption) */
#define PCR_CACHE_BYPASS_ADDR       0x4000F000  /* _PCR_BASE_CACHE_BYPASS register */
#define PCR_SOFT_RESET_ADDR         0x4000F030  /* PCR software reset register */

static int write_reg(struct target *target, uint32_t reg, uint32_t value) {
    return target_write_u32(target, PHYPLUS6252_SPIF_BASE + reg, value);
}

static int read_reg(struct target *target, uint32_t reg, uint32_t *value) {
    return target_read_u32(target, PHYPLUS6252_SPIF_BASE + reg, value);
}

static int wait_idle(struct target *target) {
    uint32_t status; 
    int timeout = 10000;
    do { 
        read_reg(target, SPIF_FCMD, &status); 
        if (!(status & 0x02)) break; 
    } while (--timeout);
    do { 
        read_reg(target, SPIF_CONFIG, &status); 
        if (!(status & 0x80000000)) break; 
    } while (--timeout);
    return timeout ? ERROR_OK : ERROR_FAIL;
}

static int send_cmd(struct target *target, uint8_t cmd_code, uint32_t addr, int addr_len, int write_len, int read_len) {
    if (addr_len) 
        write_reg(target, SPIF_FCMD_ADDR, addr);
    uint32_t fcmd = (cmd_code & 0xFF) | ((addr_len & 0x7) << 8) | ((read_len & 0xF) << 16) | ((write_len & 0xF) << 20);
    write_reg(target, SPIF_FCMD, fcmd);
    return wait_idle(target);
}

static int read_status(struct target *target, uint8_t *status_out) {
    if (send_cmd(target, FCMD_RDSR, 0, 0, 0, 1) != ERROR_OK) 
        return ERROR_FAIL;
    uint32_t reg_value; 
    if (read_reg(target, SPIF_FCMD_RDDATA0, &reg_value) != ERROR_OK) 
        return ERROR_FAIL;
    *status_out = reg_value & 0xFF; 
    return ERROR_OK;
}

static int wait_ready(struct target *target) {
    uint8_t status = 0; 
    int timeout = 100000;
    do { 
        read_status(target, &status); 
        if (!(status & 0x01)) 
            return ERROR_OK; 
    } while (--timeout);
    return ERROR_FAIL;
}

/* Check if flash is locked by testing IOMUX gate */
static int is_flash_locked(struct target *target, bool *locked) {
    uint32_t iomux_gate, iomux_gate_alt;
    int result;
    
    /* Read both possible IOMUX gate addresses */
    result = target_read_u32(target, IOMUX_GATE_ADDR, &iomux_gate);
    if (result != ERROR_OK) {
        LOG_ERROR("Failed to read IOMUX gate at 0x%08" PRIx32, IOMUX_GATE_ADDR);
        return result;
    }
    
    result = target_read_u32(target, IOMUX_GATE_ADDR_ALT, &iomux_gate_alt);
    if (result != ERROR_OK) {
        LOG_ERROR("Failed to read IOMUX gate at 0x%08" PRIx32, IOMUX_GATE_ADDR_ALT);
        return result;
    }
    
    LOG_INFO("IOMUX gate status: 0x%08" PRIx32 " @ 0x%08" PRIx32 ", 0x%08" PRIx32 " @ 0x%08" PRIx32, 
             iomux_gate, IOMUX_GATE_ADDR, iomux_gate_alt, IOMUX_GATE_ADDR_ALT);
    
    /* Use the alternative address for now until we confirm the correct one */
    *locked = (iomux_gate_alt == 0);  /* IOMUX gate = 0 means flash access is gated/locked */
    
    if (*locked) {
        LOG_WARNING("Flash is locked (IOMUX gate = 0x%08" PRIx32 ")", iomux_gate_alt);
    } else {
        LOG_INFO("Flash is unlocked (IOMUX gate = 0x%08" PRIx32 ")", iomux_gate_alt);
    }
    
    return ERROR_OK;
}

/* Implement the SRAM override + soft reset unlock sequence */
static int phyplus6252_unlock_flash(struct target *target) {
    uint8_t override_value;
    int result;
    
    LOG_INFO("Attempting to unlock PHYPLUS6252 flash using boot ROM mechanism");
    
    /* Step 1: Check if flash is already unlocked */
    bool locked;
    result = is_flash_locked(target, &locked);
    if (result != ERROR_OK)
        return result;
        
    if (!locked) {
        LOG_INFO("Flash is already unlocked, proceeding");
        return ERROR_OK;
    }
    
    /* Step 2: Halt target if running */
    if (target->state != TARGET_HALTED) {
        LOG_INFO("Halting target for flash unlock sequence");
        result = target_halt(target);
        if (result != ERROR_OK) {
            LOG_ERROR("Failed to halt target");
            return result;
        }
        
        /* Wait for halt */
        result = target_wait_state(target, TARGET_HALTED, 1000);
        if (result != ERROR_OK) {
            LOG_ERROR("Target failed to halt within timeout");
            return result;
        }
    }
    
    /* Step 3: Set SRAM override flag - MUST be exactly 1 */
    LOG_INFO("Setting SRAM override flag at 0x%08" PRIx32, SRAM_OVERRIDE_ADDR);
    result = target_write_u8(target, SRAM_OVERRIDE_ADDR, 0x01);
    if (result != ERROR_OK) {
        LOG_ERROR("Failed to set SRAM override flag");
        return result;
    }
    
    /* Verify the flag was set correctly */
    result = target_read_u8(target, SRAM_OVERRIDE_ADDR, &override_value);
    if (result != ERROR_OK) {
        LOG_ERROR("Failed to read back SRAM override flag");
        return result;
    }
    
    if (override_value != 0x01) {
        LOG_ERROR("SRAM override flag verification failed: expected 0x01, got 0x%02x", override_value);
        return ERROR_FAIL;
    }
    
    LOG_INFO("SRAM override flag set successfully");
    
    /* Step 4: Enable cache bypass for direct hardware access */
    LOG_INFO("Enabling cache bypass for direct hardware access");
    result = target_write_u32(target, PCR_CACHE_BYPASS_ADDR, 0x01);
    if (result != ERROR_OK) {
        LOG_WARNING("Failed to set cache bypass (non-critical)");
        /* Continue anyway - this is not critical for unlock */
    }
    
    /* Step 5: Trigger software reset via PCR register */
    LOG_INFO("Triggering software reset to activate boot ROM unlock sequence");
    result = target_write_u32(target, PCR_SOFT_RESET_ADDR, 0x01);
    if (result != ERROR_OK) {
        LOG_ERROR("Failed to trigger software reset");
        return result;
    }
    
    /* Give reset time to take effect */
    alive_sleep(50);
    
    /* The target should restart and halt again due to OpenOCD configuration */
    /* Wait for the target to become responsive again */
    result = target_wait_state(target, TARGET_HALTED, 2000);
    if (result != ERROR_OK) {
        LOG_WARNING("Target did not halt after reset within timeout, attempting to halt manually");
        result = target_halt(target);
        if (result != ERROR_OK) {
            LOG_ERROR("Failed to halt target after reset");
            return result;
        }
        result = target_wait_state(target, TARGET_HALTED, 1000);
        if (result != ERROR_OK) {
            LOG_ERROR("Target failed to halt after manual halt command");
            return result;
        }
    }
    
    /* Step 6: Verify flash is now unlocked */
    LOG_INFO("Verifying flash unlock after reset");
    result = is_flash_locked(target, &locked);
    if (result != ERROR_OK)
        return result;
        
    if (locked) {
        LOG_ERROR("Flash unlock failed - flash is still locked after reset sequence");
        
        /* Try manual IOMUX gate unlock as a last resort */
        LOG_INFO("Attempting manual IOMUX gate unlock");
        
        /* Try both possible addresses */
        result = target_write_u32(target, IOMUX_GATE_ADDR, 0xFFFFFFFF);
        if (result == ERROR_OK) {
            LOG_INFO("Manually set IOMUX gate at 0x%08" PRIx32, IOMUX_GATE_ADDR);
        }
        
        result = target_write_u32(target, IOMUX_GATE_ADDR_ALT, 0xFFFFFFFF);
        if (result == ERROR_OK) {
            LOG_INFO("Manually set IOMUX gate at 0x%08" PRIx32, IOMUX_GATE_ADDR_ALT);
        }
        
        /* Check again */
        result = is_flash_locked(target, &locked);
        if (result != ERROR_OK || locked) {
            LOG_ERROR("Manual IOMUX unlock also failed");
            LOG_ERROR("This may indicate hardware issues or incorrect boot ROM behavior");
            return ERROR_FAIL;
        } else {
            LOG_INFO("Manual IOMUX unlock succeeded");
        }
    } else {
        LOG_INFO("Flash unlock succeeded via boot ROM mechanism");
    }
    
    /* Step 7: Re-enable cache bypass after reset (may have been cleared) */
    result = target_write_u32(target, PCR_CACHE_BYPASS_ADDR, 0x01);
    if (result != ERROR_OK) {
        LOG_WARNING("Failed to re-enable cache bypass after reset (non-critical)");
    }
    
    LOG_INFO("Flash unlock sequence completed successfully");
    return ERROR_OK;
}
/* Legacy SPI flash status register unlock (now supplementary to main unlock) */
static int flash_unlock(struct target *target) {
    send_cmd(target, FCMD_WREN, 0, 0, 0, 0);
    write_reg(target, SPIF_FCMD_WRDATA0, 0x00);
    send_cmd(target, 0x01, 0, 0, 1, 0);
    return wait_ready(target);
}

/* Disable write protection register */
static int disable_wp(struct target *target) {
    return write_reg(target, SPIF_WR_PROTECTION, 0);
}

/* Erase a single sector */
static int erase_sector(struct flash_bank *bank, uint32_t sector_idx) {
    struct target *target = bank->target;
    uint32_t addr = bank->sectors[sector_idx].offset + bank->base;
    
    /* Ensure flash is unlocked before erase */
    int result = phyplus6252_unlock_flash(target);
    if (result != ERROR_OK) {
        LOG_ERROR("Failed to unlock flash for sector erase");
        return result;
    }
    
    if (disable_wp(target) != ERROR_OK) 
        return ERROR_FAIL;
    if (flash_unlock(target) != ERROR_OK) 
        return ERROR_FAIL;
        
    send_cmd(target, FCMD_WREN, 0, 0, 0, 0);
    send_cmd(target, FCMD_SE, addr, 3, 0, 0);
    result = wait_ready(target);
    
    /* Note: We don't re-lock flash here as it would require another unlock sequence */
    return result;
}
/* ARM Thumb stub to call ROM spif_write function 
 * This stub calls the ROM's spif_write function at 0x10017395
 * Parameters: r0=flash_addr, r1=data_ptr, r2=length
 */
static const uint8_t rom_stub[] = {
    0x00, 0xB5,                                        /* push {lr}           */
    0x02, 0x4B,                                        /* ldr r3, [pc, #8]    */
    0x18, 0x47,                                        /* bx r3               */
    0x00, 0xBD,                                        /* pop {pc}            */
    /* Address of ROM spif_write function (little-endian) */
    (ROM_SPIF_WRITE_ADDR & 0xFF), 
    ((ROM_SPIF_WRITE_ADDR >> 8) & 0xFF),
    ((ROM_SPIF_WRITE_ADDR >> 16) & 0xFF), 
    ((ROM_SPIF_WRITE_ADDR >> 24) & 0xFF),
};

static int write_common(struct flash_bank *bank, uint32_t offset, const uint8_t *buffer, uint32_t length, int use_rom) {
    struct target *target = bank->target;
    int result = ERROR_OK;
    
    /* Ensure flash is unlocked before any write operation */
    result = phyplus6252_unlock_flash(target);
    if (result != ERROR_OK) {
        LOG_ERROR("Failed to unlock flash for write operation");
        return result;
    }
    
    /* Check for writes to protected boot sectors */
    if (offset < 0x2000) {
        LOG_ERROR("Write to protected boot sector (offset 0x%08" PRIx32 ") rejected", offset);
        return ERROR_FAIL;
    }
    
    if (disable_wp(target) != ERROR_OK) 
        return ERROR_FAIL;
    if (flash_unlock(target) != ERROR_OK) 
        return ERROR_FAIL;
        
    if (use_rom) {
        /* Use ROM spif_write function for reliable writes */
        LOG_INFO("Using ROM spif_write function for %u bytes at offset 0x%08" PRIx32, length, offset);
        
        wait_idle(target);
        
        /* Upload stub code to SRAM */
        if ((result = target_write_buffer(target, ROM_STUB_ADDR, sizeof(rom_stub), rom_stub))) {
            LOG_ERROR("Failed to upload ROM stub code");
            return result;
        }
        
        /* Upload data to SRAM buffer */
        if ((result = target_write_buffer(target, ROM_BUFFER_ADDR, length, buffer))) {
            LOG_ERROR("Failed to upload data buffer");
            return result;
        }
        
        /* Set up parameters for ROM function call */
        struct reg_param reg_params[3];
        init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);  /* flash address */
        init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);  /* data pointer */
        init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);  /* length */
        
        buf_set_u32(reg_params[0].value, 0, 32, PHYPLUS6252_FLASH_BASE + offset);
        buf_set_u32(reg_params[1].value, 0, 32, ROM_BUFFER_ADDR);
        buf_set_u32(reg_params[2].value, 0, 32, length);
        
        /* Execute ROM function via stub */
        result = target_run_algorithm(target, 0, NULL, 3, reg_params, ROM_STUB_ADDR, 0, 10000, NULL);
        
        if (result != ERROR_OK) {
            LOG_ERROR("ROM spif_write algorithm execution failed");
        } else {
            LOG_INFO("ROM spif_write completed successfully");
        }
        
        /* Clean up parameters */
        destroy_reg_param(&reg_params[0]); 
        destroy_reg_param(&reg_params[1]); 
        destroy_reg_param(&reg_params[2]);
        
        wait_idle(target);
    } else {
        /* Fallback: Direct SPI controller write (less reliable when locked) */
        LOG_WARNING("Using direct SPI controller write (may fail if flash is locked)");
        uint32_t addr = PHYPLUS6252_FLASH_BASE + offset;
        uint32_t written = 0;
        
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
    
    /* Note: We don't re-lock flash here as it would require another unlock sequence */
    
    if (result != ERROR_OK) {
        LOG_ERROR("Flash write failed at offset 0x%08" PRIx32, offset);
    }
    
    return result;
}
static int phyplus6252_flash_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count) {
    return target_read_memory(bank->target, PHYPLUS6252_FLASH_BASE + offset, 1, count, buffer);
}

static int phyplus6252_flash_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count) {
#if ROM_BASED_WRITE
    return write_common(bank, offset, buffer, count, 1);  /* Use ROM-based write by default */
#else
    return write_common(bank, offset, buffer, count, 0);  /* Use direct SPI write */
#endif
}

static int probe(struct flash_bank *bank) {
    struct target *target = bank->target;
    uint8_t jedec[3] = {0};
    
    /* Attempt to unlock flash for probing */
    int unlock_result = phyplus6252_unlock_flash(target);
    if (unlock_result != ERROR_OK) {
        LOG_WARNING("Flash unlock failed during probe, JEDEC detection may fail");
        /* Continue anyway - might work if flash was already unlocked */
    }
    
    /* Read JEDEC ID */
    target_write_u32(target, PHYPLUS6252_SPIF_BASE + SPIF_FCMD, JEDEC_ID_CMD | (3 << 16));  /* Read 3 bytes */
    wait_idle(target);
    
    uint32_t reg_value; 
    target_read_u32(target, PHYPLUS6252_SPIF_BASE + SPIF_FCMD_RDDATA0, &reg_value);
    jedec[0] = reg_value & 0xFF; 
    jedec[1] = (reg_value >> 8) & 0xFF; 
    jedec[2] = (reg_value >> 16) & 0xFF;
    
    LOG_INFO("PHYPLUS6252 JEDEC ID: %02X %02X %02X", jedec[0], jedec[1], jedec[2]);
    
    /* Set flash size based on JEDEC ID or default to 1MB */
    if (jedec[0] == 0x01 && jedec[1] == 0x40 && jedec[2] == 0x17) {
        bank->size = 0x100000;  /* 1MB */
        LOG_INFO("Detected 1MB flash");
    } else if (jedec[0] == 0x01 && jedec[1] == 0x40 && jedec[2] == 0x16) {
        bank->size = 0x80000;   /* 512KB */
        LOG_INFO("Detected 512KB flash");
    } else if (jedec[0] == 0x01 && jedec[1] == 0x40 && jedec[2] == 0x15) {
        bank->size = 0x40000;   /* 256KB */
        LOG_INFO("Detected 256KB flash");
    } else {
        bank->size = 0x100000;  /* Default to 1MB */
        LOG_WARNING("Unknown JEDEC ID, defaulting to 1MB flash size");
    }
    
    /* Set up sectors (4KB each) */
    bank->num_sectors = bank->size / 0x1000;
    bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
    
    for (unsigned int i = 0; i < bank->num_sectors; i++) {
        bank->sectors[i].offset = i * 0x1000;
        bank->sectors[i].size = 0x1000;
        bank->sectors[i].is_erased = -1;
        bank->sectors[i].is_protected = (i < 2) ? 1 : 0;  /* Protect first 8KB (boot sectors) */
    }
    
    if (jedec[0] != EXPECTED_MANUFACTURER_ID && jedec[0] != 0x00) {
        LOG_ERROR("Unexpected JEDEC manufacturer ID: 0x%02X (expected 0x%02X)", 
                  jedec[0], EXPECTED_MANUFACTURER_ID);
    }
    
    LOG_INFO("Flash probe completed: %u sectors of 4KB each", bank->num_sectors);
    return ERROR_OK;
}
static int erase(struct flash_bank *bank, unsigned int first, unsigned int last) {
    LOG_INFO("Erasing sectors %u to %u", first, last);
    
    for (unsigned int sector = first; sector <= last; sector++) {
        /* Check if sector is protected */
        if (bank->sectors[sector].is_protected) {
            LOG_ERROR("Sector %u is protected (boot sector), skipping", sector);
            return ERROR_FAIL;
        }
        
        LOG_INFO("Erasing sector %u (offset 0x%08" PRIx32 ")", 
                 sector, bank->sectors[sector].offset);
                 
        if (erase_sector(bank, sector) != ERROR_OK) {
            LOG_ERROR("Failed to erase sector %u", sector);
            return ERROR_FAIL;
        }
        
        bank->sectors[sector].is_erased = 1;
    }
    
    LOG_INFO("Erase completed successfully");
    return ERROR_OK;
}

static int protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last) { 
    /* Boot sectors (0-1) are always protected by hardware */
    for (unsigned int i = first; i <= last; i++) {
        if (i < 2) {
            LOG_WARNING("Sector %u is a boot sector and cannot be unprotected", i);
            continue;
        }
        bank->sectors[i].is_protected = set;
    }
    return ERROR_OK; 
}

static int auto_probe(struct flash_bank *bank) { 
    return probe(bank); 
}

static int info(struct flash_bank *bank, struct command_invocation *cmd) {
    command_print(cmd, "PHYPLUS6252 flash driver");
    command_print(cmd, "  Base address: 0x%08" PRIx64, (uint64_t)bank->base);
    command_print(cmd, "  Flash size: 0x%08" PRIx64 " (%u KB)", 
                  (uint64_t)bank->size, (unsigned)(bank->size / 1024));
    command_print(cmd, "  Sectors: %u (4KB each)", bank->num_sectors);
    command_print(cmd, "  Boot ROM unlock mechanism: SRAM override + soft reset");
    command_print(cmd, "  ROM spif_write address: 0x%08X", ROM_SPIF_WRITE_ADDR);
    return ERROR_OK;
}

/* Manual unlock command for troubleshooting */
COMMAND_HANDLER(phyplus6252_unlock_command) {
    struct target *target = get_current_target(CMD_CTX);
    
    if (CMD_ARGC != 0) {
        command_print(CMD, "Usage: phyplus6252 unlock");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }
    
    command_print(CMD, "Attempting to unlock PHYPLUS6252 flash...");
    
    int result = phyplus6252_unlock_flash(target);
    if (result == ERROR_OK) {
        command_print(CMD, "Flash unlock successful");
    } else {
        command_print(CMD, "Flash unlock failed");
    }
    
    return result;
}

/* Check flash lock status command */
COMMAND_HANDLER(phyplus6252_status_command) {
    struct target *target = get_current_target(CMD_CTX);
    
    if (CMD_ARGC != 0) {
        command_print(CMD, "Usage: phyplus6252 status");
        return ERROR_COMMAND_SYNTAX_ERROR;
    }
    
    bool locked;
    int result = is_flash_locked(target, &locked);
    if (result != ERROR_OK) {
        command_print(CMD, "Failed to read flash lock status");
        return result;
    }
    
    /* Read SRAM override flag */
    uint8_t override_flag;
    result = target_read_u8(target, SRAM_OVERRIDE_ADDR, &override_flag);
    if (result != ERROR_OK) {
        command_print(CMD, "Failed to read SRAM override flag");
        return result;
    }
    
    command_print(CMD, "Flash lock status: %s", locked ? "LOCKED" : "UNLOCKED");
    command_print(CMD, "SRAM override flag (0x%08X): 0x%02X", SRAM_OVERRIDE_ADDR, override_flag);
    
    return ERROR_OK;
}

static const struct command_registration phyplus6252_command_handlers[] = {
    {
        .name = "unlock",
        .handler = phyplus6252_unlock_command,
        .mode = COMMAND_EXEC,
        .help = "Unlock PHYPLUS6252 flash using boot ROM mechanism",
        .usage = "",
    },
    {
        .name = "status", 
        .handler = phyplus6252_status_command,
        .mode = COMMAND_EXEC,
        .help = "Check PHYPLUS6252 flash lock status",
        .usage = "",
    },
    COMMAND_REGISTRATION_DONE
};

static const struct command_registration phyplus6252_commands[] = {
    {
        .name = "phyplus6252",
        .mode = COMMAND_ANY,
        .help = "PHYPLUS6252 flash commands",
        .usage = "",
        .chain = phyplus6252_command_handlers,
    },
    COMMAND_REGISTRATION_DONE
};

FLASH_BANK_COMMAND_HANDLER(flash_bank_command) { 
    return ERROR_OK; 
}

const struct flash_driver phyplus6252_flash = {
    .name = "phyplus6252",
    .commands = phyplus6252_commands,
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
