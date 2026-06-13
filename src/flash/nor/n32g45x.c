// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/cortex_m.h>

/* N32G45x register locations */
#define FLASH_REG_BASE      0x40022000
#define DBGMCU_IDCODE       0xE0042000
#define STM32_OB_RDP        0x1FFFF800

/* Register Offsets */
#define STM32_FLASH_ACR     0x00
#define STM32_FLASH_KEYR    0x04
#define STM32_FLASH_OPTKEYR 0x08
#define STM32_FLASH_SR      0x0C
#define STM32_FLASH_CR      0x10
#define STM32_FLASH_AR      0x14
#define STM32_FLASH_OBR     0x1C
#define STM32_FLASH_WRPR    0x20

/* FLASH_CR register bits */
#define FLASH_PG            (1 << 0)
#define FLASH_PER           (1 << 1)
#define FLASH_MER           (1 << 2)
#define FLASH_OPTPG         (1 << 4)
#define FLASH_OPTER         (1 << 5)
#define FLASH_STRT          (1 << 6)
#define FLASH_LOCK          (1 << 7)
#define FLASH_OPTWRE        (1 << 9)
#define FLASH_OBL_LAUNCH    (1 << 13)

/* FLASH_SR register bits */
#define FLASH_BSY           (1 << 0)
#define FLASH_PGERR         (1 << 2)
#define FLASH_WRPRTERR      (1 << 4)
#define FLASH_EOP           (1 << 5)

/* STM32_FLASH_OBR bit definitions (reading) */
#define OPT_ERROR           0
#define OPT_READOUT         1
#define OPT_RDWDGSW         2
#define OPT_RDRSTSTOP       3
#define OPT_RDRSTSTDBY      4

/* register unlock keys */
#define KEY1                0x45670123
#define KEY2                0xCDEF89AB

/* timeout values */
#define FLASH_WRITE_TIMEOUT 10
#define FLASH_ERASE_TIMEOUT 100

struct n32g45x_options {
    uint8_t rdp;
    uint8_t user;
    uint16_t data;
    uint32_t protection;
};

struct n32g45x_flash_bank {
    struct n32g45x_options option_bytes;
    int ppage_size;
    bool probed;
    bool can_load_options;
    uint32_t register_base;
    uint8_t default_rdp;
    int user_data_offset;
    int option_offset;
    uint32_t user_bank_size;
};

static int n32g45x_mass_erase(struct flash_bank *bank);
static int n32g45x_write_block(struct flash_bank *bank, const uint8_t *buffer,
        uint32_t address, uint32_t words_count);

/* flash bank n32g45x <base> <size> 0 0 <target#> */
FLASH_BANK_COMMAND_HANDLER(n32g45x_flash_bank_command)
{
    struct n32g45x_flash_bank *n32g45x_info;

    if (CMD_ARGC < 6)
        return ERROR_COMMAND_SYNTAX_ERROR;

    n32g45x_info = malloc(sizeof(struct n32g45x_flash_bank));

    bank->driver_priv = n32g45x_info;
    n32g45x_info->probed = false;
    n32g45x_info->can_load_options = false;
    n32g45x_info->register_base = FLASH_REG_BASE;
    n32g45x_info->user_bank_size = bank->size;

    /* The flash write must be aligned to a word boundary (4 bytes for fast write) */
    bank->write_start_alignment = bank->write_end_alignment = 4;

    return ERROR_OK;
}

static inline int n32g45x_get_flash_reg(struct flash_bank *bank, uint32_t reg)
{
    struct n32g45x_flash_bank *n32g45x_info = bank->driver_priv;
    return reg + n32g45x_info->register_base;
}

static inline int n32g45x_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
    struct target *target = bank->target;
    return target_read_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_SR), status);
}

static int n32g45x_get_device_id(struct flash_bank *bank, uint32_t *device_id)
{
    return target_read_u32(bank->target, DBGMCU_IDCODE, device_id);
}

static int n32g45x_wait_status_busy(struct flash_bank *bank, int timeout)
{
    struct target *target = bank->target;
    uint32_t status;
    int retval = ERROR_OK;

    /* wait for busy to clear */
    for (;;) {
        retval = n32g45x_get_flash_status(bank, &status);
        if (retval != ERROR_OK)
            return retval;
        LOG_DEBUG("status: 0x%" PRIx32 "", status);
        if ((status & FLASH_BSY) == 0)
            break;
        if (timeout-- <= 0) {
            LOG_ERROR("timed out waiting for flash");
            return ERROR_FLASH_BUSY;
        }
        alive_sleep(1);
    }

    if (status & FLASH_WRPRTERR) {
        LOG_ERROR("n32g45x device protected");
        retval = ERROR_FLASH_PROTECTED;
    }

    if (status & FLASH_PGERR) {
        LOG_ERROR("n32g45x device programming failed / flash not erased");
        retval = ERROR_FLASH_OPERATION_FAILED;
    }

    /* Clear but report errors */
    if (status & (FLASH_WRPRTERR | FLASH_PGERR)) {
        target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_SR),
                FLASH_WRPRTERR | FLASH_PGERR);
    }
    return retval;
}

static int n32g45x_read_options(struct flash_bank *bank)
{
    struct n32g45x_flash_bank *n32g45x_info = bank->driver_priv;
    struct target *target = bank->target;
    uint32_t option_bytes;
    int retval;

    retval = target_read_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_OBR), &option_bytes);
    if (retval != ERROR_OK)
        return retval;

    n32g45x_info->option_bytes.rdp = (option_bytes & (1 << OPT_READOUT)) ? 0 : n32g45x_info->default_rdp;
    n32g45x_info->option_bytes.user = (option_bytes >> n32g45x_info->option_offset) & 0xff;
    n32g45x_info->option_bytes.data = (option_bytes >> n32g45x_info->user_data_offset) & 0xffff;

    retval = target_read_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_WRPR), &n32g45x_info->option_bytes.protection);
    if (retval != ERROR_OK)
        return retval;

    return ERROR_OK;
}

static int n32g45x_erase_options(struct flash_bank *bank)
{
    struct n32g45x_flash_bank *n32g45x_info = bank->driver_priv;
    struct target *target = bank->target;

    n32g45x_read_options(bank);

    int retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_KEYR), KEY1);
    if (retval != ERROR_OK)
        return retval;
    retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_KEYR), KEY2);
    if (retval != ERROR_OK)
        goto flash_lock;

    retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_OPTKEYR), KEY1);
    if (retval != ERROR_OK)
        goto flash_lock;
    retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_OPTKEYR), KEY2);
    if (retval != ERROR_OK)
        goto flash_lock;

    retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_OPTER | FLASH_OPTWRE);
    if (retval != ERROR_OK)
        goto flash_lock;
    retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_OPTER | FLASH_STRT | FLASH_OPTWRE);
    if (retval != ERROR_OK)
        goto flash_lock;

    retval = n32g45x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
    if (retval != ERROR_OK)
        goto flash_lock;

    n32g45x_info->option_bytes.rdp = n32g45x_info->default_rdp;

    return ERROR_OK;

flash_lock:
    target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_LOCK);
    return retval;
}

static int n32g45x_write_options(struct flash_bank *bank)
{
    struct n32g45x_flash_bank *n32g45x_info = bank->driver_priv;
    struct target *target = bank->target;

    int retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_KEYR), KEY1);
    if (retval != ERROR_OK)
        return retval;
    retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_KEYR), KEY2);
    if (retval != ERROR_OK)
        goto flash_lock;

    retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_OPTKEYR), KEY1);
    if (retval != ERROR_OK)
        goto flash_lock;
    retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_OPTKEYR), KEY2);
    if (retval != ERROR_OK)
        goto flash_lock;

    retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_OPTPG | FLASH_OPTWRE);
    if (retval != ERROR_OK)
        goto flash_lock;

    uint8_t opt_bytes[16];

    target_buffer_set_u16(target, opt_bytes, n32g45x_info->option_bytes.rdp);
    target_buffer_set_u16(target, opt_bytes + 2, n32g45x_info->option_bytes.user);
    target_buffer_set_u16(target, opt_bytes + 4, n32g45x_info->option_bytes.data & 0xff);
    target_buffer_set_u16(target, opt_bytes + 6, (n32g45x_info->option_bytes.data >> 8) & 0xff);
    target_buffer_set_u16(target, opt_bytes + 8, n32g45x_info->option_bytes.protection & 0xff);
    target_buffer_set_u16(target, opt_bytes + 10, (n32g45x_info->option_bytes.protection >> 8) & 0xff);
    target_buffer_set_u16(target, opt_bytes + 12, (n32g45x_info->option_bytes.protection >> 16) & 0xff);
    target_buffer_set_u16(target, opt_bytes + 14, (n32g45x_info->option_bytes.protection >> 24) & 0xff);

    retval = n32g45x_write_block(bank, opt_bytes, STM32_OB_RDP, sizeof(opt_bytes) / 4);

flash_lock:
    {
        int retval2 = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_LOCK);
        if (retval == ERROR_OK)
            retval = retval2;
    }
    return retval;
}

static int n32g45x_protect_check(struct flash_bank *bank)
{
    struct target *target = bank->target;
    uint32_t protection;

    int retval = target_read_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_WRPR), &protection);
    if (retval != ERROR_OK)
        return retval;

    for (unsigned int i = 0; i < bank->num_prot_blocks; i++)
        bank->prot_blocks[i].is_protected = (protection & (1 << i)) ? 0 : 1;

    return ERROR_OK;
}

static int n32g45x_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
    struct target *target = bank->target;

    if (bank->target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    if ((first == 0) && (last == (bank->num_sectors - 1)))
        return n32g45x_mass_erase(bank);

    int retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_KEYR), KEY1);
    if (retval != ERROR_OK)
        return retval;
    retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_KEYR), KEY2);
    if (retval != ERROR_OK)
        goto flash_lock;

    for (unsigned int i = first; i <= last; i++) {
        retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_PER);
        if (retval != ERROR_OK)
            goto flash_lock;
        
        retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_AR),
                bank->base + bank->sectors[i].offset);
        if (retval != ERROR_OK)
            goto flash_lock;
        
        retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_CR), 
                FLASH_PER | FLASH_STRT);
        if (retval != ERROR_OK)
            goto flash_lock;

        retval = n32g45x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
        if (retval != ERROR_OK)
            goto flash_lock;
    }

flash_lock:
    {
        int retval2 = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_LOCK);
        if (retval == ERROR_OK)
            retval = retval2;
    }
    return retval;
}

static int n32g45x_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
    struct target *target = bank->target;
    struct n32g45x_flash_bank *n32g45x_info = bank->driver_priv;

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    int retval = n32g45x_erase_options(bank);
    if (retval != ERROR_OK) {
        LOG_ERROR("n32g45x failed to erase options");
        return retval;
    }

    for (unsigned int i = first; i <= last; i++) {
        if (set)
            n32g45x_info->option_bytes.protection &= ~(1 << i);
        else
            n32g45x_info->option_bytes.protection |= (1 << i);
    }

    return n32g45x_write_options(bank);
}

static int n32g45x_write_block_async(struct flash_bank *bank, const uint8_t *buffer,
        uint32_t address, uint32_t words_count)
{
    struct n32g45x_flash_bank *n32g45x_info = bank->driver_priv;
    struct target *target = bank->target;
    uint32_t buffer_size;
    struct working_area *write_algorithm;
    struct working_area *source;
    struct armv7m_algorithm armv7m_info;
    int retval;

    static const uint8_t n32g45x_flash_write_code[] = {
#include "../../../contrib/loaders/flash/n32/n32g45x.inc"
    };

    if (target_alloc_working_area(target, sizeof(n32g45x_flash_write_code),
            &write_algorithm) != ERROR_OK) {
        LOG_WARNING("no working area available, can't do block memory writes");
        return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
    }

    retval = target_write_buffer(target, write_algorithm->address,
            sizeof(n32g45x_flash_write_code), n32g45x_flash_write_code);
    if (retval != ERROR_OK) {
        target_free_working_area(target, write_algorithm);
        return retval;
    }

    buffer_size = target_get_working_area_avail(target);
    buffer_size = MIN(words_count * 4 + 8, MAX(buffer_size, 256));
    
    LOG_INFO("target_alloc_working_area: words_count=%d, buffer_size=%d", words_count, buffer_size);
    retval = target_alloc_working_area(target, buffer_size, &source);
    if (retval != ERROR_OK) {
        target_free_working_area(target, write_algorithm);
        LOG_WARNING("no large enough working area available, can't do block memory writes");
        return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
    }

    struct reg_param reg_params[5];

    init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT); /* flash base (in), status (out) */
    init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);    /* count (words-32bit) */
    init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);    /* buffer start */
    init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);    /* buffer end */
    init_reg_param(&reg_params[4], "r4", 32, PARAM_IN_OUT); /* target address */

    buf_set_u32(reg_params[0].value, 0, 32, n32g45x_info->register_base);
    buf_set_u32(reg_params[1].value, 0, 32, words_count);
    buf_set_u32(reg_params[2].value, 0, 32, source->address);
    buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
    buf_set_u32(reg_params[4].value, 0, 32, address);

    armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
    armv7m_info.core_mode = ARM_MODE_THREAD;

    retval = target_run_flash_async_algorithm(target, buffer, words_count, 4,
            0, NULL,
            ARRAY_SIZE(reg_params), reg_params,
            source->address, source->size,
            write_algorithm->address, 0,
            &armv7m_info);

    if (retval == ERROR_FLASH_OPERATION_FAILED) {
        int retval2 = n32g45x_wait_status_busy(bank, 5);
        if (retval2 != ERROR_OK)
            retval = retval2;

        LOG_ERROR("flash write failed just before address 0x%"PRIx32,
                buf_get_u32(reg_params[4].value, 0, 32));
    }

    for (unsigned int i = 0; i < ARRAY_SIZE(reg_params); i++)
        destroy_reg_param(&reg_params[i]);

    target_free_working_area(target, source);
    target_free_working_area(target, write_algorithm);

    return retval;
}

static int n32g45x_write_block(struct flash_bank *bank, const uint8_t *buffer, 
        uint32_t address, uint32_t words_count)
{
    struct target *target = bank->target;
    int retval = ERROR_OK;

    assert(address % 4 == 0);

    retval = n32g45x_write_block_async(bank, buffer, address, words_count);
    
    if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
        LOG_WARNING("couldn't use block writes, falling back to slow memory accesses");
        
        while (words_count > 0) {
            retval = target_write_memory(target, address, 4, 1, buffer);
            if (retval != ERROR_OK)
                return retval;

            retval = n32g45x_wait_status_busy(bank, 5);
            if (retval != ERROR_OK)
                return retval;

            words_count--;
            buffer += 4;
            address += 4;
        }
    }
    return retval;
}

static int n32g45x_write(struct flash_bank *bank, const uint8_t *buffer,
        uint32_t offset, uint32_t count)
{
    struct target *target = bank->target;

    if (bank->target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    if (offset % 4 != 0 || count % 4 != 0) {
        LOG_ERROR("N32G45x write offset and count must be word-aligned (multiples of 4)");
        return ERROR_FAIL;
    }

    LOG_INFO("n32g45x_write(bank, buffer, offset=%d, count=%d)", offset, count);
    int retval, retval2;

    retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_KEYR), KEY1);
    if (retval != ERROR_OK)
        return retval;
    retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_KEYR), KEY2);
    if (retval != ERROR_OK)
        goto reset_pg_and_lock;

    retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_PG);
    if (retval != ERROR_OK)
        goto reset_pg_and_lock;
    
    retval = n32g45x_write_block(bank, buffer, bank->base + offset, count / 4);

reset_pg_and_lock:
    retval2 = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_LOCK);
    if (retval == ERROR_OK)
        retval = retval2;

    return retval;
}

static int n32g45x_probe(struct flash_bank *bank)
{
    struct n32g45x_flash_bank *n32g45x_info = bank->driver_priv;
    uint16_t flash_size_in_kb;
    uint16_t max_flash_size_in_kb;
    uint32_t dbgmcu_idcode;
    int page_size;
    uint32_t base_address = FLASH_REG_BASE;

    n32g45x_info->probed = false;
    n32g45x_info->register_base = FLASH_REG_BASE;
    
    n32g45x_info->user_data_offset = 10;
    n32g45x_info->option_offset = 2;
    n32g45x_info->default_rdp = 0xA5;

    int retval = n32g45x_get_device_id(bank, &dbgmcu_idcode);
    if (retval != ERROR_OK)
        return retval;

    LOG_INFO("device id = 0x%08" PRIx32 "", dbgmcu_idcode);

    uint16_t dev_num_h = (dbgmcu_idcode >> 12) & 0x0F;
    uint16_t dev_num_m = (dbgmcu_idcode >> 8)  & 0x0F;
    uint16_t dev_num_l = (dbgmcu_idcode >> 20) & 0x0F;
    uint16_t device_id = (dev_num_h << 8) | (dev_num_m << 4) | dev_num_l;

    switch (device_id) {
    case 0x452:
    case 0x455:
    case 0x457:
        page_size = 2048;                
        n32g45x_info->ppage_size = 2;
        max_flash_size_in_kb = 512;      
        n32g45x_info->can_load_options = true;
        break;
    default:
        LOG_WARNING("Cannot identify target as a N32G45x family (ID: 0x%03x).", device_id);
        return ERROR_FAIL;
    }

    flash_size_in_kb = ((dbgmcu_idcode >> 16) & 0x0F) * 64;

    if (flash_size_in_kb == 0) {
        LOG_WARNING("N32G45x flash size failed, probe inaccurate - assuming %dk flash",
            max_flash_size_in_kb);
        flash_size_in_kb = max_flash_size_in_kb;
    }

    if (n32g45x_info->user_bank_size) {
        LOG_INFO("ignoring flash probed value, using configured bank size");
        flash_size_in_kb = n32g45x_info->user_bank_size / 1024;
    }

    LOG_INFO("flash size = %d KiB", flash_size_in_kb);
    assert(flash_size_in_kb != 0xffff);

    int num_pages = flash_size_in_kb * 1024 / page_size;
    assert(num_pages > 0);

    free(bank->sectors);
    bank->sectors = NULL;
    free(bank->prot_blocks);
    bank->prot_blocks = NULL;

    bank->base = base_address;
    bank->size = (num_pages * page_size);
    bank->num_sectors = num_pages;
    bank->sectors = alloc_block_array(0, page_size, num_pages);
    if (!bank->sectors)
        return ERROR_FAIL;

    int num_prot_blocks = num_pages / n32g45x_info->ppage_size;
    if (num_prot_blocks > 32)
        num_prot_blocks = 32;

    bank->num_prot_blocks = num_prot_blocks;
    bank->prot_blocks = alloc_block_array(0, n32g45x_info->ppage_size * page_size, num_prot_blocks);
    if (!bank->prot_blocks)
        return ERROR_FAIL;

    if (num_prot_blocks == 32)
        bank->prot_blocks[31].size = (num_pages - (31 * n32g45x_info->ppage_size)) * page_size;

    n32g45x_info->probed = true;

    return ERROR_OK;
}

static int n32g45x_auto_probe(struct flash_bank *bank)
{
    struct n32g45x_flash_bank *n32g45x_info = bank->driver_priv;
    if (n32g45x_info->probed)
        return ERROR_OK;
    return n32g45x_probe(bank);
}

static int get_n32g45x_info(struct flash_bank *bank, struct command_invocation *cmd)
{
    uint32_t dbgmcu_idcode;

    int retval = n32g45x_get_device_id(bank, &dbgmcu_idcode);
    if (retval != ERROR_OK)
        return retval;

    uint16_t dev_num_h = (dbgmcu_idcode >> 12) & 0x0F;
    uint16_t dev_num_m = (dbgmcu_idcode >> 8)  & 0x0F;
    uint16_t dev_num_l = (dbgmcu_idcode >> 20) & 0x0F;
    uint16_t device_id = (dev_num_h << 8) | (dev_num_m << 4) | dev_num_l;

    uint8_t rev_id = dbgmcu_idcode & 0xFF;

    uint32_t flash_kb = ((dbgmcu_idcode >> 16) & 0x0F) * 64;
    uint32_t sram_kb  = (((dbgmcu_idcode >> 28) & 0x0F) + 1) * 16;

    const char *device_str;

    switch (device_id) {
    case 0x452:
        device_str = "Nations N32G452";
        break;
    case 0x455:
        device_str = "Nations N32G455";
        break;
    case 0x457:
        device_str = "Nations N32G457";
        break;
    default:
        command_print_sameline(cmd, "Cannot identify target as a N32G45x (Device ID: 0x%03x)\n", device_id);
        return ERROR_FAIL;
    }

    command_print_sameline(cmd, "%s - Rev: 0x%02x (Flash: %u KB, SRAM: %u KB)", 
                           device_str, rev_id, flash_kb, sram_kb);

    return ERROR_OK;
}

COMMAND_HANDLER(n32g45x_handle_lock_command)
{
    struct target *target = NULL;
    struct n32g45x_flash_bank *n32g45x_info = NULL;

    if (CMD_ARGC < 1)
        return ERROR_COMMAND_SYNTAX_ERROR;

    struct flash_bank *bank;
    int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
    if (retval != ERROR_OK)
        return retval;

    n32g45x_info = bank->driver_priv;
    target = bank->target;

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    if (n32g45x_erase_options(bank) != ERROR_OK) {
        command_print(CMD, "n32g45x failed to erase options");
        return ERROR_OK;
    }

    n32g45x_info->option_bytes.rdp = 0;

    if (n32g45x_write_options(bank) != ERROR_OK) {
        command_print(CMD, "n32g45x failed to lock device");
        return ERROR_OK;
    }

    command_print(CMD, "n32g45x locked");

    return ERROR_OK;
}

COMMAND_HANDLER(n32g45x_handle_unlock_command)
{
    struct target *target = NULL;

    if (CMD_ARGC < 1)
        return ERROR_COMMAND_SYNTAX_ERROR;

    struct flash_bank *bank;
    int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
    if (retval != ERROR_OK)
        return retval;

    target = bank->target;

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    if (n32g45x_erase_options(bank) != ERROR_OK) {
        command_print(CMD, "n32g45x failed to erase options");
        return ERROR_OK;
    }

    if (n32g45x_write_options(bank) != ERROR_OK) {
        command_print(CMD, "n32g45x failed to unlock device");
        return ERROR_OK;
    }

    command_print(CMD, "n32g45x unlocked.\n"
            "INFO: a reset or power cycle is required "
            "for the new settings to take effect.");

    return ERROR_OK;
}

COMMAND_HANDLER(n32g45x_handle_options_read_command)
{
    uint32_t optionbyte, protection;
    struct target *target = NULL;
    struct n32g45x_flash_bank *n32g45x_info = NULL;

    if (CMD_ARGC < 1)
        return ERROR_COMMAND_SYNTAX_ERROR;

    struct flash_bank *bank;
    int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
    if (retval != ERROR_OK)
        return retval;

    n32g45x_info = bank->driver_priv;
    target = bank->target;

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    retval = target_read_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_OBR), &optionbyte);
    if (retval != ERROR_OK)
        return retval;

    uint16_t user_data = optionbyte >> n32g45x_info->user_data_offset;

    retval = target_read_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_WRPR), &protection);
    if (retval != ERROR_OK)
        return retval;

    if (optionbyte & (1 << OPT_ERROR))
        command_print(CMD, "option byte complement error");

    command_print(CMD, "option byte register = 0x%" PRIx32 "", optionbyte);
    command_print(CMD, "write protection register = 0x%" PRIx32 "", protection);

    command_print(CMD, "read protection: %s",
                (optionbyte & (1 << OPT_READOUT)) ? "on" : "off");

    optionbyte >>= n32g45x_info->option_offset;

    command_print(CMD, "watchdog: %sware",
                (optionbyte & (1 << OPT_RDWDGSW)) ? "soft" : "hard");

    command_print(CMD, "stop mode: %sreset generated upon entry",
                (optionbyte & (1 << OPT_RDRSTSTOP)) ? "no " : "");

    command_print(CMD, "standby mode: %sreset generated upon entry",
                (optionbyte & (1 << OPT_RDRSTSTDBY)) ? "no " : "");

    command_print(CMD, "user data = 0x%02" PRIx16 "", user_data);

    return ERROR_OK;
}

COMMAND_HANDLER(n32g45x_handle_options_write_command)
{
    struct target *target = NULL;
    struct n32g45x_flash_bank *n32g45x_info = NULL;
    uint8_t optionbyte;
    uint16_t useropt;

    if (CMD_ARGC < 2)
        return ERROR_COMMAND_SYNTAX_ERROR;

    struct flash_bank *bank;
    int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
    if (retval != ERROR_OK)
        return retval;

    n32g45x_info = bank->driver_priv;
    target = bank->target;

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    retval = n32g45x_read_options(bank);
    if (retval != ERROR_OK)
        return retval;

    optionbyte = n32g45x_info->option_bytes.user;
    useropt = n32g45x_info->option_bytes.data;

    CMD_ARGC--;
    CMD_ARGV++;

    while (CMD_ARGC) {
        if (strcmp("SWWDG", CMD_ARGV[0]) == 0)
            optionbyte |= (1 << 0);
        else if (strcmp("HWWDG", CMD_ARGV[0]) == 0)
            optionbyte &= ~(1 << 0);
        else if (strcmp("NORSTSTOP", CMD_ARGV[0]) == 0)
            optionbyte |= (1 << 1);
        else if (strcmp("RSTSTOP", CMD_ARGV[0]) == 0)
            optionbyte &= ~(1 << 1);
        else if (strcmp("NORSTSTNDBY", CMD_ARGV[0]) == 0)
            optionbyte |= (1 << 2);
        else if (strcmp("RSTSTNDBY", CMD_ARGV[0]) == 0)
            optionbyte &= ~(1 << 2);
        else if (strcmp("USEROPT", CMD_ARGV[0]) == 0) {
            if (CMD_ARGC < 2)
                return ERROR_COMMAND_SYNTAX_ERROR;
            COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], useropt);
            CMD_ARGC--;
            CMD_ARGV++;
        } else
            return ERROR_COMMAND_SYNTAX_ERROR;
        
        CMD_ARGC--;
        CMD_ARGV++;
    }

    if (n32g45x_erase_options(bank) != ERROR_OK) {
        command_print(CMD, "n32g45x failed to erase options");
        return ERROR_OK;
    }

    n32g45x_info->option_bytes.user = optionbyte;
    n32g45x_info->option_bytes.data = useropt;

    if (n32g45x_write_options(bank) != ERROR_OK) {
        command_print(CMD, "n32g45x failed to write options");
        return ERROR_OK;
    }

    command_print(CMD, "n32g45x write options complete.\n"
                "INFO: %spower cycle is required "
                "for the new settings to take effect.",
                n32g45x_info->can_load_options
                    ? "'n32g45x options_load' command or " : "");

    return ERROR_OK;
}

COMMAND_HANDLER(n32g45x_handle_options_load_command)
{
    if (CMD_ARGC < 1)
        return ERROR_COMMAND_SYNTAX_ERROR;

    struct flash_bank *bank;
    int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
    if (retval != ERROR_OK)
        return retval;

    struct n32g45x_flash_bank *n32g45x_info = bank->driver_priv;

    if (!n32g45x_info->can_load_options) {
        LOG_ERROR("Command not applicable to these devices - power cycle is required instead.");
        return ERROR_FAIL;
    }

    struct target *target = bank->target;

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_KEYR), KEY1);
    if (retval != ERROR_OK)
        return retval;
    retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_KEYR), KEY2);
    if (retval != ERROR_OK) {
        (void)target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_LOCK);
        return retval;
    }

    retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_OBL_LAUNCH);
    if (retval != ERROR_OK)
        return retval;

    return ERROR_OK;
}

static int n32g45x_mass_erase(struct flash_bank *bank)
{
    struct target *target = bank->target;

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    int retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_KEYR), KEY1);
    if (retval != ERROR_OK)
        return retval;
    retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_KEYR), KEY2);
    if (retval != ERROR_OK)
        goto flash_lock;

    retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_MER);
    if (retval != ERROR_OK)
        goto flash_lock;
    retval = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_CR),
            FLASH_MER | FLASH_STRT);
    if (retval != ERROR_OK)
        goto flash_lock;

    retval = n32g45x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);

flash_lock:
    {
        int retval2 = target_write_u32(target, n32g45x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_LOCK);
        if (retval == ERROR_OK)
            retval = retval2;
    }
    return retval;
}

COMMAND_HANDLER(n32g45x_handle_mass_erase_command)
{
    if (CMD_ARGC < 1)
        return ERROR_COMMAND_SYNTAX_ERROR;

    struct flash_bank *bank;
    int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
    if (retval != ERROR_OK)
        return retval;

    retval = n32g45x_mass_erase(bank);
    if (retval == ERROR_OK)
        command_print(CMD, "n32g45x mass erase complete");
    else
        command_print(CMD, "n32g45x mass erase failed");

    return retval;
}

static const struct command_registration n32g45x_exec_command_handlers[] = {
    {
        .name = "lock",
        .handler = n32g45x_handle_lock_command,
        .mode = COMMAND_EXEC,
        .usage = "bank_id",
        .help = "Lock entire flash device.",
    },
    {
        .name = "unlock",
        .handler = n32g45x_handle_unlock_command,
        .mode = COMMAND_EXEC,
        .usage = "bank_id",
        .help = "Unlock entire protected flash device.",
    },
    {
        .name = "mass_erase",
        .handler = n32g45x_handle_mass_erase_command,
        .mode = COMMAND_EXEC,
        .usage = "bank_id",
        .help = "Erase entire flash device.",
    },
    {
        .name = "options_read",
        .handler = n32g45x_handle_options_read_command,
        .mode = COMMAND_EXEC,
        .usage = "bank_id",
        .help = "Read and display device option bytes.",
    },
    {
        .name = "options_write",
        .handler = n32g45x_handle_options_write_command,
        .mode = COMMAND_EXEC,
        .usage = "bank_id ('SWWDG'|'HWWDG') "
            "('RSTSTNDBY'|'NORSTSTNDBY') "
            "('RSTSTOP'|'NORSTSTOP') ('USEROPT' user_data)",
        .help = "Replace bits in device option bytes.",
    },
    {
        .name = "options_load",
        .handler = n32g45x_handle_options_load_command,
        .mode = COMMAND_EXEC,
        .usage = "bank_id",
        .help = "Force re-load of device option bytes.",
    },
    COMMAND_REGISTRATION_DONE
};

static const struct command_registration n32g45x_command_handlers[] = {
    {
        .name = "n32g45x",
        .mode = COMMAND_ANY,
        .help = "n32g45x flash command group",
        .usage = "",
        .chain = n32g45x_exec_command_handlers,
    },
    COMMAND_REGISTRATION_DONE
};

const struct flash_driver n32g45x_flash = {
    .name = "n32g45x",
    .commands = n32g45x_command_handlers,
    .flash_bank_command = n32g45x_flash_bank_command,
    .erase = n32g45x_erase,
    .protect = n32g45x_protect,
    .write = n32g45x_write,
    .read = default_flash_read,
    .probe = n32g45x_probe,
    .auto_probe = n32g45x_auto_probe,
    .erase_check = default_flash_blank_check,
    .protect_check = n32g45x_protect_check,
    .info = get_n32g45x_info,
    .free_driver_priv = default_flash_free_driver_priv,
};