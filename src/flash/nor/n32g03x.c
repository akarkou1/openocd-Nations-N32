// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 * Nations N32G03x internal Flash driver
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>
#include <stdbool.h>

#include "imp.h"
#include <helper/log.h>
#include <flash/nor/core.h>
#include <target/cortex_m.h>

#define N32_FLASH_BASE      0x08000000U
#define N32_DEFAULT_SIZE    (64 * 1024U)
#define N32_DEFAULT_PAGES   128U
#define N32_DEFAULT_PAGE_SZ (N32_DEFAULT_SIZE / N32_DEFAULT_PAGES)

/* FLASH register base (AHB + 0xA000) is 0x40022000 on N32G031 */
#define N32_FLASH_REG_BASE  0x40022000U

/* FLASH register offsets */
#define N32_FLASH_AC_OFF    0x00U
#define N32_FLASH_KEY_OFF   0x04U
#define N32_FLASH_OPTKEY_OFF 0x08U
#define N32_FLASH_STS_OFF   0x0CU
#define N32_FLASH_CTRL_OFF  0x10U
#define N32_FLASH_ADD_OFF   0x14U

/* FLASH control bits */
#define FLASH_CTRL_PG       0x0001U
#define FLASH_CTRL_PER      0x0002U
#define FLASH_CTRL_MER      0x0004U
#define FLASH_CTRL_OPTPG    0x0010U
#define FLASH_CTRL_OPTER    0x0020U
#define FLASH_CTRL_START    0x0040U
#define FLASH_CTRL_LOCK     0x0080U

/* FLASH status bits */
#define FLASH_STS_BUSY      0x01U
#define FLASH_STS_PGERR     0x04U
#define FLASH_STS_WRPERR    0x10U
#define FLASH_STS_EOP       0x20U

/* FLASH keys */
#define FLASH_KEY1          0x45670123U
#define FLASH_KEY2          0xCDEF89ABU

/* Option bytes addresses (STM32F1-style layout assumed) */
#define N32_OB_RDP      0x1FFFF800U
#define N32_OB_USER     0x1FFFF802U
#define N32_OB_DATA0    0x1FFFF804U
#define N32_OB_DATA1    0x1FFFF806U
#define N32_OB_WRP0     0x1FFFF808U
#define N32_OB_WRP1     0x1FFFF80AU
#define N32_OB_WRP2     0x1FFFF80CU
#define N32_OB_WRP3     0x1FFFF80EU

struct n32g03x_flash {
    bool probed;
    uint32_t page_size;
    uint32_t register_base;
};

FLASH_BANK_COMMAND_HANDLER(n32g03x_flash_bank_command)
{
    struct n32g03x_flash *info;
    if (CMD_ARGC < 6)
        return ERROR_COMMAND_SYNTAX_ERROR;
    info = calloc(1, sizeof(*info));
    if (!info)
        return ERROR_FAIL;
    bank->driver_priv = info;
    info->probed = false;
    info->page_size = N32_DEFAULT_PAGE_SZ;
    info->register_base = N32_FLASH_REG_BASE;
    bank->write_start_alignment = bank->write_end_alignment = 2;
    return ERROR_OK;
}

static int n32g03x_probe(struct flash_bank *bank)
{
    struct n32g03x_flash *info = bank->driver_priv;
    if (!info)
        return ERROR_FAIL;
    if (bank->size == 0)
        bank->size = N32_DEFAULT_SIZE;
    uint32_t page_sz = info->page_size ? info->page_size : N32_DEFAULT_PAGE_SZ;
    unsigned int num = bank->size / page_sz;
    if (num == 0)
        return ERROR_FAIL;
    bank->num_sectors = num;
    bank->sectors = alloc_block_array(0, bank->size, num);
    if (!bank->sectors)
        return ERROR_FAIL;
    info->probed = true;
    LOG_INFO("n32g03x: probed, size=%u, page=%u, sectors=%u", bank->size, page_sz, num);
    return ERROR_OK;
}

static int n32g03x_auto_probe(struct flash_bank *bank)
{
    struct n32g03x_flash *info = bank->driver_priv;
    if (!info || !info->probed)
        return n32g03x_probe(bank);
    return ERROR_OK;
}

/* ---- Helper functions (pure C) ---- */
static int n32_read32(struct target *target, uint32_t addr, uint32_t *val)
{
    return target_read_u32(target, addr, val);
}

static int n32_write32(struct target *target, uint32_t addr, uint32_t val)
{
    return target_write_u32(target, addr, val);
}

static int n32_clear_flags(struct target *target, uint32_t base)
{
    return n32_write32(target, base + N32_FLASH_STS_OFF,
                       (FLASH_STS_PGERR | FLASH_STS_WRPERR | FLASH_STS_EOP));
}

static int n32_read16(struct target *target, uint32_t addr, uint16_t *val)
{
    return target_read_u16(target, addr, val);
}

static int n32_write16(struct target *target, uint32_t addr, uint16_t val)
{
    return target_write_u16(target, addr, val);
}

static int n32_read_opt16(struct target *target, uint32_t addr, uint16_t *val)
{
    return target_read_u16(target, addr, val);
}

static int n32_write_opt16(struct target *target, uint32_t addr, uint8_t byte)
{
    uint16_t half = (uint16_t)byte | (uint16_t)((~byte & 0xFFU) << 8);
    return n32_write16(target, addr, half);
}

static int n32_wait_ready(struct target *target, uint32_t base, uint32_t timeout)
{
    while (timeout--) {
        uint32_t sts = 0;
        if (n32_read32(target, base + N32_FLASH_STS_OFF, &sts) != ERROR_OK)
            return ERROR_FAIL;
        if (!(sts & FLASH_STS_BUSY)) {
            if (sts & (FLASH_STS_PGERR | FLASH_STS_WRPERR))
                return ERROR_FAIL;
            return ERROR_OK;
        }
    }
    return ERROR_TIMEOUT_REACHED;
}

static int n32g03x_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
    struct n32g03x_flash *info = bank->driver_priv;
    if (!info)
        return ERROR_FAIL;
    if (first > last || last >= bank->num_sectors)
        return ERROR_COMMAND_ARGUMENT_INVALID;

    struct target *target = bank->target;
    uint32_t base = info->register_base;

    /* unlock */
    if (n32_write32(target, base + N32_FLASH_KEY_OFF, FLASH_KEY1) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_write32(target, base + N32_FLASH_KEY_OFF, FLASH_KEY2) != ERROR_OK)
        return ERROR_FAIL;

    if (first == 0 && last == bank->num_sectors - 1) {
        if (n32_clear_flags(target, base) != ERROR_OK)
            return ERROR_FAIL;
        if (n32_wait_ready(target, base, 0x00200000U) != ERROR_OK)
            return ERROR_FAIL;
        uint32_t ctrl = 0;
        if (n32_read32(target, base + N32_FLASH_CTRL_OFF, &ctrl) != ERROR_OK)
            return ERROR_FAIL;
        ctrl |= FLASH_CTRL_MER;
        if (n32_write32(target, base + N32_FLASH_CTRL_OFF, ctrl) != ERROR_OK)
            return ERROR_FAIL;
        ctrl |= FLASH_CTRL_START;
        if (n32_write32(target, base + N32_FLASH_CTRL_OFF, ctrl) != ERROR_OK)
            return ERROR_FAIL;
        if (n32_wait_ready(target, base, 0x00400000U) != ERROR_OK)
            return ERROR_FAIL;
        if (n32_read32(target, base + N32_FLASH_CTRL_OFF, &ctrl) != ERROR_OK)
            return ERROR_FAIL;
        ctrl &= ~FLASH_CTRL_MER;
        if (n32_write32(target, base + N32_FLASH_CTRL_OFF, ctrl) != ERROR_OK)
            return ERROR_FAIL;
    } else {
        for (unsigned int s = first; s <= last; ++s) {
            uint32_t sector_addr = bank->base + bank->sectors[s].offset;

            if (n32_clear_flags(target, base) != ERROR_OK)
                return ERROR_FAIL;
            if (n32_wait_ready(target, base, 0x00200000U) != ERROR_OK)
                return ERROR_FAIL;

            uint32_t ctrl = 0;
            if (n32_read32(target, base + N32_FLASH_CTRL_OFF, &ctrl) != ERROR_OK)
                return ERROR_FAIL;
            ctrl |= FLASH_CTRL_PER;
            if (n32_write32(target, base + N32_FLASH_CTRL_OFF, ctrl) != ERROR_OK)
                return ERROR_FAIL;
            if (n32_write32(target, base + N32_FLASH_ADD_OFF, sector_addr) != ERROR_OK)
                return ERROR_FAIL;
            ctrl |= FLASH_CTRL_START;
            if (n32_write32(target, base + N32_FLASH_CTRL_OFF, ctrl) != ERROR_OK)
                return ERROR_FAIL;

            if (n32_wait_ready(target, base, 0x00200000U) != ERROR_OK)
                return ERROR_FAIL;

            if (n32_read32(target, base + N32_FLASH_CTRL_OFF, &ctrl) != ERROR_OK)
                return ERROR_FAIL;
            ctrl &= ~FLASH_CTRL_PER;
            if (n32_write32(target, base + N32_FLASH_CTRL_OFF, ctrl) != ERROR_OK)
                return ERROR_FAIL;
        }
    }

    /* lock */
    uint32_t ctrl = 0;
    if (n32_read32(target, base + N32_FLASH_CTRL_OFF, &ctrl) == ERROR_OK) {
        ctrl |= FLASH_CTRL_LOCK;
        n32_write32(target, base + N32_FLASH_CTRL_OFF, ctrl);
    }

    return ERROR_OK;
}

static int n32g03x_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
    struct target *target = bank->target;
    struct n32g03x_flash *info = bank->driver_priv;
    if (!info)
        return ERROR_FAIL;

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    if (first > last || last >= bank->num_sectors)
        return ERROR_COMMAND_ARGUMENT_INVALID;

    /* Read current option bytes that we will preserve */
    uint16_t rdp_hw = 0, user_hw = 0, data0_hw = 0, data1_hw = 0;
    uint16_t wrp0_hw = 0, wrp1_hw = 0, wrp2_hw = 0, wrp3_hw = 0;
    uint8_t wrp0 = 0xFF, wrp1 = 0xFF, wrp2 = 0xFF, wrp3 = 0xFF;

    if (n32_read_opt16(target, N32_OB_RDP, &rdp_hw) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_read_opt16(target, N32_OB_USER, &user_hw) != ERROR_OK)
        return ERROR_FAIL;
    (void)n32_read_opt16(target, N32_OB_DATA0, &data0_hw);
    (void)n32_read_opt16(target, N32_OB_DATA1, &data1_hw);
    if (n32_read_opt16(target, N32_OB_WRP0, &wrp0_hw) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_read_opt16(target, N32_OB_WRP1, &wrp1_hw) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_read_opt16(target, N32_OB_WRP2, &wrp2_hw) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_read_opt16(target, N32_OB_WRP3, &wrp3_hw) != ERROR_OK)
        return ERROR_FAIL;

    wrp0 = (uint8_t)(wrp0_hw & 0xFFU);
    wrp1 = (uint8_t)(wrp1_hw & 0xFFU);
    wrp2 = (uint8_t)(wrp2_hw & 0xFFU);
    wrp3 = (uint8_t)(wrp3_hw & 0xFFU);

    /* Map sectors to 32 WRP bits (coarse granularity) */
    unsigned int prot_bits = 32U;
    unsigned int group_size = (bank->num_sectors + prot_bits - 1U) / prot_bits; /* ceil */

    for (unsigned int s = first; s <= last; ++s) {
        unsigned int bit = s / group_size;
        if (bit < 8U) {
            if (set)
                wrp0 &= ~(1U << bit);
            else
                wrp0 |= (1U << bit);
        } else if (bit < 16U) {
            unsigned int b = bit - 8U;
            if (set)
                wrp1 &= ~(1U << b);
            else
                wrp1 |= (1U << b);
        } else if (bit < 24U) {
            unsigned int b = bit - 16U;
            if (set)
                wrp2 &= ~(1U << b);
            else
                wrp2 |= (1U << b);
        } else if (bit < 32U) {
            unsigned int b = bit - 24U;
            if (set)
                wrp3 &= ~(1U << b);
            else
                wrp3 |= (1U << b);
        }
    }

    uint32_t base = info->register_base;

    /* Unlock options */
    if (n32_write32(target, base + N32_FLASH_OPTKEY_OFF, FLASH_KEY1) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_write32(target, base + N32_FLASH_OPTKEY_OFF, FLASH_KEY2) != ERROR_OK)
        return ERROR_FAIL;

    /* Erase option bytes */
    if (n32_clear_flags(target, base) != ERROR_OK)
        return ERROR_FAIL;
    uint32_t ctrl = 0;
    if (n32_read32(target, base + N32_FLASH_CTRL_OFF, &ctrl) != ERROR_OK)
        return ERROR_FAIL;
    ctrl |= FLASH_CTRL_OPTER;
    if (n32_write32(target, base + N32_FLASH_CTRL_OFF, ctrl) != ERROR_OK)
        return ERROR_FAIL;
    ctrl |= FLASH_CTRL_START;
    if (n32_write32(target, base + N32_FLASH_CTRL_OFF, ctrl) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_wait_ready(target, base, 0x00200000U) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_read32(target, base + N32_FLASH_CTRL_OFF, &ctrl) != ERROR_OK)
        return ERROR_FAIL;
    ctrl &= ~FLASH_CTRL_OPTER;
    if (n32_write32(target, base + N32_FLASH_CTRL_OFF, ctrl) != ERROR_OK)
        return ERROR_FAIL;

    /* Program option bytes we preserved/updated */
    if (n32_clear_flags(target, base) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_read32(target, base + N32_FLASH_CTRL_OFF, &ctrl) != ERROR_OK)
        return ERROR_FAIL;
    ctrl |= FLASH_CTRL_OPTPG;
    if (n32_write32(target, base + N32_FLASH_CTRL_OFF, ctrl) != ERROR_OK)
        return ERROR_FAIL;

    /* RDP, USER, DATA0, DATA1 unchanged */
    if (n32_write16(target, N32_OB_RDP, rdp_hw) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_wait_ready(target, base, 0x00020000U) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_write16(target, N32_OB_USER, user_hw) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_wait_ready(target, base, 0x00020000U) != ERROR_OK)
        return ERROR_FAIL;
    (void)n32_write16(target, N32_OB_DATA0, data0_hw);
    (void)n32_wait_ready(target, base, 0x00020000U);
    (void)n32_write16(target, N32_OB_DATA1, data1_hw);
    (void)n32_wait_ready(target, base, 0x00020000U);

    /* Updated WRP values */
    if (n32_write_opt16(target, N32_OB_WRP0, wrp0) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_wait_ready(target, base, 0x00020000U) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_write_opt16(target, N32_OB_WRP1, wrp1) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_wait_ready(target, base, 0x00020000U) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_write_opt16(target, N32_OB_WRP2, wrp2) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_wait_ready(target, base, 0x00020000U) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_write_opt16(target, N32_OB_WRP3, wrp3) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_wait_ready(target, base, 0x00020000U) != ERROR_OK)
        return ERROR_FAIL;

    /* Clear OPTPG */
    if (n32_read32(target, base + N32_FLASH_CTRL_OFF, &ctrl) != ERROR_OK)
        return ERROR_FAIL;
    ctrl &= ~FLASH_CTRL_OPTPG;
    if (n32_write32(target, base + N32_FLASH_CTRL_OFF, ctrl) != ERROR_OK)
        return ERROR_FAIL;

    /* Lock */
    if (n32_read32(target, base + N32_FLASH_CTRL_OFF, &ctrl) == ERROR_OK) {
        ctrl |= FLASH_CTRL_LOCK;
        n32_write32(target, base + N32_FLASH_CTRL_OFF, ctrl);
    }

    return ERROR_OK;
}

static int n32g03x_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
    struct n32g03x_flash *info = bank->driver_priv;
    if (!info)
        return ERROR_FAIL;
    if ((offset % 2) != 0 || (count % 2) != 0)
        return ERROR_FLASH_OPER_UNSUPPORTED;

    struct target *target = bank->target;
    uint32_t base = info->register_base;
    uint32_t addr = bank->base + offset;

    /* unlock */
    if (n32_write32(target, base + N32_FLASH_KEY_OFF, FLASH_KEY1) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_write32(target, base + N32_FLASH_KEY_OFF, FLASH_KEY2) != ERROR_OK)
        return ERROR_FAIL;

    uint32_t i = 0;
    for (; i + 4 <= count; i += 4, addr += 4) {
        uint32_t word = buffer[i] | (buffer[i + 1] << 8) | (buffer[i + 2] << 16) | (buffer[i + 3] << 24);

        if (n32_clear_flags(target, base) != ERROR_OK)
            return ERROR_FAIL;
        if (n32_wait_ready(target, base, 0x00020000U) != ERROR_OK)
            return ERROR_FAIL;

        uint32_t ctrl = 0;
        if (n32_read32(target, base + N32_FLASH_CTRL_OFF, &ctrl) != ERROR_OK)
            return ERROR_FAIL;
        ctrl |= FLASH_CTRL_PG;
        if (n32_write32(target, base + N32_FLASH_CTRL_OFF, ctrl) != ERROR_OK)
            return ERROR_FAIL;

        if (n32_write32(target, addr, word) != ERROR_OK)
            return ERROR_FAIL;

        if (n32_wait_ready(target, base, 0x00020000U) != ERROR_OK)
            return ERROR_FAIL;

        /* read-back verification */
        uint32_t verify = 0;
        if (n32_read32(target, addr, &verify) != ERROR_OK)
            return ERROR_FAIL;
        if (verify != word)
            return ERROR_FAIL;

        if (n32_read32(target, base + N32_FLASH_CTRL_OFF, &ctrl) != ERROR_OK)
            return ERROR_FAIL;
        ctrl &= ~FLASH_CTRL_PG;
        if (n32_write32(target, base + N32_FLASH_CTRL_OFF, ctrl) != ERROR_OK)
            return ERROR_FAIL;
    }

    if (i + 2 == count) {
        uint16_t half = buffer[i] | (buffer[i + 1] << 8);

        if (n32_clear_flags(target, base) != ERROR_OK)
            return ERROR_FAIL;
        if (n32_wait_ready(target, base, 0x00020000U) != ERROR_OK)
            return ERROR_FAIL;

        uint32_t ctrl = 0;
        if (n32_read32(target, base + N32_FLASH_CTRL_OFF, &ctrl) != ERROR_OK)
            return ERROR_FAIL;
        ctrl |= FLASH_CTRL_PG;
        if (n32_write32(target, base + N32_FLASH_CTRL_OFF, ctrl) != ERROR_OK)
            return ERROR_FAIL;

        if (n32_write16(target, addr, half) != ERROR_OK)
            return ERROR_FAIL;

        if (n32_wait_ready(target, base, 0x00020000U) != ERROR_OK)
            return ERROR_FAIL;

        uint16_t v16 = 0;
        if (n32_read16(target, addr, &v16) != ERROR_OK)
            return ERROR_FAIL;
        if (v16 != half)
            return ERROR_FAIL;

        if (n32_read32(target, base + N32_FLASH_CTRL_OFF, &ctrl) != ERROR_OK)
            return ERROR_FAIL;
        ctrl &= ~FLASH_CTRL_PG;
        if (n32_write32(target, base + N32_FLASH_CTRL_OFF, ctrl) != ERROR_OK)
            return ERROR_FAIL;
    }

    /* lock */
    uint32_t ctrl = 0;
    if (n32_read32(target, base + N32_FLASH_CTRL_OFF, &ctrl) == ERROR_OK) {
        ctrl |= FLASH_CTRL_LOCK;
        n32_write32(target, base + N32_FLASH_CTRL_OFF, ctrl);
    }

    return ERROR_OK;
}

static int n32g03x_protect_check(struct flash_bank *bank)
{
    struct target *target = bank->target;
    uint16_t wrp0_hw = 0, wrp1_hw = 0, wrp2_hw = 0, wrp3_hw = 0;
    if (n32_read_opt16(target, N32_OB_WRP0, &wrp0_hw) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_read_opt16(target, N32_OB_WRP1, &wrp1_hw) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_read_opt16(target, N32_OB_WRP2, &wrp2_hw) != ERROR_OK)
        return ERROR_FAIL;
    if (n32_read_opt16(target, N32_OB_WRP3, &wrp3_hw) != ERROR_OK)
        return ERROR_FAIL;

    uint8_t wrp0 = (uint8_t)(wrp0_hw & 0xFFU);
    uint8_t wrp1 = (uint8_t)(wrp1_hw & 0xFFU);
    uint8_t wrp2 = (uint8_t)(wrp2_hw & 0xFFU);
    uint8_t wrp3 = (uint8_t)(wrp3_hw & 0xFFU);

    unsigned int prot_bits = 32U;
    unsigned int group_size = (bank->num_sectors + prot_bits - 1U) / prot_bits;

    for (unsigned int s = 0; s < bank->num_sectors; ++s) {
        unsigned int bit = s / group_size;
        unsigned int p = 1U;
        if (bit < 8U)
            p = (wrp0 >> bit) & 1U;
        else if (bit < 16U)
            p = (wrp1 >> (bit - 8U)) & 1U;
        else if (bit < 24U)
            p = (wrp2 >> (bit - 16U)) & 1U;
        else if (bit < 32U)
            p = (wrp3 >> (bit - 24U)) & 1U;
        bank->sectors[s].is_protected = (p == 0U) ? 1 : 0;
    }
    return ERROR_OK;
}

static int n32g03x_erase_check(struct flash_bank *bank)
{
    return default_flash_blank_check(bank);
}

static int n32g03x_info(struct flash_bank *bank, struct command_invocation *cmd)
{
    struct n32g03x_flash *info = bank->driver_priv;
    if (!info)
        return ERROR_FAIL;
    command_print_sameline(cmd, "Nations N32G03x internal flash");
    command_print_sameline(cmd, ", size=%u bytes, page=%u bytes", bank->size,
                           info->page_size ? info->page_size : N32_DEFAULT_PAGE_SZ);
    return ERROR_OK;
}

static void n32g03x_free_priv(struct flash_bank *bank)
{
    free(bank->driver_priv);
    bank->driver_priv = NULL;
}

const struct flash_driver n32g03x_flash = {
    .name = "n32g03x",
    .commands = NULL,
    .flash_bank_command = n32g03x_flash_bank_command,
    .erase = n32g03x_erase,
    .protect = n32g03x_protect,
    .write = n32g03x_write,
    .read = default_flash_read,
    .probe = n32g03x_probe,
    .auto_probe = n32g03x_auto_probe,
    .erase_check = n32g03x_erase_check,
    .protect_check = n32g03x_protect_check,
    .info = n32g03x_info,
    .free_driver_priv = n32g03x_free_priv,
};