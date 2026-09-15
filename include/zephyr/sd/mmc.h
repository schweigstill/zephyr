/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public API for MMC memory card subsystem
 */

#ifndef ZEPHYR_INCLUDE_SD_MMC_H_
#define ZEPHYR_INCLUDE_SD_MMC_H_

#include <zephyr/device.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/sd/sd.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Write blocks to MMC card from buffer
 *
 * Writes blocks from MMC buffer to MMC card. For best performance, this buffer
 * should be aligned to CONFIG_SDHC_BUFFER_ALIGNMENT
 * @param card MMC card to write from
 * @param wbuf write buffer
 * @param start_block first block to write to
 * @param num_blocks number of blocks to write
 * @retval 0 write succeeded
 * @retval -EBUSY: card is busy with another request
 * @retval -ETIMEDOUT: card write timed out
 * @retval -EIO: I/O error
 */
int mmc_write_blocks(struct sd_card *card, const uint8_t *wbuf,
	uint32_t start_block, uint32_t num_blocks);

/**
 * @brief Read block from MMC card to buffer
 *
 * Reads blocks into MMC buffer from MMC card. For best performance, this buffer
 * should be aligned to CONFIG_SDHC_BUFFER_ALIGNMENT
 * @param card MMC card to read from
 * @param rbuf read buffer
 * @param start_block first block to read from
 * @param num_blocks number of blocks to read
 * @retval 0 read succeeded
 * @retval -EBUSY: card is busy with another request
 * @retval -ETIMEDOUT: card read timed out
 * @retval -EIO: I/O error
 */
int mmc_read_blocks(struct sd_card *card, uint8_t *rbuf,
	uint32_t start_block, uint32_t num_blocks);

/**
 * @brief Erase complete MMC erase groups and release them to the device.
 *
 * Uses CMD35/CMD36/CMD38 (normal ERASE), waits for completion and checks status.
 * This is logical erasure to the device's erased value, not a sanitization of
 * inaccessible NAND copies, boot partitions or RPMB.
 *
 * @param card Initialized MMC card in the user data partition.
 * @param start_block First 512-byte sector, aligned to an erase group.
 * @param num_blocks Nonzero sector count, aligned to an erase group.
 * @retval 0 All requested groups erased.
 * @retval -EINVAL Range is empty, unaligned or outside the device.
 * @retval -ENOTSUP High-capacity erase geometry unavailable.
 * @retval -EBUSY Card lock unavailable.
 * @retval -ETIMEDOUT Erase did not finish within the device timeout.
 * @retval -EIO Card reported an error.
 * @return Other negative error code from the host controller.
 */
int mmc_erase_blocks(struct sd_card *card, uint32_t start_block, uint32_t num_blocks);

/**
 * @brief Get I/O control data from MMC card
 *
 * Sends I/O control commands to MMC card.
 * @param card MMC card
 * @param cmd I/O control command
 *	Mirrors disk subsystem,
 *	see include/zephyr/drivers/disk.h for list of possible commands.
 * @param buf I/O control buf
 * @retval 0 IOCTL command succeeded
 * @retval -ENOTSUP: IOCTL command not supported
 * @retval -EIO: I/O failure
 */
int mmc_ioctl(struct sd_card *card, uint8_t cmd, void *buf);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SD_MMC_H_ */
