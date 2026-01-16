# SPDX-FileCopyrightText: Copyright (c) 2025 Dipak Shetty
# SPDX-FileCopyrightText: Copyright (c) 2026 A. Schweigstill
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_STEPPER_ADI_TMC52XX tmc52xx.c)
zephyr_library_sources_ifdef(CONFIG_STEPPER_ADI_TMC52XX_STEPPER tmc52xx_stepper.c)
zephyr_library_sources_ifdef(CONFIG_STEPPER_ADI_TMC52XX_STEPPER_DRV tmc52xx_stepper_drv.c)
