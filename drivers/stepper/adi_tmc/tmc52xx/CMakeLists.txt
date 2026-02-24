# SPDX-FileCopyrightText: Copyright (c) 2025 Dipak Shetty
# SPDX-FileCopyrightText: Copyright (c) 2026 A. Schweigstill
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_STEPPER_ADI_TMC52XX tmc52xx.c)
zephyr_library_sources_ifdef(CONFIG_STEPPER_ADI_TMC52XX_STEPPER_CTRL tmc52xx_stepper_ctrl.c)
zephyr_library_sources_ifdef(CONFIG_STEPPER_ADI_TMC52XX_STEPPER_DRIVER tmc52xx_stepper_driver.c)
