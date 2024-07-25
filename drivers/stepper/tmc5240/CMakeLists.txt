# SPDX-FileCopyrightText: Copyright (c) 2024 Nathan Perry <np@npry.dev>
# SPDX-License-Identifier: MIT OR Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_TMC5240_STEPPER tmc5240.c stepper_api.c)
