.. zephyr:code-sample:: stepper-tmc5240
   :name: TMC5240 driver example
   :relevant-api: stepper_interface

   Spin a stepper motor driven by the TMC5240.

Overview
********

This example demonstrates control of a TMC5240 stepper motor using both the generic
stepper API and the TMC5240's device-specific API.


Requirements
************

The board must have a SPI bus with an `adi,tmc5240`-compatible child node.

