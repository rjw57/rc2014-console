# TV output console for RC2014

**THIS PROJECT IS INCOMPLETE AND EXPERIMENTAL.**

A video console for the [RC2014 Z80 computer](https://rc2014.co.uk/) which
generates a black and white 525-line PAL signal.

## Overview

This project is an experiment in writing a video terminal using two Ardunio Nano
boards. Two are required since one board is fully utilised generating the 40x32
character display.

Video output is via the ATMega328's USART in SPI mode at 8MHz. This is slightly
faster than the "true" TV dot clock but is close enough in most cases.

A second Arduino consumes serial data from the RC2014 and converts it to SPI
data sent to the console Arduino.

## Status

This project is very much an experiment and is under-documented.

## Documentation

Sections of the project are documented separately:

* [Hardware KiCAD project](hw/) including the
  [hardware schematic](hw/plots/hw.pdf)
* [Console driver](tvout/)
* [Serial driver](serial-to-spi/)
