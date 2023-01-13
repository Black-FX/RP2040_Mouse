/************************************************************************
  main.h

  Main functions
    RP2040 - USB to quadrature mouse converter
    Copyright (C) 2023 Darren Jones
    Copyright (C) 2017-2020 Simon Inns

  This file is part of RP2040 Mouse based on the original SmallyMouse from Simon Inns.

    RP2040 Mouse is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Email: nz.darren.jones@gmail.com

************************************************************************/

#ifndef _MAIN_H_
#define _MAIN_H_

// Hardware map

// Quadrature mouse output pins
// Right mouse button
#define RB_PIN	6
// Middle mouse button
#define MB_PIN	5
// Left mouse button
#define LB_PIN	4
// Y axis output
#define YA_PIN	2
#define YB_PIN	3
// X axis output
#define XA_PIN	0
#define XB_PIN	1
// Mouse Status LED
#define STATUS_PIN	7

// Function prototypes
void initialiseHardware(void);
void initialiseTimers(void);

void processMouseMovement(int8_t movementUnits, uint8_t axis);

#endif