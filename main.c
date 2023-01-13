/************************************************************************
  main.c

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
#include <stdlib.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"

#define PIO_USB_DP_PIN_DEFAULT 8
#define VERSION "0.1"

#include "pio_usb.h"
#include "tusb.h"
#include "main.h"

#include "hardware/uart.h"
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define UART_TX_PIN 12
#define UART_RX_PIN 13

#define MOUSEX 0
#define MOUSEY 1

// Quadrature output buffer limit
//
// Since the slow rate limit will prevent the quadrature output keeping up with the USB movement
// report input, the quadrature output will lag behind (i.e. the quadrature mouse will continue
// to move after the USB mouse has stopped).  This setting limits the maximum number of buffered
// movements to the quadrature output.  If the buffer reaches this value further USB movements
// will be discarded
#define Q_BUFFERLIMIT 300

// Interrupt Service Routines for quadrature output -------------------------------------------------------------------

// The following globals are used by the interrupt service routines to track the mouse
// movement.  The mouseDirection indicates which direction the mouse is moving in and
// the mouseEncoderPhase tracks the current phase of the quadrature output.
//
// The mouseDistance variable tracks the current distance the mouse has left to move
// (this is incremented by the USB mouse reports and decremented by the ISRs as they
// output the quadrature to the retro host).
volatile int8_t mouseDirectionX = 0;    // X direction (0 = decrement, 1 = increment)
volatile int8_t mouseEncoderPhaseX = 0; // X Quadrature phase (0-3)

volatile int8_t mouseDirectionY = 0;    // Y direction (0 = decrement, 1 = increment)
volatile int8_t mouseEncoderPhaseY = 0; // Y Quadrature phase (0-3)

volatile int16_t mouseDistanceX = 0; // Distance left for mouse to move
volatile int16_t mouseDistanceY = 0; // Distance left for mouse to move
volatile bool mouseConnected = 0;

void gpio_set_mode_open_drain(uint gpio)
{
  // enable output on gpio (inverting)
  gpio_set_oeover(gpio, 0x1);
  // set output signal for gpio: drive output low
  gpio_set_outover(gpio, 0x2);
}

void core1_main()
{
  sleep_ms(10);

  // Use tuh_configure() to pass pio configuration to the host stack
  // Note: tuh_configure() must be called before
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  tuh_configure(1, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);

  // To run USB SOF interrupt in core1, init host stack for pio_usb (roothub
  // port1) on core1
  tuh_init(1);

  while (true)
  {
    tuh_task(); // tinyusb host task
  }
}

int main()
{
  // default 125MHz is not appropreate. Sysclock should be multiple of 12MHz.
  set_sys_clock_khz(120000, true);
  stdio_init_all();

  // Setup Debug to UART
  uart_init(UART_ID, 2400);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);
  uart_set_hw_flow(UART_ID, false, false);
  uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
  printf("\033[2J");
  printf("****************************************************\r\n");
  printf("*         RP2040 USB To Quadrature Adapter         *\r\n");
  printf("*         Copyright 2022 Darren Jones              *\r\n");
  printf("*         (nz.darren.jones@gmail.com)              *\r\n");
  printf("*         Version: %s                             *\r\n", VERSION);
  printf("*         Build Date: %s %s         *\r\n", __DATE__, __TIME__);
  printf("****************************************************\r\n");
  printf("\r\n");
  printf("RP2040 USB To Quadrature Booting.....\r\n");

  // all USB task run in core1
  multicore_reset_core1();
  printf("Core1 Reset\r\n");

  multicore_launch_core1(core1_main);
  printf("Core1 Launched\r\n");

  // Initialise the RP2040 hardware
  initialiseHardware();
  printf("Hardware Initalized\r\n");

  // Blink Status LED and wait for everything to settle
  int8_t i = 0;
  while (i < 10)
  {
    gpio_put(STATUS_PIN, 1);
    sleep_ms(200);
    gpio_put(STATUS_PIN, 0);
    sleep_ms(200);
    i++;
  }

  // Initialise the timers
  initialiseTimers();
  printf("Timers Running\r\n");
  while (true)
  {
    stdio_flush();
    sleep_us(10);
  }
}

void initialiseHardware(void)
{
  // Initalize the pins
  gpio_init(RB_PIN);
  gpio_init(MB_PIN);
  gpio_init(LB_PIN);
  gpio_init(XA_PIN);
  gpio_init(XB_PIN);
  gpio_init(YB_PIN);
  gpio_init(YB_PIN);
  gpio_init(STATUS_PIN);

  // Set pin directions
  gpio_set_dir(RB_PIN, GPIO_OUT);
  gpio_set_dir(MB_PIN, GPIO_OUT);
  gpio_set_dir(LB_PIN, GPIO_OUT);
  gpio_set_dir(XA_PIN, GPIO_OUT);
  gpio_set_dir(XB_PIN, GPIO_OUT);
  gpio_set_dir(YA_PIN, GPIO_OUT);
  gpio_set_dir(YB_PIN, GPIO_OUT);
  gpio_set_dir(STATUS_PIN, GPIO_OUT);

  // Set the pins low
  gpio_put(RB_PIN, 1);
  gpio_put(MB_PIN, 1);
  gpio_put(LB_PIN, 1);
  gpio_put(XA_PIN, 0);
  gpio_put(XB_PIN, 0);
  gpio_put(YA_PIN, 0);
  gpio_put(YB_PIN, 0);
  gpio_put(STATUS_PIN, 0);

  // Set buttons to be open drain
  // gpio_set_mode_open_drain(RB_PIN);
  // gpio_set_mode_open_drain(MB_PIN);
  // gpio_set_mode_open_drain(LB_PIN);
}

bool timer1_callback(struct repeating_timer *t)
{
  if (mouseConnected == 1)
  {
    // Silence compilation warning
    (void)t;
    // Process X output
    if (mouseDistanceX > 0)
    {
      // Change phase and range check
      if (mouseDirectionX == 0)
      {
        mouseEncoderPhaseX--;
        if (mouseEncoderPhaseX < 0)
          mouseEncoderPhaseX = 3;
      }
      else
      {
        mouseEncoderPhaseX++;
        if (mouseEncoderPhaseX > 3)
          mouseEncoderPhaseX = 0;
      }

      // Set the output pins according to the current phase
      if (mouseEncoderPhaseX == 0)
        gpio_put(XA_PIN, 1); // Set X1 to 1
      if (mouseEncoderPhaseX == 1)
        gpio_put(XB_PIN, 1); // Set X2 to 1
      if (mouseEncoderPhaseX == 2)
        gpio_put(XA_PIN, 0); // Set X1 to 0
      if (mouseEncoderPhaseX == 3)
        gpio_put(XB_PIN, 0); // Set X2 to 0

      // Decrement the distance left to move
      mouseDistanceX--;
    }
    printf(".");
  }
  return true;
}

// Interrupt Service Routine based on Timer2 for mouse Y movement quadrature output
bool timer2_callback(struct repeating_timer *t)
{
  if (mouseConnected == 1)
  {
    // Silence compilation warning
    (void)t;
    // Process Y output
    if (mouseDistanceY > 0)
    {
      // Change phase and range check
      if (mouseDirectionY == 0)
      {
        mouseEncoderPhaseY--;
        if (mouseEncoderPhaseY < 0)
          mouseEncoderPhaseY = 3;
      }
      else
      {
        mouseEncoderPhaseY++;
        if (mouseEncoderPhaseY > 3)
          mouseEncoderPhaseY = 0;
      }

      // Set the output pins according to the current phase
      if (mouseEncoderPhaseY == 3)
        gpio_put(YA_PIN, 0); // Set Y1 to 0
      if (mouseEncoderPhaseY == 2)
        gpio_put(YB_PIN, 0); // Set Y2 to 0
      if (mouseEncoderPhaseY == 1)
        gpio_put(YA_PIN, 1); // Set Y1 to 1
      if (mouseEncoderPhaseY == 0)
        gpio_put(YB_PIN, 1); // Set Y2 to 1

      // Decrement the distance left to move
      mouseDistanceY--;
    }
    printf("+");
  }
  return true;
}

// Initialise the ISR timers
void initialiseTimers(void)
{
  // Set timers to 260us for now which is ~3.9kHz
  struct repeating_timer timer1;
  struct repeating_timer timer2;
  add_repeating_timer_us(-260, timer1_callback, NULL, &timer1);
  add_repeating_timer_us(-260, timer2_callback, NULL, &timer2);
}
//--------------------------------------------------------------------+
// Host HID
//--------------------------------------------------------------------+

// Invoked when device with hid interface is mounted
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len)
{
  (void)desc_report;
  (void)desc_len;
  printf("Device Attached\r\n");

  gpio_put(STATUS_PIN, 1); // Turn status LED on
  // Interface protocol (hid_interface_protocol_enum_t)
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

  char output[255];
  sprintf(output, "Protocol: %d\r\n", itf_protocol);
  printf(output);

  // Receive report from boot mouse only
  // tuh_hid_report_received_cb() will be invoked when report is available
  if (itf_protocol == HID_ITF_PROTOCOL_MOUSE)
  {
    if (tuh_hid_receive_report(dev_addr, instance))
    {
      mouseConnected = 1;
      gpio_put(STATUS_PIN, 1); // Turn status LED on
    }
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
  (void)dev_addr;
  (void)instance;
  mouseConnected = 0;
  printf("Device Removed\r\n");
  gpio_put(STATUS_PIN, 0); // Turn status LED off
}

static void processMouse(uint8_t dev_addr, hid_mouse_report_t const *report)
{
  // Blink status LED
  // gpio_put(STATUS_PIN, 0);
  (void)dev_addr;
  printf("%02x\r\n", mouseConnected);
  // Handle scroll wheel
  if (report->wheel)
  {
    gpio_put(MB_PIN, 1);
    printf("Wheel %02x\r\n", report->wheel);
    processMouseMovement(report->wheel, MOUSEY);
    gpio_put(MB_PIN, 0);
  }

  // Handle mouse buttons
  // Check for left mouse button
  if (report->buttons & MOUSE_BUTTON_LEFT)
  {
    gpio_put(LB_PIN, 0);
    printf("Left Click\r\n");
  }
  else
  {
    gpio_put(LB_PIN, 1);
    printf("Left Unclick\r\n");
  }

  // Check for middle mouse button
  if (report->buttons & MOUSE_BUTTON_MIDDLE)
  {
    gpio_put(MB_PIN, 0);
    printf("Middle Click\r\n");
  }
  else
  {
    gpio_put(MB_PIN, 1);
    printf("Middle Unclick\r\n");
  }

  // Check for right mouse button
  if (report->buttons & MOUSE_BUTTON_RIGHT)
  {
    gpio_put(RB_PIN, 0);
    printf("Right Click\r\n");
  }
  else
  {
    gpio_put(RB_PIN, 1);
    printf("Right Unclick\r\n");
  }

  // Handle mouse movement
  if (report->x > 0 && mouseDirectionX == 0)
  {
    mouseDistanceX = 0;
    mouseDirectionX = 1;
    printf("X++\r\n");
  }
  else if (report->x < 0 && mouseDirectionX == 1)
  {
    mouseDistanceX = 0;
    mouseDirectionX = 0;
    printf("X--\r\n");
  }

  if (report->y > 0 && mouseDirectionY == 0)
  {
    mouseDistanceY = 0;
    mouseDirectionY = 1;
    printf("Y++\r\n");
  }
  else if (report->y < 0 && mouseDirectionY == 1)
  {
    mouseDistanceY = 0;
    mouseDirectionY = 0;
    printf("Y--\r\n");
  }

  // Process mouse X and Y movement
  processMouseMovement(report->x, MOUSEX);
  processMouseMovement(report->y, MOUSEY);
  // Blink status LED
  // gpio_put(STATUS_PIN, 1);
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len)
{
  (void)len;
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

  switch (itf_protocol)
  {
  case HID_ITF_PROTOCOL_MOUSE:
    processMouse(dev_addr, (hid_mouse_report_t const *)report);
    break;

  default:
    break;
  }

  // continue to request to receive report
  if (!tuh_hid_receive_report(dev_addr, instance))
  {
    return;
  }
}

// Process the mouse movement units from the USB report
void processMouseMovement(int8_t movementUnits, uint8_t axis)
{
  // Set the mouse movement direction and record the movement units
  if (movementUnits > 0)
  {
    // Moving in the positive direction
    // Add the movement units to the quadrature output buffer
    if (axis == MOUSEX)
      mouseDistanceX += movementUnits;
    else
      mouseDistanceY += movementUnits;
  }
  else if (movementUnits < 0)
  {
    // Moving in the negative direction
    // Add the movement units to the quadrature output buffer
    if (axis == MOUSEX)
      mouseDistanceX += -movementUnits;
    else
      mouseDistanceY += -movementUnits;
  }
  else
  {
    if (axis == MOUSEX)
      mouseDistanceX = 0;
    else
      mouseDistanceY = 0;
  }
  // Apply the quadrature output buffer limit
  if (axis == MOUSEX)
  {
    if (mouseDistanceX > Q_BUFFERLIMIT)
      mouseDistanceX = Q_BUFFERLIMIT;
  }
  else
  {
    if (mouseDistanceY > Q_BUFFERLIMIT)
      mouseDistanceY = Q_BUFFERLIMIT;
  }
}