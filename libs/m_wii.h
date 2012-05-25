// -----------------------------------------------------------------------------
// M2 Wiimote Pixart Sensor interface
// version: 1.1
// date: Nov 11, 2011
// author: J. Fiene
// -----------------------------------------------------------------------------

#ifndef m_wii__
#define m_wii__

#include "m_general.h"
#include "m_bus.h"

// -----------------------------------------------------------------------------
// Public functions:
// -----------------------------------------------------------------------------

char m_wii_open(void);
// FUNCTIONALITY:
// configure the mWii Pixart sensor.
// (note:  the mWii does not generate interrupts)
//
// TAKES:
// nothing
//
// RETURNS:
// 1 : success
// 0 : failure


char m_wii_read(unsigned int* blobs);
// FUNCTIONALITY:
// read blob data
//
// TAKES:
// blobs : pointer to a 12-element unsigned int buffer
//
// RETURNS:
// 1 : success
// 0 : failure

#endif