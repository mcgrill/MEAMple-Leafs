// -----------------------------------------------------------------------------
// M2 data bus subsystem
// version: 2.0
// date: June 30, 2011
// author: J. Fiene
// -----------------------------------------------------------------------------

#ifndef m_bus__
#define m_bus__

#include "m_general.h"
#include <util/twi.h>

// -----------------------------------------------------------------------------
// Public functions:
// -----------------------------------------------------------------------------

void m_bus_init(void);
// FUNCTIONALITY:
// initialize the M2 data bus, which uses pins D0-D2 and is available through the
// 5-pin end header.  When new data is available from a slave, the INT2_vect interrupt
// will be triggered, and you must act accordingly!
//
// TAKES:
// nothing
//
// RETURNS:
// nothing

#endif