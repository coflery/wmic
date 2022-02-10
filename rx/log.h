/*  Copyright (C) 2020-2022 Frand Ren
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */

#ifndef _LOG_H
#define _LOG_H

#include <stdio.h>

#ifdef DEBUG
    #define DEBUG_PRINT printf
#else
    #define DEBUG_PRINT(...)
#endif
#define ERROR_PRINT(fmt, args...) printf("ERROR: "fmt, ## args)

#endif
