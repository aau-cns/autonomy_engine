// Copyright (C) 2021 Alessandro Fornasier,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <alessandro.fornasier@ieee.org>

#ifndef _COLORS_
#define _COLORS_

#define RESET "\033[0m"
#define BLACK_ESCAPE "\033[30m"
#define RED_ESCAPE "\033[31m"
#define GREEN_ESCAPE "\033[32m"
#define YELLOW_ESCAPE "\033[33m"
#define BLUE_ESCAPE "\033[34m"
#define MAGENTA_ESCAPE "\033[35m"
#define CYAN_ESCAPE "\033[36m"
#define WHITE_ESCAPE "\033[37m"
#define REDPURPLE_ESCAPE "\033[95m"
#define BOLD_ESCAPE "\x1B[1m"
#define UNDERLINE_ESCAPE "\x1B[4m"

#define BLACK(x) BLACK_ESCAPE x RESET
#define RED(x) RED_ESCAPE x RESET
#define GREEN(x) GREEN_ESCAPE x RESET
#define YELLOW(x) YELLOW_ESCAPE x RESET
#define BLUE(x) BLUE_ESCAPE x RESET
#define MAGENTA(x) MAGENTA_ESCAPE x RESET
#define CYAN(x) CYAN_ESCAPE x RESET
#define WHITE(x) WHITE_ESCAPE x RESET
#define REDPURPLE(x) REDPURPLE_ESCAPE x RESET

#define BOLD(x) BOLD_ESCAPE x RESET
#define UNDERLINE(x) UNDERLINE_ESCAPE x RESET

#define ESCAPE(x, y) x y

#endif /* _COLORS_ */
