#ifndef GUI_H
#define GUI_H 103

/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "common.h"

void guiInit( );
//void guiUpdate(struct State s);
void guiUpdate( );

#ifdef __cplusplus
}
#endif

#endif
