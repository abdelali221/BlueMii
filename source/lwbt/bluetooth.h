#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

#include "l2cap.h"

#ifdef __cplusplus
	extern "C" {
#endif /* __cplusplus */

typedef s32 (*btecallback)(s32 result,void *userdata);

void BT_Init(btecallback cb, struct l2cap_pcb **pcbptr);
void BT_Shutdown(void);

#ifdef __cplusplus
	}
#endif /* __cplusplus */

#endif

