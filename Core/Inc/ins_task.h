#ifndef __INS_TASK_H__
#define __INS_TASK_H__

#include "Fusion.h"
#include "ICM42688P.h"

#define SAMPLE_RATE (100) // replace this with actual sample rate

void InitializePose(void);
void GetPose(ICM42688P_Data_t *imudata);

#endif /* __INS_TASK_H__ */
