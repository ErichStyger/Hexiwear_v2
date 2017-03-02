#ifndef __QUIZZ_H
#define __QUIZZ_H

#include "Platform.h"
#if PL_CONFIG_HAS_QUIZZ

void QUIZZ_CreateTask(void);

void QUIZZ_SuspendTask(void);

void QUIZZ_ActivateTask(void);

void QUIZZ_KillTask(void);

void QUIZZ_Init(void);

#endif /* PL_CONFIG_HAS_QUIZZ */

#endif /* __QUIZZ_H */

