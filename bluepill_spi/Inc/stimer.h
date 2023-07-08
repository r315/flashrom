#ifndef _TIMER_H_
#define _TIMER_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

typedef struct simpletimer_s {
    uint32_t interval;
    uint32_t countdown;
    uint32_t (*callback)(struct simpletimer_s *timer);
    struct simpletimer_s *next;
}simpletimer_t;


#define STIMER_MAX_TIMERS   10

void STIMER_Config(simpletimer_t *timer, uint32_t interval, uint32_t (*callback)(simpletimer_t *timer));
void STIMER_Remove(simpletimer_t *timer);
void STIMER_Start(simpletimer_t *timer);
void STIMER_Stop(simpletimer_t *timer);
void STIMER_Handler(uint32_t stick);

#ifdef __cplusplus
}
#endif

#endif