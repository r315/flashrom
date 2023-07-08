#include <stddef.h>
#include <stdint.h>
#include "stimer.h"

static simpletimer_t *tlist = NULL;
static volatile uint32_t last_tic;

void STIMER_Config(simpletimer_t *timer, uint32_t interval, uint32_t (*callback)(simpletimer_t *timer))
{
    if(interval == 0 || timer == NULL || callback == NULL){
        return;
    }

    timer->interval = interval;
    timer->callback = callback;
    timer->countdown = 0;
    timer->next = NULL;

    if(tlist == NULL){
        tlist = timer;
        return;
    }

    simpletimer_t *head = tlist;

    while(head->next != NULL){
        head = head->next;
    }

    head->next = timer;
}

void STIMER_Remove(simpletimer_t *timer)
{
    #if 0
    simpletimer_t *head = tlist;

    while(head){
        if(head == timer){

        }        
        head = head->next;
    }
    #endif
}

void STIMER_Start(simpletimer_t *timer)
{
    timer->countdown = timer->interval;
}

void STIMER_Stop(simpletimer_t *timer)
{
    timer->countdown = 0;
}

void STIMER_Handler(uint32_t tic)
{
    simpletimer_t *head = tlist;

    while(head){
        if(head->countdown > 0){
            if((--head->countdown) == 0){
                head->countdown = head->callback(head);
            }
        }
        head = head->next;
    }

    last_tic = tic;
}