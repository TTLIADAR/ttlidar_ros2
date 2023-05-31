/*
*  TTldiar System
*  Driver Interface
*
*  Copyright 2021 TT Team
*  All rights reserved.
*
*	Author: Titans  2021-2-21
*
*/


#include "arch/linux/arch_linux.h"

#include <sched.h>

namespace ttlidar {

Thread Thread::create(thread_proc_t proc, void * data)
{
    Thread newborn(proc, data);
    

    assert( sizeof(newborn._handle) >= sizeof(pthread_t));

    pthread_create((pthread_t *)&newborn._handle, NULL, (void * (*)(void *))proc, data);

    return newborn;
}

result_t Thread::terminate()
{
    if (!this->_handle) return RESULT_OK;
    
    return pthread_cancel((pthread_t)this->_handle)==0?RESULT_OK:RESULT_OPERATION_FAIL;
}

result_t Thread::setPriority( priority_level_t p)
{
    if (!this->_handle) return RESULT_OPERATION_FAIL;
    
    
    
    int current_policy;
    struct sched_param current_param;
    int ans;
    if (pthread_getschedparam( (pthread_t) this->_handle, &current_policy, &current_param))
    {
        
        return RESULT_OPERATION_FAIL;
    }   
    //set  pthread_priority
    //int pthread_priority = 0 ;

    switch(p)
    {
    case PRIORITY_REALTIME:      
        current_policy = SCHED_RR;
        break;
    case PRIORITY_HIGH:
        //pthread_priority = (pthread_priority_max + pthread_priority_min)/2;
        current_policy = SCHED_RR;
        break;
    case PRIORITY_NORMAL:
    case PRIORITY_LOW:
    case PRIORITY_IDLE:
        //pthread_priority = 0;
        current_policy = SCHED_OTHER;
        break;
    }

    current_param.__sched_priority = current_policy;
    if ( (ans = pthread_setschedparam( (pthread_t) this->_handle, current_policy, &current_param)) )
    {
        return RESULT_OPERATION_FAIL;
    }
    return  RESULT_OK;
}

Thread::priority_level_t Thread::getPriority()
{
    if (!this->_handle) return PRIORITY_NORMAL;

    int current_policy;
    struct sched_param current_param;
    if (pthread_getschedparam( (pthread_t) this->_handle, &current_policy, &current_param))
    {
        // cannot retreieve values
        return PRIORITY_NORMAL;
    }   

    int pthread_priority_max = sched_get_priority_max(SCHED_RR);
    int pthread_priority_min = sched_get_priority_min(SCHED_RR);

    if (current_param.__sched_priority ==(pthread_priority_max ))
    {
        return PRIORITY_REALTIME;
    }
    if (current_param.__sched_priority >=(pthread_priority_max + pthread_priority_min)/2)
    {
        return PRIORITY_HIGH;
    }
    return PRIORITY_NORMAL;
}

result_t Thread::join(unsigned long timeout)
{
    if (!this->_handle) return RESULT_OK;
    
    pthread_join((pthread_t)(this->_handle), NULL);
    return RESULT_OK;
}

}
