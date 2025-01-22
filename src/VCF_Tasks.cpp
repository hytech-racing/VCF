#include "VCF_Tasks.h"



bool init_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    return true;
}
bool run_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    return true;
}
HT_TASK::Task read_adc1_task = HT_TASK::Task(init_read_adc1_task, run_read_adc1_task, 10, 1000UL); // 1000us is 1kHz //NOLINT



bool init_read_adc2_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    return true;
}
bool run_read_adc2_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    return true;
}
HT_TASK::Task read_adc2_task = HT_TASK::Task(init_read_adc2_task, run_read_adc2_task, 10, 1000UL); // 1000us is 1kHz //NOLINT
