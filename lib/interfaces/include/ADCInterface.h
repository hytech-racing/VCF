#ifndef ADC_INTERFACE_H
#define ADC_INTERFACE_H


/// @brief This file defines ADC interface - which should replace the handling of ADC code in the VCF repo. The actual implementation is currently in VCF_Tasks and we want to move it here.



#include <Arduino.h>
#include "SharedFirmwareTypes.h"
#include "etl/singleton.h"


class ADCInterface
{
    public:
        ADCInterface() = default; //default constructor for now

}

#endif /* ADC_INTERFACE_H */


