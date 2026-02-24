#include "mbed.h"

class Potentiometer {
    private:
        AnalogIn inputSignal; // Declaration of AnalogIn object
        float VDD, currentSampleNorm,
            currentSampleVolts; // Float variable to speficy the value of VDD (3.3 V
                                // for the Nucleo-64)

    public:
        Potentiometer(PinName pin, float v): inputSignal(pin), VDD(v) {} // Constructor - user provided pin name assigned to AnalogIn...
            // VDD is also provided to determine maximum measurable voltage
        float amplitudeVolts(void) // Public member function to measure the amplitude in volts
        {
            return (inputSignal.read() *
                    VDD); // Scales the 0.0-1.0 value by VDD to read the input in volts
        }

        float amplitudeNorm(void) // Public member function to measure the normalised amplitude
        {
            return inputSignal
                .read(); // Returns the ADC value normalised to range 0.0 - 1.0
        }

        void sample(void) // Public member function to read a sample and store the
                            // value as data members
        {
            currentSampleNorm =
                inputSignal.read(); // Read a sample from the ADC and store normalised
                                    // representation [0..1]
            currentSampleVolts =
                currentSampleNorm *
                VDD; // Convert this to a voltage and store that as a data member too.
        }

        const float getCurrentSampleNorm(void) // Public member function to return the
                                                // most recent normalised sample [0..1]
        {
            return currentSampleNorm; // Return the most recent normalised sample
        }

        const float
        getCurrentSampleVolts(void) // Public member function to return the most
                                    // recent sampled voltage [0.. 3.3 V]
        {
            return currentSampleVolts; // Return the most recent sampled voltage
        }
};