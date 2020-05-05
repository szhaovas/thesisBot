#ifndef _PROPRIOCEPTIVE_SENSOR_CPP
#define _PROPRIOCEPTIVE_SENSOR_CPP

#include "iostream"

#include "proprioceptiveSensor.h"

#include "neuron.h"

extern int HINGE;
extern int SLIDER;
extern int THRUSTER;

PROPRIOCEPTIVE_SENSOR::PROPRIOCEPTIVE_SENSOR(int myID, int evalPeriod, bool locheck, double lo) {

	ID = myID;

	angles = new double[evalPeriod];

    mySensorNeuron = NULL;

	this->locheck = locheck;

	this->lo = lo;
}

PROPRIOCEPTIVE_SENSOR::~PROPRIOCEPTIVE_SENSOR(void) {

}

void PROPRIOCEPTIVE_SENSOR::Connect_To_Sensor_Neuron(NEURON *sensorNeuron) {

        mySensorNeuron = sensorNeuron;
}

int  PROPRIOCEPTIVE_SENSOR::Get_ID(void) {

        return ID;
}

void PROPRIOCEPTIVE_SENSOR::Poll(dJointID joint, int type, int t) {

        const dReal *pos;

        if(type==HINGE) {
			double angle = dJointGetHingeAngle(joint);
			if(locheck)
			{
				if (angle <= lo)
				{
					angles[t] = 1;
				} else
				{
					angles[t] = 0;
				}
			} else
			{
				angles[t] = angle;
			}
		}
        else if(type==SLIDER) {
			double position = dJointGetSliderPosition(joint);
			if(locheck)
			{
				if (position <= lo)
				{
					angles[t] = 1;
				} else
				{
					angles[t] = 0;
				}
			} else
			{
				angles[t] = position;
			}
		}
}

void PROPRIOCEPTIVE_SENSOR::Update_Sensor_Neurons(int t) {

        if ( mySensorNeuron )
                mySensorNeuron->Set( angles[t] );
}

void PROPRIOCEPTIVE_SENSOR::Write_To_Python(int evalPeriod) {

        char outString[1000000];

        sprintf(outString,"%d %d ",ID,1);

        for ( int  t = 0 ; t < evalPeriod ; t++ )

                sprintf(outString,"%s %f ",outString,angles[t]);

        sprintf(outString,"%s \n",outString);

        std::cout << outString;
}

#endif
