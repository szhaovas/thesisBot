#ifndef _PROPRIOCEPTIVE_SENSOR_H
#define _PROPRIOCEPTIVE_SENSOR_H

#include <ode/ode.h>

class NEURON;

class PROPRIOCEPTIVE_SENSOR {

private:

	int ID;

	double *angles;

    NEURON *mySensorNeuron;

	bool locheck;

	double lo;

public:
	PROPRIOCEPTIVE_SENSOR(int myID, int evalPeriod, bool locheck, double lo);

	~PROPRIOCEPTIVE_SENSOR(void);

        void Connect_To_Sensor_Neuron(NEURON *sensorNeuron);

        int  Get_ID(void);

	void Poll(dJointID joint, int type, int t);

        void Update_Sensor_Neurons(int t);

	void Write_To_Python(int evalPeriod);
};

#endif
