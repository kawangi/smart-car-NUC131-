#include "SERVO_PID.h"

int P, In, Id, D, acc, last, L_last, target;

/*void PID(int p, int in, int id, int d) {
	P = p;
	In = in; Id = id;	// I break "I" into two parts to avoid decimal
	D = d;
	acc = last = 0;
}*/
void setPID(int p, int in, int id, int d) {
	P = p;
	In = in; Id = id;	// I break "I" into two parts to avoid decimal
	D = d;
	acc = last = L_last = 0;
}

void reset(void) {acc = last = 0;}

void setSpeed(int speed)
{
	target = speed;
}

int control(int x) {

	int error = x;
	int diff = x - last - L_last;
	L_last = last;
	last = x;

	acc += error;
	if (acc > 2000000000) acc = 2000000000;
	if (acc < -2000000000) acc = -2000000000;
	return - P * error - In * (acc >> Id) - D * diff;
}
