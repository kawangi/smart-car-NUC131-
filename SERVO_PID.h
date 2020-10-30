
#ifndef SERVO_PID_H_
#define SERVO_PID_H_

/*void PID(int p, int in, int id, int d);*/
void setPID(int p, int in, int id, int d);
void setSpeed(int speed);
void reset(void);
int control(int x);

#endif /* SERVO_PID_H_*/
