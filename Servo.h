class Servo
{
public:
	void SetServo(unsigned int servo1_gpio, unsigned int servo2_gpio);
	void MoveServo(unsigned int servo1_gpio, unsigned int servo2_gpio, int paddledegree);
};
