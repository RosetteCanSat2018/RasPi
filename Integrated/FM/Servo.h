class Servo
{
private:
	unsigned int servo1_gpio;
	unsigned int servo2_gpio;
public:
	void SetServo(unsigned int _servo1_gpio, unsigned int _servo2_gpio);
	void MoveServo(int paddledegree1, int paddledegree2);
};
