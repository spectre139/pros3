#include "main.h"
#include <string>
using namespace pros;

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		lcd::set_text(2, "who tf clicked me");
	} else {
		lcd::clear_line(2);
	}
}
void my_task_fn(void* param) {
     std::cout << "Hello" << (char*)param << std::endl;
     // ...
 }
void initSensors(){
	ADIEncoder encoderL (encoderLTop_Port, encoderLTop_Port, false);
	ADIEncoder encoderR (encoderRTop_Port, encoderRTop_Port, false);
	ADIEncoder encoderM (encoderMTop_Port, encoderMTop_Port, false);

 }
void initialize() {
	lcd::initialize();
	lcd::set_text(1, "Welcome 139A Gods");
	initSensors();
	Motor example_motor_initializer (motorLFront_Port, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
	std::string text("PROS");
	Task my_task(my_task_fn, &text);
	lcd::register_btn1_cb(on_center_button);
	while(true){
		std::string a = "hello";
		std::string motorEncoder = std::to_string((int)example_motor_initializer.get_position());
		lcd::print(0, "motorEncoder: " + a);
		delay(20);
	}
}

void disabled() {}


void competition_initialize() {}
