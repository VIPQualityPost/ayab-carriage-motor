#include <Arduino.h>
#include <AccelStepper.h>
#include <limits.h>

#define footswitch_pin A3

#define limit_min_pin A5
#define limit_max_pin A4

#define motor_en_pin 13
#define motor_dir_pin 12
#define motor_step_pin 11

#define max_velocity 160000

float motor_acceleration = 1600.0;

float homing_velocity = 1600.0;
float cruise_velocity = 1600.0;

uint32_t min_position = 0;
uint32_t max_position;


bool continuous_knit = false;

typedef enum direction_t {
  left,
  right,
};

direction_t next_direction = left;

AccelStepper* carriage_motor = new AccelStepper(
  AccelStepper::MotorInterfaceType::DRIVER,
  motor_step_pin,
  motor_dir_pin);

void home_carriage(direction_t);
void knit_row(direction_t);
void flip_direction();

void setup() {
  pinMode(footswitch_pin, INPUT_PULLUP);
  pinMode(limit_min_pin, INPUT_PULLUP);
  pinMode(limit_max_pin, INPUT_PULLUP);

  carriage_motor->setEnablePin(motor_en_pin);
  carriage_motor->enableOutputs();
  carriage_motor->setAcceleration(motor_acceleration);
  carriage_motor->setSpeed(homing_velocity);
  carriage_motor->setMaxSpeed(max_velocity);

  while(digitalRead(footswitch_pin) != LOW) {};

  home_carriage(direction_t::left);
  home_carriage(direction_t::right);

  carriage_motor->setSpeed(cruise_velocity);
}

void loop() {
  if(digitalRead(footswitch_pin) == LOW)
    continuous_knit = !continuous_knit;

  if(continuous_knit)
    knit_row(next_direction);
}

void home_carriage(direction_t direction)
{
  // no bad directions !!
  if(direction != direction_t::left && direction != direction_t::right)
    return;

  uint8_t limit_pin = direction == direction_t::left ? limit_min_pin : limit_max_pin;

  if(digitalRead(limit_pin) == HIGH)
  {
    // huge move that should exceed the size of the bed
    if(direction == direction_t::left)
      carriage_motor->move(LONG_MIN);
    else
      carriage_motor->move(LONG_MAX);

    // get to the limit
    while(digitalRead(limit_pin) != LOW)
      carriage_motor->run();
    carriage_motor->setAcceleration(1000000000.);
    carriage_motor->stop();
    carriage_motor->runToPosition();
    // back off the limit with a small move, 1/10 rev (about 3mm)
    if(direction == direction_t::left)
      carriage_motor->move(160);
    else
      carriage_motor->move(-160);

    // finish the move, clean up and leave
    while(carriage_motor->run()) {};

    if(direction == direction_t::left)
      carriage_motor->setCurrentPosition(0);
    else
      max_position =  carriage_motor->currentPosition();

    flip_direction();
  }
}

void knit_row(direction_t direction)
{
  // move to the limit at the desired direction
  if(direction == direction_t::left)
    carriage_motor->moveTo(min_position);
  else
    carriage_motor->moveTo(max_position);

  // don't return until we're done with the move (this would be bad if we wanted to do other things)
  while(carriage_motor->run()) {};

  // swap the next direction to move.
  flip_direction();
}

void flip_direction()
{
  next_direction = next_direction == direction_t::left ? direction_t::right : direction_t::left;
}
