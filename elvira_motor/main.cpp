#include "motor_driver.h"
#include <stdint.h>
#include <unistd.h>
#include "math.h"

/*   ASCII Schematic (Plan View, numbers indicate leg #)
 *        1  F   2     z
 *         \ * /       |_ x
 *          [ ]         
 *         /   \
 *        3     4
 *        
 *  ASCII Schematic (Section View, from back)     
 *    
 *        /\[ ]/\      y
 *       /       \     |_ x
 *       
 *       
 *  Rotation Definitions
 *  Yaw   = My (how it changes directions, ∈[0, 2pi], F is 0, 2pi and increases clockwise)
 *  Pitch = Mx (lean forward or backward, ∈[-1, 1], 0 is aligned with yaxis. +CW, -CCW)
 *  Roll  = Mz (lean side to side, ∈[-1. 1], 0 is aligned with yaxis. +forward, -backward CCW)
 *  
 *  
 *  Servo Map
 *  LEG | Shoulder, Top, Bottom
 *  1   | 0, 2, 1
 *  2   | 6, 8, 7
 *  3   | 3, 5, 4
 *  4   | 9, 11, 10
 *  
 *  (Top is top of leg actuator, bottom is bottom of leg actuator)
 *  
 *  General Notes:
 *  - 20ms delay is approximate min for servo refresh, any lower causes eratic behavior
 */

// Constructors
// CONSTRUCT THE LEGS
Body body;
PCA9685 PCA9685(0x40);

Leg leg1(0,2,1);
Leg leg2(6,8,7);
Leg leg3(3,5,4);
Leg leg4(9,11,10);

// Define globals
uint16_t delayt = 1000;  // Define movement delay

// function prototypes
void setup();
void stanceReset();
void delay(uint16_t);
void sayHello(uint8_t);
void gettall();
void getmiddle();
void getshort();


int main(){
    setup();    
    return 0;
}

// setup functions
void setup() {
  stanceReset();
  delay(delayt);

  sayHello(2);
  delay(delayt);
//  aroundTown();
  delay(delayt);  
  

//  walk(3);  
  stanceReset();
  delay(2*delayt);
  gettall();
    getshort();
}

// movement functions
void stanceReset(){
  leg1.straighten();
  leg2.straighten();
  leg3.straighten();
  leg4.straighten();
  gettall();
  leg1.sweep(180);
  leg2.sweep(0);
  leg3.sweep(180);
  leg4.sweep(0);
}

void gettall(){
  leg1.articulate(180,180);
  leg2.articulate(0,0);
  leg4.articulate(180,180);
  leg3.articulate(0,0);
}

void getmiddle(){
  leg1.articulate(90,90);
  leg2.articulate(90,90);
  leg4.articulate(90,90);
  leg3.articulate(90,90);
}

void sayHello(uint8_t n){
  stanceReset();
  int pushupTime = 200;
  for(int i =0; i<n;i++){
    leg1.articulate(180,0);
    leg2.articulate(0,180);
    leg4.articulate(180,0);
    leg3.articulate(0,180);
    delay(pushupTime);
    leg1.articulate(180,180);
    leg2.articulate(0,0);
    leg4.articulate(180,180);
    leg3.articulate(0,0);
    delay(pushupTime);
  }
}

void getshort(){
  leg1.articulate(0,0);
  leg2.articulate(180,180);
  leg4.articulate(0,0);
  leg3.articulate(180,180);
}

// utility functions
void delay(uint16_t time){
    usleep(time * 1e3);
}
