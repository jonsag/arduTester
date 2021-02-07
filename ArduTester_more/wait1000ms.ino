// wait loops , sketch compatible


/****   delayMicroseconds()   ****
Pauses the program for the amount of time (in microseconds) specified as parameter.
Currently, the largest value that will produce an accurate delay is 16383.
This could change in future Arduino releases.
For delays longer than a few thousand microseconds, you should use delay() instead.
This function works very accurately in the range 3 microseconds and up.
We cannot assure that delayMicroseconds will perform precisely for smaller delay-times.

As of Arduino 0018, delayMicroseconds() no longer disables interrupts.
*/



void wait3s(void){
  delay(3000);
}

void wait2s(void){
  delay(2000);
}

void wait1s(void){
  delay(1000);
}

void wait500ms(void){
  delay(500);
}

void wait300ms(void){
    delay(300);
}

void wait200ms(void){
    delay(200);
}

void wait100ms(void){
    delay(100);
}

void wait50ms(void){
    delay(50);
}

void wait20ms(void){
    delay(20);
}

void wait10ms(void){
  delayMicroseconds(10000);
}

void wait5ms(void){
  delayMicroseconds(5000);
}

void wait2ms(void){
  delayMicroseconds(2000);
}

void wait1ms(void){
  delayMicroseconds(1000);
}

void wait500us(void){
  delayMicroseconds(500);
}

void wait300us(void){
  delayMicroseconds(300);
}

void wait200us(void){
  delayMicroseconds(200);
}

void wait100us(void){
  delayMicroseconds(100);
}

void wait50us(void){
  delayMicroseconds(50);
}

void wait20us(void){
  delayMicroseconds(20);
}

void wait10us(void){
  delayMicroseconds(10);
}

void wait5us(void){
  delayMicroseconds(5);
}

void wait4us(void){
  delayMicroseconds(4);
}

void wait3us(void){
  delayMicroseconds(3);
}

void wait2us(void){
 asm volatile (
    "nop \n"
    "nop \n"
    );
//delayMicroseconds(2);
}

void wait500ns(void){
 asm volatile (
    "nop \n"
    );
//  delayMicroseconds(1);
}
