/* Name: main.c
 * Author: Team Mega
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include "uart.h"

//************* global variable*****************
typedef enum {initialize,trigger_1, trigger_2, trigger_check, check_complete, speaker_ready} state;
typedef enum {trigger_ready, trigger_in_progress, target_out_of_range} echostate;
volatile echostate echo_state;
volatile state current_state;
volatile unsigned long int ECHOHigh, ECHOLow;
volatile unsigned int count,TimeOutCnt,Tick,ECHOHighPipe,detections,reinitialize,num,complete_flag,sweep_complete,object;
volatile float sound_freq = 0.0,dist=0.0, capt1=0.0, capt2=0.0, capt3=0.0;
volatile int cycle, timer_2_flag,angle,count_t2, count_t2_2,i, tick;
volatile int di[100];
volatile char Trigger_button;
volatile int angl[100];
int sum_angle;
int sample_count;
int flag;
/**************************************************************************************
 When the TIMER 2 reaches OCRA value generates a PWM that is driven into speaker
 we use the intterupt to calculate a time of 3 seconds.
 **************************************************************************************/

ISR(TIMER2_COMPA_vect){ // plays sound for 3 seconds
    count_t2++; // increments for every interrupt
    timer_2_flag=1; // sets flag so to not return into speaker()
    if(!(count_t2%2)){ // since to interrupts are equal to 1 period we use modulus
        count_t2_2++; // counts the number of periods
    }
    cycle = (int)(3*sound_freq); //* we changed value from 6 to 3 for testing purposes
    if(count_t2_2== cycle){ // based on the calculations 3 seconds has elapsed since the speaker start
        //TCCR2B= 0x00; //Turn off timer 2 clock
        //TIMSK2 = 0x00;// Turn off interrupts for Timer2
       // PORTD &=~(1<<PD7);// turns Pin D7 off
        sweep_complete= 1; //set sweep complete flag
       // printf_P(PSTR("Sweep complete!")); // prints a final message
    }
    
}
/**************************************************************************************
  When when TIMER0 reaches OCRA value generates the pulse wave width
 that is required to position motor at sepererate places .
 ***************************************************************************************/

ISR(TIMER0_COMPA_vect){ //moves motor by 1 tick (7.5 degrees)
    count++; // counter for compare A interrupt
    if(!(count%7)&&(count>20)){ //every 25 pulse widths increments position going left
        OCR0A+=1; //The motor moves one increment approximately 7.5 degrees
        current_state = trigger_1; //change of state after 1 tick has been completed
        echo_state = trigger_ready; //change echo state thus allowing for re-trigger
    }
    
    if(OCR0A >31){ //maximum angle for the motor
        //state = complete; // sensor has run a full sweep
        TCCR0B = 0x00; //stops motor
        current_state= speaker_ready; //bring system back to an initial state
        //allows for motor to restart after prompt
    }
    num = OCR0A; //stores the value in order to calculate distance
}


/***************************************************************************************
 When the echo length is longer than the counter duration, we use an additional byte to
 indicate how many times we reach the maximum value.
 ***************************************************************************************/

ISR (TIMER1_OVF_vect){    // For long ECHO High
    if(ECHOHighPipe >= 2) { //it has been high for an extended period of time
        TIMSK1 = 0;    // No further interrupts.
        TCCR1B = 0; // Stop Clock
        echo_state = target_out_of_range ;    // change state
        TCCR0B = (1<<CS02) | (1<<CS00); //sets prescaler to 1024
      //  printf_P(PSTR("Overflow! \n"));//alerts user of an overflow
    }
    ECHOHighPipe++;    // Add 1 to High byte.
}

/***************************************************************************************
 Interrupt service routine called when the input capture pin state is changed
 ***************************************************************************************/

ISR (TIMER1_CAPT_vect) {    // Start and Stop ECHO measurement;
    if((TCCR1B & (1<<ICES1)) != 0) { // a rising edge has been detected
        TCCR1B |= (1<<CS11);    // Start counting with ck/8;
        TCCR1B &= ~(1<<ICES1);  // Configure Negative Edge Capture for end of echo pulse.
    }
    
    else {                        // a falling edge has been detected
        ECHOLow = TCNT1; // stores TCNT(this is the time of the echo pulse
        ECHOHigh = ECHOHighPipe;
        TIMSK1 = (1<<OCIE1B);    // Enables the Compare B interrupt for POST Trigger Delay: Approx 10mS
        TCNT1 = 0;
        detections +=1;
        
    }
}


/***************************************************************************************
 Interrupt service routine called when the counter 1 has reached the compare value
 ***************************************************************************************/
ISR (TIMER1_COMPB_vect) {    // Compare B: Post ECHO delay 10mS
    echo_state = trigger_ready;
    if(current_state==trigger_1){
        capt1= ECHOLow;
    }else if(current_state==trigger_2){
        capt2= ECHOLow;
    }else if(current_state== trigger_check){
        capt3= ECHOLow;
    }
    TCCR1B= 0x00;
    //printf_P(PSTR("COMPB"));
    //End measurement
}

/***************************************************************************************
 Interrupt service routine called when the counter 1 has reached the compare value
 ***************************************************************************************/
ISR (TIMER1_COMPA_vect) {    // Compare A : End of Trigger Pulse
    PORTB &= ~(1<<PB1); // turns PB1 off thus stopping trigger
    TIMSK1 = (1<<ICIE1)|(1<<TOIE1); // enables the T/C1 Overflow and Capture interrupt;
    TCCR1B = (1<<ICES1);            // Set Positive edge for capture but Don't count yet
}


/*****************************************************************************************
 ************************************* FUNCTIONS *****************************************
 *****************************************************************************************/

/*
void object_detection(){
	int final_angle;
	int final_distance;
	int sum_distance;

	

	if(((abs(di[tick]-di[tick-1]))<=7) && flag==0) //monitor for first point of object
	{
		sample_count=2;
		sum_distance=di[tick]+di[tick-1];
		sum_angle=angl[tick]+angl[tick-1];
		flag=1;
	}
	 if(((abs(di[tick]-di[tick-1]))<=7) && (flag==1 || flag==2)) //the rest of the object
	{
		sample_count++;
		sum_distance=sum_distance+di[tick];
		sum_angle=sum_angle+angl[tick];
		flag=2;
		printf_P(PSTR("count is %d\n"), sample_count);
	}
	

	 if(((abs(di[tick]-di[tick-1]))>7) && (flag==1 || flag==2)) //first point to the left of object
	{
		final_distance=sum_distance/sample_count;
		final_angle=sum_angle/sample_count;
		printf_P(PSTR("Distance is %d, Angle is %d\n"), final_distance, final_angle);
		sample_count=0;
		flag=0;
		sum_distance=0;
		sum_angle=0;
	}
	

}
*/

void speaker(){
float freq_dist;
int c,dist_cal,angle_cal;
int x;
	int dist_val[10];
	int angle[10];
	int array_2_val=0;
    if(timer_2_flag == 0){
        TCCR2A = (1<<COM2A0) | (1<<WGM21); // Toggle OC2A on compare, Set CTC mode
        TCCR2B = (1<<CS22)|(1<<CS21)|(0<<CS21); //Set prescaler - 256
        TCNT2 = 0;//Starts timer at zero
		for(tick=0; tick<=31;tick++){
		if(di[tick]){
			x=1;
			dist_cal = 0;
			angle_cal = 0;
		/*while (1){
			if (abs(di[tick+x]-di[tick])<5){
				x++;
				}
			if ((abs(di[tick+x]-di[tick])>5)&&(x>4)){
				for(c=0; c<=x;c++){
				dist_cal= dist_cal+di[tick+c];
				angle_cal= angle_cal+angl[tick+c];
				}
				dist_val[array_2_val]= (int)(dist_cal/c) ;
				angle[array_2_val] = (int)(angle_cal/c);
				printf_P(PSTR("\nDistance = %d Angle=%d"), dist_val[array_2_val],angle[array_2_val]);
				array_2_val++;
				tick=tick +x;
				break;
			}
			if (abs(di[tick+x]-di[tick])>5){ break;}
			}*/
			if((abs(di[tick]-di[tick-1])<5) && (abs(di[tick+1]-di[tick])<5) && (abs(di[tick+2]-di[tick])<5) ){
				dist_val[array_2_val]= (di[tick]+di[tick-1]+di[tick+1]+di[tick+2])/4;
				angle[array_2_val] =(angl[tick]+angl[tick-1]+angl[tick+1]+angl[tick+2])/4;
				tick=tick+3;
				printf_P(PSTR("\nDistance = %d Angle=%d\n"), dist_val[array_2_val],angle[array_2_val]);
				array_2_val++;
				} 
		}
			
		}
		//printf_P(PSTR("\nfrequency played %d\n"),dist_val);
		for(array_2_val=0; array_2_val<3; array_2_val++){
		sweep_complete = 0;
		freq_dist = (float)(dist_val[array_2_val]);
        sound_freq = 0.580723595323*freq_dist + 261.6096013065;//converts recorded distance to frequency
        OCR2A =(int)((14.7456e6/256.0)/(2.0*sound_freq)) ; // Set OCR2A value to the one required for
		printf_P(PSTR("Frequency= %.2f\n"), sound_freq);
		while(sweep_complete == 0){};
		TCNT2= 0;
		count_t2=0;
		count_t2_2=0;
	
		}
		
    }
	   TCCR2B= 0x00; //Turn off timer 2 clock
	   TIMSK2 = 0x00;// Turn off interrupts for Timer2
	   PORTD &=~(1<<PD7);// turns Pin D7 off
}



void motor_reset(){
    TCCR0A = (1<<COM0A1) | (1<<WGM01)|(1<<WGM00);
    TCCR0B = (1<<CS02) | (1<<CS00); //14.7/1040
    TCNT0 = 0;
    OCR0A = 7; // position -90 *might need to change to 7 for better performance
    complete_flag=1;
    //fast pulse with of duty cycle of 10 percent frequency of (14.7e^7/1020)
}


void Trigger() {        // Config Timer 1 for 10 to 15uS pulse.
    if(echo_state == trigger_ready) {    // Don't allow re-trigger.
        echo_state = trigger_in_progress ;                // Set Measurement in progress FLAG
        TCNT1 = 0;                // Clear last Echo times.
        ECHOHighPipe = 0;
        OCR1B = 20;            // 10 mS Post echo Delay
        OCR1A = 12;                // 10 us Trigger length.
        PORTB |= (1<<PB1);        // Start Pulse.
        TIFR1 = 0xFF;            //  Clear all timer interrupt flags
        TCCR1A = 0;   // Timer mode with Clear Output on Match
        TCCR1B = (1<<WGM12) | (1<<CS11);  // Counting with CKio/8 CTC Mode enabled
        TIMSK1 = (1<<OCIE1A)|(1<<TOIE1);    // enables the T/C1 Overflow, Compare A, and Capture interrupt;
    }
}

void motor_resume(){
    TCCR0B = (1<<CS02) | (1<<CS00);
}



void distance_angle_calc(){
   int d;
    Trigger(); // initialize trigger
    while (echo_state == trigger_in_progress){}; // wait till trigger was successful
    if ((echo_state == trigger_ready)&&(detections==2)){
         //distance[detections] = 1; //(float)((ECHOLow*0.000000544*340.0)/2.0); // distance of object in meters
        angle = (int)(((num - 7.0) *7.5) - 90.0);
        dist= (((capt1+capt2+capt3)*0.0000544*340.0)/2.0)/2.0;
        d= (int)dist;
		
        if (d<350 && tick>0){
			 angl[tick]=angle;
			 di[tick]=d;
			// object_detection();
		}
        if((current_state == trigger_2 )||(current_state == trigger_check)){
            //speaker_setup();
            //speaker();//Activate square wave frequency
        }
        
        
        if (echo_state == target_out_of_range){
            echo_state = trigger_ready;
			di[tick]=0;
			angl[tick]=angle;
        }
    }
}
void putty_screen(){
    int i;
    for(i=0; i<=2; i++){
        printf_P(PSTR( "********************************************************************************\n"));
        
    }
    printf_P(PSTR(" _______ _______ _______ _______      _______ _______  ______ _______\n"));
    printf_P(PSTR("    |    |______ |_____| |  |  |      |  |  | |______ |  ____ |_____|\n"));
    printf_P(PSTR("    |    |______ |     | |  |  |      |  |  | |______ |_____| |     |\n"));
    printf_P(PSTR("\n"));
    printf_P(PSTR("\n"));
    printf_P(PSTR("        8888888888 .d8888b.  888    888  .d88888b.  \n"));
    printf_P(PSTR("        888       d88P  Y88b 888    888 d88P* *Y88b \n"));
    printf_P(PSTR("        888       888    888 888    888 888     888 \n"));
    printf_P(PSTR("        8888888   888        8888888888 888     888 \n"));
    printf_P(PSTR("        888       888        888    888 888     888 \n"));
    printf_P(PSTR("        888       888    888 888    888 888     888 \n"));
    printf_P(PSTR("        888       Y88b  d88P 888    888 Y88b. .d88P \n"));
    printf_P(PSTR("        8888888888 *Y8888P*  888    888  *Y88888P*   \n"));
    
    for(i=0; i<=2; i++){
        printf_P(PSTR( "********************************************************************************\n"));
        
    }
    printf_P(PSTR("\n"));
    printf_P(PSTR("\n"));
    printf_P(PSTR("\n"));
}


void system_start(){
    printf_P(PSTR("==============================================================================\n"));
    printf_P(PSTR("\n"));
    printf_P(PSTR("Press \'T\' to scan\n\r"));//prints a prompt to host computer
    scanf(("%c"),&Trigger_button);//waits for button to be pressed
    printf_P(PSTR("%c\n\r"),Trigger_button);//prints the button that was pressed
    printf_P(PSTR("===============================================================================\n"));
    printf_P(PSTR("\n"));
    
}

/*****************************************************************************************************
 ********************************************* MAIN FUNCTION *****************************************
 *****************************************************************************************************/

int main(void){
	//int sum_angle=0;
	//int sample_count=0;
	//int flag=0;
	tick= 0;
    init_uart(95); //set baud rate for UART
    putty_screen(); // The Putty Startup Screen
    DDRB = (1<<PB3)|(1<<PB1);//PB1 as Output for Trigger pulse, PB3 as output for motor signal
    DDRD = (0<<PD6) |(1<<PD7);//PD6 as Input for ECHO, PD7 as output for speaker signal
    TIMSK0 = (1<<OCIE0A);//Enable Timer 0 output compare A match interrupt
    TIMSK2 = (1<<OCIE2A);//Enable Timer 2 output compare A match interrupt *doesn't seem to be needed for the code
    sei(); //enable interrupts
    //initialize variables
    detections = 0;
    reinitialize = 0;
    count = 0;
    current_state  = initialize;
    echo_state = trigger_ready;
    complete_flag= 0;
    sweep_complete=1;
    timer_2_flag= 0;
    while(1){
        if(sweep_complete==1){
            system_start();
            complete_flag = 0;
            sweep_complete= 0;
            count_t2 = 0;
            count_t2_2 =0;
        }
        if((Trigger_button == 't')||(Trigger_button=='T')){
            switch(current_state){
                case initialize: if(complete_flag!=1){motor_reset();};break;
                case trigger_1: TCCR0B= 0x00; distance_angle_calc();current_state= trigger_2; break; //once a tick has been completed sonar runs
                case trigger_2: distance_angle_calc(); current_state= trigger_check; break; //sends second pulse after first echo has been received led turns
                case trigger_check: if(detections == 1) distance_angle_calc(); tick++;   current_state = check_complete; break; //begins clock after second echo has been received led2 turns on
                case check_complete: detections= 0; motor_resume(); break;
                case speaker_ready:  speaker(); current_state = initialize; break;
            }
        }else{
            printf_P(PSTR("INVALID OPTION please try again\n"));
            sweep_complete =1;
            printf_P(PSTR("\n"));
        }
    }
    
    return 0;
}