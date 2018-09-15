int n1 = 0;
int flag=1;
int a,b,c,d,e,f,g;
int ctrls=1,ctrls2=1;
int prev=0,prev2=0;

int signal_flag1=49;
int signal_flag2=47;
///////////////////
// MOTOR DIRECTION PINS ; 1 ---> TOP   2 ---> BOTTOM
//////////////////
int L1_CC = 5;
int L1_CCW = 40;
int L2_CC = 6;
int L2_CCW = 7;
int R1_CC = 13;
int R1_CCW = 12;
int R2_CC = 15;
int R2_CCW = 14;

// RPM Vars
int rpmL1 = 0;
int rpmL2 = 0;
int rpmR1 = 0;
int rpmR2 = 0;
int ctr=1;
// RPM Setpoints
float set_rpmL1 = 100;
float set_rpmR1 = 100;
float set_rpmL2 = 100;
float set_rpmR2 = 100; 

// Time recorder
unsigned long lastmillis = 0;

// PID params
// kp
float kpL1 = 0.00001;
float kpL2 = 0.00001;
float kpR1 = 0.00001;
float kpR2 = 0.00001;

// kd
float kdL1 = 0.008;
float kdL2 = 0.008;
float kdR1 = 0.008;
float kdR2 = 0.008;

// For Line following  LS PID params

float kpls = 1;
float kdls = 0.1;
float kils = 1;



// Bases for motor's pid
int baseL1 = 0;
int baseL2 = 0;
int baseR1 = 0;
int baseR2 = 0;

//  LS Line following Base speed //////////////////////
int basels = 80;



/////////////// Stop counting initiator  ///////////////////////////////////
int init_stop_count = 0; 


////////////////////////// LS correction variable  /////////////////////////////////////  

int corr_ls = 0;   ////////////////////// Always used with ls on the rear end of motion //////////////////////////////




// Error vars
float errorL1 = 0 ;
float errorL2 = 0 ;
float errorR1 = 0 ;
float errorR2 = 0 ;

//  ERROR variable for LS ////////////
float errorls = 0;



// Last Error Vars
float lastL1 = 0;
float lastL2 = 0;
float lastR1 = 0;
float lastR2 = 0;

//////////////////////////// Last error storage variable for LS ///////////////////////////////////////

float lastls = 0;



// Change in error Vars
float derrL1 = 0;
float derrL2 = 0;
float derrR1 = 0;
float derrR2 = 0;

////////////////////////////// Differential of Error for LS ///////////////////////////////
float derrls = 0;



// PID VARS
float pidL1 = 110;
float pidL2 = 110;
float pidR1 = 110;
float pidR2 = 110;

/////////////////////////// PID variable for LS Line following ///////////////////////


float pidls = 0;

int flagls=0,v=0,prev_v=77,v1=0;

// Multipliers
float multL1 = 1;
float multL2 = 1;
float multR1 = 1;
float multR2 = 1;
// MOTOR PWM PINS
int L1_PWM = 8;
int L2_PWM = 9;
int R1_PWM = 10;
int R2_PWM = 11;


int val,x;
// ENCODER PINS CONFIG
int L1_count = 21;
int L2_count = 3;
int R1_count = 20;
int R2_count = 19;

/////////////////////////////// LS pin Configuration //////////////////////////////////////////
int ls1 = A1;
int junctionls1 = 38;
int ls2 = A5;
int junctionls2=42;

// ENCODER COUNT VARIABLES

unsigned int CountL1 = 0;
unsigned int CountL2 = 0;
unsigned int CountR1 = 0;
unsigned int CountR2 = 0;

// ENCODER STOPPING COUNT VARIABLES
unsigned long int stop_countL1 = 0;
unsigned long int stop_countL2 = 0;
unsigned long int stop_countR1 = 0;
unsigned long int stop_countR2 = 0;

//junction variables
int dark_jun = 0;
int light_jun = 0;


/////////////////////// LS Line following controlled speed ///////////////////////////////////

int right_pwmls = 0;
int left_pwmls = 0;

int maxspeedls = 180;

///////////////////////////// LS setpoint for PID and LS current Value Storage ///////////////////////////

float ls1_value = 0;
float set_ls1_value = 35;

float ls2_value = 0;
float set_ls2_value = 35;

// Last Value Storage ///

int last_ls1_value = 0;
int last_ls2_value = 0;





// Ecoder manipulation vars
int m = 1;
int n = 1;

// Mean Vars

double mean = 0.0;
float mean_pid = 0.0;

//////////////////////////////////////////////////////////// To do process vars //////////////////////////////////////////////////////////////////////////////////////////////////////

char process = 'f';      /////////////////////// f:Forward, b:Backward, l:Left, r:Right, o:Orientation, x:StrafeLeft, y:STRAFERIGHT




////////////////////// Stop Brake flag ///////////////////////////////////////
int stop_flag = 0;




/////////////////////////////// Junction Counter ////////////////////////////////////////////
int junction_countls1 = 0;
int junction_countls2 = 0;





/////////////////////////////  Distance Measuring vars for all the motor ////////////////////

unsigned long long int dist_countL1 = 0;

unsigned long long int dist_countL2 = 0;

unsigned long long int dist_countR1 = 0;

unsigned long long int dist_countR2 = 0;
/////////////////////////////////// Orientation variable ////////////////////////////////////////////////////////////

int orient = 0;


/// PWM for orienting the bot 
int orient_pwm = 50;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////   Function Definitions   ////////////////////////////////////////////////////
void cal_and_set_rpm(float set_rpmL1, float set_rpmL2, float set_rpmR1, float set_rpmR2);
void line_follow_pidls();
void line_follow_pid_bothls_forward();
void line_follow_pid_bothls_backward();
void FORWARD();
void BACKWARD();
void LEFT();
void RIGHT();
void BRAKE();
void STOP();
//void JUN_PULSE_map(analogRead(ls2),50,810,0,70);
void JUN_PULSE_LS1();
void L1_SHAFT_COUNT();
void L2_SHAFT_COUNT();
void R1_SHAFT_COUNT();
void R2_SHAFT_COUNT();
void correct_orient();
void stop_counter_ls();
//int map(analogRead(ls2),50,810,0,70);
void (*resetFunc)(void)=0;
////////////////////////////////////////Encoder Interrupt Service Routines //////////////////////////////////////////////////////


/*int map(analogRead(ls2),50,810,0,70)
{
  a=!(digitalRead(22));
  b=!(digitalRead(23));
  c=!(digitalRead(24)); 
  e=!(digitalRead(26));
  f=!(digitalRead(27));
  g=!(digitalRead(28));
  v1=((a*10)+(b*20)+(c*30)+(e*50)+(f*60)+(g*70));
  if(v1>0)
  {
    v=v1/(a+b+c+e+f+g);
    return v;
  }
  else return 77;
}

*/
void L1_SHAFT_COUNT()
{
  CountL1++;
  //Serial.println("L1=");
  //Serial.print(CountL1);
}

void L2_SHAFT_COUNT()
{
  CountL2++;
  stop_countL2++;
  dist_countL2++;
  //Serial.println("L2=");
  //Serial.println(CountL2);
}

void R1_SHAFT_COUNT()
{
  CountR1++;
  //Serial.println("R1");
  //Serial.println(CountR1);
}

void R2_SHAFT_COUNT()
{
  CountR2++;
  stop_countR2++;
  dist_countR2++;
  //Serial.println("R2=");
  //Serial.println(CountR2);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////// Encoder count wise stopping function ///////////////////////////////////////////////////

void stop_counter_ls()
{
 //Serial.println("check"); 
  if(stop_countL2 > 110)
{
  
BRAKE();
delay(200);
STOP();
correct_orient();
process = 'o';
stop_flag=1;
//Serial.println("Stop Flag increased !");
prev= 0;
//Serial.println("check");
//junction_countls2 = 0;
basels = 80;
digitalWrite(signal_flag1,HIGH);
Serial.println("sent");
while(!digitalRead(signal_flag2));
{
  BRAKE();
}
 digitalWrite(signal_flag1,LOW);

}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////// Junction Interrupt Service Routine ///////////////////////////////////////////////////
/*void JUN_PULSE_LS1()
{
  
  
  
  junction_countls1++;
  stop_countL2 = 0;
  stop_countR2  = 0;
  Serial.println(junction_countls1);
  
 
   } 

void JUN_PULSE_map(analogRead(ls2),50,810,0,70)
{
 
  junction_countls2++;
  stop_countL2 = 0;
  stop_countR2  = 0;
  
  } 
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////  Movement Functions /////////////////////////////////////////////////////////////////////////


  void FORWARD()
{ m = 1;
n = 1;
  digitalWrite(L1_CC,HIGH);
  digitalWrite(L1_CCW,LOW);
  digitalWrite(L2_CC,HIGH);
  digitalWrite(L2_CCW,LOW);
  digitalWrite(R1_CC,HIGH);
  digitalWrite(R1_CCW,LOW);
  digitalWrite(R2_CC,HIGH);
  digitalWrite(R2_CCW,LOW);
}

void BACKWARD()
{ m = 2;
n = 2;
  digitalWrite(L1_CCW,HIGH);
  digitalWrite(L1_CC,LOW);
  digitalWrite(L2_CCW,HIGH);
  digitalWrite(L2_CC,LOW);
  digitalWrite(R1_CCW,HIGH);
  digitalWrite(R1_CC,LOW);
  digitalWrite(R2_CCW,HIGH);
  digitalWrite(R2_CC,LOW);
}

void LEFT()
{ 
  digitalWrite(L1_CCW,HIGH);
  digitalWrite(L1_CC,LOW);
  digitalWrite(L2_CCW,HIGH);
  digitalWrite(L2_CC,LOW);
  digitalWrite(R1_CC,HIGH);
  digitalWrite(R1_CCW,LOW);
  digitalWrite(R2_CC,HIGH);
  digitalWrite(R2_CCW,LOW);
   analogWrite(L1_PWM, 80);
 analogWrite(L2_PWM, 80);
 analogWrite(R1_PWM, 80);
 analogWrite(R2_PWM, 80);
  
}

void RIGHT()
{ 
  digitalWrite(L1_CC,HIGH);
  digitalWrite(L1_CCW,LOW);
  digitalWrite(L2_CC,HIGH);
  digitalWrite(L2_CCW,LOW);
  digitalWrite(R1_CCW,HIGH);
  digitalWrite(R1_CC,LOW);
  digitalWrite(R2_CCW,HIGH);
  digitalWrite(R2_CC,LOW);
}
void BRAKE()
{ 
  digitalWrite(L1_CC,HIGH);
  digitalWrite(L1_CCW,HIGH);
  digitalWrite(L2_CC,HIGH);
  digitalWrite(L2_CCW,HIGH);
  digitalWrite(R1_CCW,HIGH);
  digitalWrite(R1_CC,HIGH);
  digitalWrite(R2_CCW,HIGH);
  digitalWrite(R2_CC,HIGH);
   analogWrite(L1_PWM, 80);
 analogWrite(L2_PWM, 80);
 analogWrite(R1_PWM, 80);
 analogWrite(R2_PWM, 80);
}
void STOP()
{ 
  digitalWrite(L1_CC,LOW);
  digitalWrite(L1_CCW,LOW);
  digitalWrite(L2_CC,LOW);
  digitalWrite(L2_CCW,LOW);
  digitalWrite(R1_CCW,LOW);
  digitalWrite(R1_CC,LOW);
  digitalWrite(R2_CCW,LOW);
  digitalWrite(R2_CC,LOW);
  analogWrite(L1_PWM, 0);
 analogWrite(L2_PWM, 0);
 analogWrite(R1_PWM, 0);
 analogWrite(R2_PWM, 0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////// Correct Orientation Function ///////////////////////////////////////////////////

void correct_orient()
{
  ls1_value = map(analogRead(ls1),230,920,20,70);        // The LS value is MAPPED, LS MIN value <--> 0 and LS MAX value <--> 70 
  ls2_value = map(analogRead(ls2),50,810,0,70);

if(ls1_value < 70)
{
 last_ls1_value = ls1_value;

  if(ls1_value > 40)
  {
    while(ls1_value > 40)
    {ls1_value = map(analogRead(ls1),230,920,20,70);
    digitalWrite(L1_CC,HIGH);
  digitalWrite(L1_CCW,LOW);
  digitalWrite(L2_CC,HIGH);
  digitalWrite(L2_CCW,HIGH);
  digitalWrite(R1_CCW,HIGH);
  digitalWrite(R1_CC,LOW);
  digitalWrite(R2_CCW,HIGH);
  digitalWrite(R2_CC,HIGH);
  analogWrite(L1_PWM, orient_pwm);
 analogWrite(L2_PWM, orient_pwm);
 analogWrite(R1_PWM, orient_pwm);
 analogWrite(R2_PWM, orient_pwm);
  }
  }
  if(ls1_value < 30)
  {
    while(ls1_value < 30)
    {ls1_value = map(analogRead(ls1),230,920,20,70);
   digitalWrite(L1_CCW,HIGH);
  digitalWrite(L1_CC,LOW);
  digitalWrite(L2_CC,HIGH);
  digitalWrite(L2_CCW,HIGH);
  digitalWrite(R1_CC,HIGH);
  digitalWrite(R1_CCW,LOW);
  digitalWrite(R2_CCW,HIGH);
  digitalWrite(R2_CC,HIGH);
  analogWrite(L1_PWM, orient_pwm);
 analogWrite(L2_PWM, orient_pwm);
 analogWrite(R1_PWM, orient_pwm);
 analogWrite(R2_PWM, orient_pwm);
  }
  }
}

if(ls2_value < 70)
{
 last_ls2_value = ls2_value;
  if(ls2_value > 40)
  {
    while(ls2_value > 40)
    {ls2_value = map(analogRead(ls2),50,810,0,70);
   digitalWrite(L1_CCW,HIGH);
  digitalWrite(L1_CC,HIGH);
  digitalWrite(L2_CC,HIGH);
  digitalWrite(L2_CCW,LOW);
  digitalWrite(R1_CC,HIGH);
  digitalWrite(R1_CCW,HIGH);
  digitalWrite(R2_CCW,HIGH);
  digitalWrite(R2_CC,LOW);
  analogWrite(L1_PWM, orient_pwm);
 analogWrite(L2_PWM, orient_pwm);
 analogWrite(R1_PWM, orient_pwm);
 analogWrite(R2_PWM, orient_pwm);
  }
    
  }

  if(ls2_value < 30)
  {
    while(ls2_value < 30)
    {ls2_value = map(analogRead(ls2),50,810,0,70);
  digitalWrite(L1_CC,HIGH);
  digitalWrite(L1_CCW,HIGH);
  digitalWrite(L2_CCW,HIGH);
  digitalWrite(L2_CC,LOW);
  digitalWrite(R1_CCW,HIGH);
  digitalWrite(R1_CC,HIGH);
  digitalWrite(R2_CC,HIGH);
  digitalWrite(R2_CCW,LOW);
  analogWrite(L1_PWM, orient_pwm);
 analogWrite(L2_PWM, orient_pwm);
 analogWrite(R1_PWM, orient_pwm);
 analogWrite(R2_PWM, orient_pwm);
  }
    
  }

}


  if(ls1_value > 70)
  {
    
    if(last_ls1_value < 35)
    {

     
      while(ls1_value < 30 || ls1_value > 70)
    { 
      ls1_value = map(analogRead(ls1),230,920,20,70);

   digitalWrite(L1_CCW,HIGH);
  digitalWrite(L1_CC,LOW);
  digitalWrite(L2_CC,HIGH);
  digitalWrite(L2_CCW,HIGH);
  digitalWrite(R1_CC,HIGH);
  digitalWrite(R1_CCW,LOW);
  digitalWrite(R2_CCW,HIGH);
  digitalWrite(R2_CC,HIGH);
  analogWrite(L1_PWM, orient_pwm);
 analogWrite(L2_PWM, orient_pwm);
 analogWrite(R1_PWM, orient_pwm);
 analogWrite(R2_PWM, orient_pwm);
  }
    }


    if(last_ls1_value < 70)
    {
      while(ls1_value > 40)
    {ls1_value = map(analogRead(ls1),230,920,20,70);
    digitalWrite(L1_CC,HIGH);
  digitalWrite(L1_CCW,LOW);
  digitalWrite(L2_CC,HIGH);
  digitalWrite(L2_CCW,HIGH);
  digitalWrite(R1_CCW,HIGH);
  digitalWrite(R1_CC,LOW);
  digitalWrite(R2_CCW,HIGH);
  digitalWrite(R2_CC,HIGH);
  analogWrite(L1_PWM, orient_pwm);
 analogWrite(L2_PWM, orient_pwm);
 analogWrite(R1_PWM, orient_pwm);
 analogWrite(R2_PWM, orient_pwm);
  }
    }
  }

  if(ls2_value > 70)
  {

    if(last_ls2_value < 30)
    {
       while(ls2_value < 30 || ls2_value > 70)
    {ls2_value = map(analogRead(ls2),50,810,0,70);
  digitalWrite(L1_CC,HIGH);
  digitalWrite(L1_CCW,HIGH);
  digitalWrite(L2_CCW,HIGH);
  digitalWrite(L2_CC,LOW);
  digitalWrite(R1_CCW,HIGH);
  digitalWrite(R1_CC,HIGH);
  digitalWrite(R2_CC,HIGH);
  digitalWrite(R2_CCW,LOW);
  analogWrite(L1_PWM, orient_pwm);
 analogWrite(L2_PWM, orient_pwm);
 analogWrite(R1_PWM, orient_pwm);
 analogWrite(R2_PWM, orient_pwm);
  }
    }

    if( last_ls2_value < 70)
    {
       while(ls2_value > 40)
    {ls2_value = map(analogRead(ls2),50,810,0,70);
   digitalWrite(L1_CCW,HIGH);
  digitalWrite(L1_CC,HIGH);
  digitalWrite(L2_CC,HIGH);
  digitalWrite(L2_CCW,LOW);
  digitalWrite(R1_CC,HIGH);
  digitalWrite(R1_CCW,HIGH);
  digitalWrite(R2_CCW,HIGH);
  digitalWrite(R2_CC,LOW);
  analogWrite(L1_PWM, orient_pwm);
 analogWrite(L2_PWM, orient_pwm);
 analogWrite(R1_PWM, orient_pwm);
 analogWrite(R2_PWM, orient_pwm);
  }
    }
    
  }

  STOP();


}

////////////////////////////////////////////////////////////////// SETUP ////////////////////////////////////////////////////////////////////
void setup()
{
  pinMode(22,INPUT);
  pinMode(24,INPUT);
  pinMode(26,INPUT);
  pinMode(28,INPUT);
  pinMode(30,INPUT);
  pinMode(32,INPUT);
  pinMode(34,INPUT);

  pinMode(signal_flag1,OUTPUT);
  pinMode(signal_flag2,INPUT);
  
  delay(2000); // Setup time for ignore any power on varations
   junction_countls1 = 0;
  pinMode(junctionls1,INPUT);
  pinMode(junctionls2,INPUT);
  Serial.begin(9600);
  //MOTOR PINS SETUP
  pinMode(L1_CC,OUTPUT);
  pinMode(L1_CCW,OUTPUT);
  pinMode(L1_PWM,OUTPUT);
  pinMode(L2_CC,OUTPUT);
  pinMode(L2_CCW,OUTPUT);
  pinMode(L2_PWM,OUTPUT);
  pinMode(R1_CC,OUTPUT);
  pinMode(R1_CCW,OUTPUT);
  pinMode(R1_PWM,OUTPUT);
  pinMode(R2_CC,OUTPUT);
  pinMode(R2_CCW,OUTPUT);
  pinMode(R2_PWM,OUTPUT);

  ////// ls1 pins config
  pinMode(ls1,INPUT);
  //pinMode(junctionls1,INPUT);

  ////// ls2 pins config
  pinMode(ls2,INPUT);
  //pinMode(junctionls2,INPUT);
  
  //ENCODERS SETUP
  attachInterrupt(digitalPinToInterrupt(21),L1_SHAFT_COUNT,FALLING);
  attachInterrupt(digitalPinToInterrupt(19),L2_SHAFT_COUNT,FALLING);
  attachInterrupt(digitalPinToInterrupt(20),R1_SHAFT_COUNT,FALLING);
  attachInterrupt(digitalPinToInterrupt(3),R2_SHAFT_COUNT,FALLING);

  // TIME RECORD START
  lastmillis = millis();
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////              LOOP        ///////////////////////////////////////////////////////

void loop()
{
   //val=map(analogRead(ls2),50,810,0,70);
  //x=map(val,15,930,0,70);
  //Serial.println(val);
  //Serial.print("stop_flag=");
  //Serial.println(stop_flag);
  //Serial.print("prev2=");
  //Serial.println(prev2);
  
 
if(digitalRead(junctionls1))
{
  while(digitalRead(junctionls1));
  junction_countls1++;
  stop_countL2 = 0;
  stop_countR2  = 0;
  if(ctrls==1)
  {
    prev=junction_countls1;
    ctrls--;
  }
  //Serial.println(prev);
}
//////////////////////  FORWARD : for parts of motion when robot is moving forward ///////////////////////////////  
if(process == 'f')
 {
  FORWARD();
  line_follow_pid_bothls_forward();



  /////// For taking the left turn
  if(stop_flag == 0 && prev==1)
  {
    stop_counter_ls();
    
    basels = 80;
        //correct_orient();
  }
  

 ////////////// De-acceleration Code 
  /*  if(dist_countL2 > 550 && dist_countR2 > 550)
    {
      basels = abs(basels - 10);
      //basels = 60;
      //process = 'u';
      if(basels < 70)
      {
        basels = 70;
      }
      dist_countL1 = 0;
      dist_countR1 = 0;
    }*/
 }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    

////////////////////////// LEFT : For part of the motion where robot is turning left until line is detected /////////////////////    

   if(process == 'l')
   {

   
     ls1_value = map(analogRead(ls1),230,920,20,70);
     
/////////// For aligning front ls 
     if(stop_flag == 1)
     { 
     while(ls1_value < 70)
     {ls1_value = map(analogRead(ls1),230,920,20,70);
      LEFT();
      //Serial.println("check----1");
      //Serial.println(ls1_value);
     }
     ls1_value = map(analogRead(ls1),230,920,20,70);
      while(ls1_value >=65)
     {ls1_value = map(analogRead(ls1),230,920,20,70);
      LEFT();
      //Serial.println("Check-----2");
      //Serial.println(ls1_value);
     }
  digitalWrite(L1_CC,HIGH);
  digitalWrite(L1_CCW,HIGH);
  digitalWrite(R1_CCW,HIGH);
  digitalWrite(R1_CC,HIGH);
  delay(1000);
     /////////// For aligning back ls 
     ls2_value = map(analogRead(ls2),50,810,0,70);
     while(ls2_value > 65)
     {
     // Serial.println("check-----3");
      //Serial.println(ls1_value);
      ls2_value = map(analogRead(ls2),50,810,0,70);
  digitalWrite(L1_CC,HIGH);
  digitalWrite(L1_CCW,HIGH);
  digitalWrite(R1_CCW,HIGH);
  digitalWrite(R1_CC,HIGH);
  digitalWrite(L2_CCW,LOW);
  digitalWrite(L2_CC,HIGH);
  digitalWrite(R2_CC,LOW);
  digitalWrite(R2_CCW,HIGH);
     }
     
      
     // Process set to forward
      correct_orient();
      BRAKE();
      STOP();
      resetFunc();
        
     }
     }
     //else
     //   BRAKE();
       // STOP();
       //process='o';
        
        //stop_flag=0;
   //}
   
//////////////////////////////// This part ORIENTS the bot in correct orientation //////////////////////////////////////
    if(process == 'o')
    {
      correct_orient();
      if(stop_flag == 1) //// LEft turn
      {
        process = 'l';
        basels = 80;
      }
      else if(stop_flag == 0)
      {
        process = 'f';
        basels = 80;
      }
    }
    if(process=='s')
    {
      STOP();
    }
}
    

/////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////  Function for Line following using both ls    ////////////////////////////////////////////////////////////////////////////


//////////////////  Forward

void line_follow_pid_bothls_forward()
{
  ls1_value = map(analogRead(ls1),230,920,20,70);        // The LS value is MAPPED, LS MIN value <--> 0 and LS MAX value <--> 70 
  ls2_value = map(analogRead(ls2),50,810,0,70);

  
  if(digitalRead(junctionls1) != 1)                        // If Junction is NOT detected then PID tuning is done
  {
   // Serial.println(analogRead(ls1));
    //Serial.print("    ");
    if(ls1_value > 70)
    {   
         //Serial.print(analogRead(ls1));
         //Serial.println("    ");
    
         analogWrite(L1_PWM, 0);
         analogWrite(L2_PWM, 0);
         analogWrite(R1_PWM, 0);
         analogWrite(R2_PWM, 0);
    }
    else if(ls1_value <= 70)
    {

    last_ls1_value = ls1_value;
   // Serial.println(ls1_value);
    
    errorls = ls1_value - set_ls1_value;                  // Error is calc with set_ls1_value as 35;  If LS value is greater than 35
    derrls = errorls - lastls;
    pidls = kpls * errorls + kdls * derrls;

    right_pwmls = basels  - pidls;                      // If error is +ve
    left_pwmls  = basels  + pidls;

    // Scaling

    if(right_pwmls > maxspeedls) right_pwmls = maxspeedls;
    if(right_pwmls < 0) right_pwmls = 0;
    if(left_pwmls > maxspeedls) left_pwmls = maxspeedls;
    if(left_pwmls < 0) left_pwmls = 0;


    // LS2 correction routine
  if(ls2_value <= 70)
  {
    
    last_ls2_value = ls2_value;
     if(0 < ls2_value && ls2_value < 15)  
    corr_ls =  25;
    if(15 < ls2_value && ls2_value < 30)  
    corr_ls =  15;
    else if(30 < ls2_value && ls2_value < 40)   // BACK LS DISCRETE PID
     corr_ls = 0;
    else if(40 < ls2_value && ls2_value < 55)
     corr_ls = -15;
    else if(55 < ls2_value && ls2_value < 70) 
     corr_ls = -25;
 
  } 
    
      // Applying pwm

    analogWrite(L1_PWM, abs(left_pwmls));
    analogWrite(L2_PWM, abs(left_pwmls - corr_ls));
    analogWrite(R1_PWM, abs(right_pwmls));
    analogWrite(R2_PWM, abs(right_pwmls+30 + corr_ls));
    
    lastls = errorls;
  }

  }
  else
  {
         analogWrite(L1_PWM, 0);
         analogWrite(L2_PWM, 0);
         analogWrite(R1_PWM, 0);
         analogWrite(R2_PWM, 0);
  }
}
