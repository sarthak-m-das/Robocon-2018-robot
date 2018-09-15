
//TSOP
int tsop=11;
int ctr=0;
int time_delay=0;
//interrupt var
volatile int rpmcount =0;
int count=0;
int flag=1;
// Target rpm
float rpm =0.0;
float setp = 140;

int signal_flag1=8;
int signal_flag2=9;

// Motor info
int pwm = 6;
int dir1 = 12;
int dir2 = 11;

// Time recorder
unsigned long lastmillis = 0;
unsigned long lastmillis2 = 0;

// PID params
float kp = 0.00002;   //0.00002;
float kd =    0.04;        //0.0000052;
float ki =0.000025;
float base = 35;
int tsop1=8;

// error vars
float error = 0.0;
float last = 0.0;
float derr = 0.0;
float pid = 80;
float mult = 1;
double err_sum = 0;
double max_err_sum = 400;

int start_pid = 0;

/////////Functions////////
void rpm_update();
void cal_and_set_rpm1();
void(*resetFunc)(void)=0;
char process='s';

void setup() {
  Serial.begin(9600);
  pinMode(signal_flag1,INPUT);
  pinMode(signal_flag2,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(2),rpm_update,RISING);
  pinMode(pwm, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, LOW);
  digitalWrite(signal_flag2,LOW);
  //lastmillis2=millis();
}

void loop() {
  
  if(!digitalRead(signal_flag1))
   {
    Serial.println("no"); 
      process='s';
    }
    else
   { 
      process='o';
      Serial.println("Recieved");
   }
if(process=='o')
{
  Serial.println("1");
  if(Serial.available()>0)
  {
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, LOW);
  digitalWrite(signal_flag2,HIGH);
  Serial.println("Done");
  flag=3;
  }
  if(flag==1)
  {
  delay(3000);
  Serial.write(1);
  delay(1000);
  flag=2;
  }
  if(flag==2)
  {
    Serial.println("Rotate  ");
      cal_and_set_rpm1();    
      Serial.write(2);
  }
  if(flag==3)
  {
    resetFunc();
  }
    
}
}

void rpm_update() { /* this code will be executed every time the interrupt 0 (pin2) gets low.*/
  rpmcount++;
  count++;
  }

void cal_and_set_rpm1()
{
  //**************** Open loop pwm

  
 
  //float setp = 140;//set rpm
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, HIGH);
  //analogWrite(pwm,255); 
  if (millis() - lastmillis >= 100) { /*Uptade every one second, this will be equal to reading frecuency (Hz).*/

    detachInterrupt(2);    //Disable interrupt when calculating


    rpm = (rpmcount * 600.0) / 135.0 ; /* Convert frecuency to RPM, note: this works for one interruption per full rotation. For two interrups per full rotation use rpmcount * 30.*/
  //Serial.println(pid);
  //Serial.print("\nerror =\t  ");
  //Serial.println(error);
  //Serial.println(" ");
   //Serial.print("RPM =\t"); //print the word "RPM" and tab
   //Serial.println(rpm); // print the rpm value.
    //Serial.print("\t Hz=\t"); //print the word "Hz".
    //Serial.println(pid/*rpmcount*/); /*print revolutions per second or Hz. And print new line or enter.*/
    rpmcount = 0; // Restart the RPM counter
    lastmillis = millis(); // Uptade lasmillis
    attachInterrupt(2, rpm_update, RISING); //enable interrupt
  }
  error = (setp - rpm);
  derr = last - error;

  pid = pid + (kp * error) + (kd * derr) + (ki * err_sum) ;
  /*if (pid<50)
  {
    pid=70;
  }*/

  if(start_pid == 0)
  {
    //long int tim = millis();
    analogWrite(pwm, 100);
    delay(2000);
    start_pid = 1;
  }
  if(start_pid == 1)
  {
  last = error;
  err_sum = err_sum + error;
  if (err_sum > max_err_sum)
  {
    err_sum = max_err_sum;
  }
  if (err_sum < -max_err_sum)
  {
    err_sum = -max_err_sum;
  }
  if (pid < 255 && pid > 0) {
    analogWrite(pwm, (int)pid); //set motor speed
  }
  else {
    if (pid > 255) {
      //pid = 255;
      analogWrite(pwm, 255);
    }
    if (pid < 0) {
      //pid = 0;
      analogWrite(pwm, 0);
    }


  }
  }
}

