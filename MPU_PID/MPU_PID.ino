/*
 * testMPU.c
 *
 * Created: 3/27/2024 3:23:21 PM
 * Author : KHANH
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>


#define STEP 1
#define DIR 0
#define  DIR_2 3
#define OUTPUT_DDR DDRB
#define OUTPUT_PORT PORTB
#define max_rpm 380
#define mpu_address 0x68 << 1
#define RAD2DEG 57.295779
#define offset 0


//Bo dieu khien
#define x1_NB 0
#define x1_NS 1
#define x1_ZE 2
#define x1_PS 3
#define x1_PB 4

#define x2_NB 0
#define x2_NS 1
#define x2_ZE 2
#define x2_PS 3
#define x2_PB 4

#define y_NB 0
#define y_NM 1
#define y_NS 2
#define y_ZE 3
#define y_PS 4
#define y_PM 5
#define y_PB 6


volatile char flag;
int8_t Dir_M1, Dir_M2, Dir_M3;                                               //Biến xác định hoạt động của động cơ và chiều quay Dir_Mx >0 quay thuận , Dir_Mx <0 quay ngược Dir_Mx =0 motor ngừng quay
volatile int16_t Count_timer1, Count_timer2, Count_timer3;                       //đếm các lần TIMER xuất hiện trong chu kỳ xung STEP
volatile int32_t Step1, Step2, Step3;
int16_t Count_TOP1, Count_BOT1, Count_TOP2, Count_BOT2, Count_TOP3, Count_BOT3;  //vị trí cuối của phần cao và cuối phần thấp trong 1 chu kỳ xung STEP

float fuzzyspeed;
float accRawX, accRawY, accRawZ, gyroRawX, accAngleX, accRawAngleX, AngleX, input, inputdot, inputlast;
float gyroX, ePos, edotPos, ePoslast, angleRef;

void mpu_init(void);
int mpu_read(unsigned char reg);
void mpu_write(unsigned char reg, unsigned char data);
void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(unsigned char data);
unsigned char i2c_read(void);
void i2c_nak(void);
void timer0_init(void);


float fuzzyControllerAngle(float a, float b);
float fuzzyControllerPos(float a, float b);
float hlt_hinhthang(float data,float l, float cl, float cr, float r);
float hlt_tamgiac(float data,float l,float m, float r);
float sum_array (float data[][5],char n, char m);
float MIN(float a,float b);
float rule(float data[],float val,char n);
float Saturation(float x);


void Set_up_A4988();
void Fuzzy_Speed_direction(float v);//(-1<v<1)
void Speed(int x);//(-200<y<200)
void Turn_on_Timer2();


ISR(TIMER0_COMPA_vect)
{
  flag = 1;
}


int main(void)
{
  sei();
  i2c_init();
  mpu_init();
  timer0_init();
  Set_up_A4988();
  Serial.begin(115200);
  float filter_input = 0, filter_inputdot = 0;
  float filter_ePos = 0, filter_edotPos = 0;
  float tau1 = 0.08, tau2 = 0.08, tau3 = 0, tau4 = 0.032;

  while (1)
  {
    if (flag)
    {
      ePos = (0 - Step1);   
      edotPos = (ePos - ePoslast)/10;
      ePoslast = ePos;
      filter_ePos = (0.01 * ePos / 6000 + tau3 * filter_ePos) / (tau3 + 0.01);
      filter_edotPos = (0.01 * edotPos / 580 + tau4 * filter_ePos) / (tau4 + 0.01);
      angleRef = fuzzyControllerPos(Saturation(filter_ePos),Saturation(filter_edotPos))*8;
      Serial.print(ePos);
      Serial.print(' ');
      Serial.print(angleRef);
      Serial.print(' ');
      accRawX = mpu_read(59);
      accRawY = mpu_read(61);
      accRawZ = mpu_read(63);
      gyroRawX = mpu_read(67);
      accRawAngleX = atan2( accRawY,sqrt(pow(accRawX,2) + pow(accRawZ,2))) * RAD2DEG;
      AngleX = 0.98 * (AngleX + gyroRawX / 131 * 0.01) + 0.02 * accRawAngleX; // unit degree
      input = angleRef - (AngleX + offset);
      inputdot = (input - inputlast) / 10;
      inputlast = input;
      filter_input = (0.01 * input / 24 + tau1 * filter_input) / (tau1 + 0.01);
      filter_inputdot = (0.01 * inputdot / 0.8 + tau2 * filter_inputdot) / (tau2 + 0.01);
      Serial.print(input);
      Serial.print(' ');
      Serial.print(filter_input);
      Serial.print(' ');
      fuzzyspeed = fuzzyControllerAngle(Saturation(filter_input),Saturation(filter_inputdot));
      Serial.println(fuzzyspeed);
      Fuzzy_Speed_direction(fuzzyspeed);
      flag = 0;
    }
  }
}


void mpu_init(void)
{
  mpu_write(107,0); // reset MPU6050
  mpu_write(25,0); // Sample Rate = Gyroscope Output Rate / (0 + 1) = 1000 Hz
  mpu_write(26,6); // DLPF = 10 Hz / 10 Hz
  mpu_write(27,0); // Gyroscope full scale range +-250
  mpu_write(28,0); // Accelerometer full scale range +-2g
}


int mpu_read(unsigned char reg)
{
  int out;
  i2c_start();
  i2c_write(mpu_address);
  i2c_write(reg);
  i2c_start();
  i2c_write(mpu_address|1);
  out = i2c_read() << 8;
  out |= i2c_read();
  i2c_nak();
  i2c_stop();
  return out;
}


void mpu_write(unsigned char reg, unsigned char data)
{
  i2c_start();
  i2c_write(mpu_address);
  i2c_write(reg);
  i2c_write(data);
  i2c_stop();
}


void i2c_init(void)
{
  TWBR = 0x08; // set SCL f = 100KHz
  TWSR = 1;
  TWCR = (1 << TWEN); // module TWI enabled
}


void i2c_start(void)
{
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0);
}


void i2c_stop(void)
{
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}


void i2c_write(unsigned char data)
{
  TWDR = data;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0);
}


unsigned char i2c_read(void)
{
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
  while ((TWCR & (1 << TWINT)) == 0);
  return TWDR;
}


void i2c_nak(void)
{
  TWCR = (1 << TWINT) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0);
}


void timer0_init(void)
{
  OCR0A = 155;
  TCCR0A = (1 << WGM01);
  TIMSK0 = (1 << OCIE0A);
  TCCR0B = (1 << CS02) | (1 << CS00);
}


//bo dieu khien
float Saturation(float x)
{
  if(x>=1)
  x=1;
  else if(x<=-1)
  x=-1;
  return x;
}


float fuzzyControllerAngle(float a, float b)
{
    float x1[]={hlt_hinhthang(a,-1.2,-1,-0.3,-0.18),hlt_tamgiac(a,-0.3,-0.18,0),hlt_tamgiac(a,-0.18,0,0.18),hlt_tamgiac(a,0,0.18,0.3),hlt_hinhthang(a,0.18,0.3,1,1.2)};
    float x2[]={hlt_hinhthang(b,-1.2,-1,-0.5,-0.3),hlt_tamgiac(b,-0.5,-0.3,0),hlt_tamgiac(b,-0.3,0,0.3),hlt_tamgiac(b,0,0.3,0.5),hlt_hinhthang(b,0.3,0.5,1,1.2)};
    float y[]={-1,-0.8,-0.6,0,0.6,0.8,1};
    float beta[5][5];


    for (int i = 0; i < 5; i++)
    {
     for (int j = 0; j < 5 ;j++)
     {
       beta[i][j]= MIN(x1[i],x2[j]);
     }
    }


    float PB[] = {beta[x1_NB][x2_NB],beta[x1_NB][x2_NS],beta[x1_NS][x2_NB]};
    float PM[] = {beta[x1_NB][x2_ZE],beta[x1_NS][x2_NS],beta[x1_ZE][x2_NB]};
    float PS[] = {beta[x1_NB][x2_PS],beta[x1_NS][x2_ZE],beta[x1_ZE][x2_NS],beta[x1_PS][x2_NB]};
    float ZE[] = {beta[x1_NB][x2_PB],beta[x1_NS][x2_PS],beta[x1_ZE][x2_ZE],beta[x1_PS][x2_NS],beta[x1_PB][x2_NB]};
    float NS[] = {beta[x1_NS][x2_PB],beta[x1_ZE][x2_PS],beta[x1_PS][x2_ZE],beta[x1_PB][x2_NS]};
    float NM[] = {beta[x1_ZE][x2_PB],beta[x1_PS][x2_PS],beta[x1_PB][x2_ZE]};
    float NB[] = {beta[x1_PS][x2_PB],beta[x1_PB][x2_PB],beta[x1_PB][x2_PS]};


    float ans = (rule(NB,y[y_NB],3)+rule(NM,y[y_NM],3)+rule(NS,y[y_NS],4)+rule(ZE,y[y_ZE],5)+rule(PS,y[y_PS],4)+rule(PM,y[y_PM],3)+rule(PB,y[y_PB],3))/sum_array(beta,5,5);


    return ans;
}

float fuzzyControllerPos(float a, float b)
{
    float x1[]={hlt_hinhthang(a,-1.2,-1,-0.6,-0.3),hlt_tamgiac(a,-0.6,-0.3,0),hlt_tamgiac(a,-0.3,0,0.3),hlt_tamgiac(a,0,0.3,0.6),hlt_hinhthang(a,0.3,0.6,1,1.2)};
    float x2[]={hlt_hinhthang(b,-1.2,-1,-0.4,-0.3),hlt_tamgiac(b,-0.4,-0.3,0),hlt_tamgiac(b,-0.3,0,0.3),hlt_tamgiac(b,0,0.3,0.4),hlt_hinhthang(b,0.3,0.4,1,1.2)};
    float y[]={-1,-0.6,-0.4,0,0.4,0.6,1};
    float beta[5][5];


    for (int i = 0; i < 5; i++)
    {
     for (int j = 0; j < 5 ;j++)
     {
       beta[i][j]= MIN(x1[i],x2[j]);
     }
    }


    float NB[] = {beta[x1_NB][x2_NB],beta[x1_NB][x2_NS],beta[x1_NS][x2_NB]};
    float NM[] = {beta[x1_NB][x2_ZE],beta[x1_NS][x2_NS],beta[x1_ZE][x2_NB]};
    float NS[] = {beta[x1_NB][x2_PS],beta[x1_NS][x2_ZE],beta[x1_ZE][x2_NS],beta[x1_PS][x2_NB]};
    float ZE[] = {beta[x1_NB][x2_PB],beta[x1_NS][x2_PS],beta[x1_ZE][x2_ZE],beta[x1_PS][x2_NS],beta[x1_PB][x2_NB]};
    float PS[] = {beta[x1_NS][x2_PB],beta[x1_ZE][x2_PS],beta[x1_PS][x2_ZE],beta[x1_PB][x2_NS]};
    float PM[] = {beta[x1_ZE][x2_PB],beta[x1_PS][x2_PS],beta[x1_PB][x2_ZE]};
    float PB[] = {beta[x1_PS][x2_PB],beta[x1_PB][x2_PB],beta[x1_PB][x2_PS]};


    float ans = (rule(NB,y[y_NB],3)+rule(NM,y[y_NM],3)+rule(NS,y[y_NS],4)+rule(ZE,y[y_ZE],5)+rule(PS,y[y_PS],4)+rule(PM,y[y_PM],3)+rule(PB,y[y_PB],3))/sum_array(beta,5,5);


    return ans;
}

float hlt_hinhthang(float data,float l, float cl, float cr, float r)
{
    if ((data < l) && (data >= r)) return 0;
    if ((data >= l) && (data < cl)) return (data - l)/(cl-l) ;
    if ((data >= cl) && (data < cr)) return 1   ;
    if ((data >= cr) && (data < r)) return (r - data)/(r-cr);
    return 0;
}


float hlt_tamgiac(float data,float l,float m, float r)
{
    if ((data < l) || (data >= r)) return 0;
    if ((data >= l) && (data < m)) return (data-l)/(m-l);
    if ((data >= m) && (data < r)) return (r - data)/(r-m);
    return 0;
}


float sum_array (float data[][5],char n, char m)
{
    float s=0;
    for (int i =0;i<n;i++)
    {
     for (int j=0;j<m;j++)
     {
       s+=data[i][j];
     }
    }
    return s;
}


float MIN(float a,float b)
{
    if (a<b) return a;
    else return b;
}


float rule(float data[],float val,char n)
{
    float s = 0;
    for (int i=0;i<n;i++)
    {
     s+=data[i]*val;
    }
    return s;
}


//hau xu ly
void Set_up_A4988()
{
    OUTPUT_DDR=(1<<STEP)|(1<<DIR)|(1<<DIR_2);
    Turn_on_Timer2();
}
void Fuzzy_Speed_direction(float v)//(-100<v<100)
{
  float a;
  if (v != 0)
  {
    float pps=v*max_rpm*10/3*16;
    a=50000/pps;
  }
  else a = 0;
  Speed((int)a);
}
void Speed(int x)//(-200<y<200)
{
    if (x < 0) {
    Dir_M1 = -1;
  }
  else if (x > 0) {
    Dir_M1 = 1;
    
  }
  else Dir_M1 = 0;
  Count_BOT1 = abs(x);
  Count_TOP1 = 1;
}
void Turn_on_Timer2()
{
  TCCR2A=(1<<WGM21);
  OCR2A=39;
  TCCR2B=(1<<CS21);
  TIMSK2=(1<<OCIE2A);
}


///phan ngat
ISR(TIMER2_COMPA_vect)
{
  if (Dir_M1 != 0) {                                                          //nếu MOTOR cho phép quay
    if (Dir_M1 > 0)
      {
        PORTB |= (1<<DIR);
        PORTB &=~(1<<DIR_2);
      }
    else if (Dir_M1 < 0)
    {
      PORTB |=(1<<DIR_2);
    PORTB&=~(1<<DIR);
    }
    Count_timer1++;
    if (Count_timer1 <= Count_TOP1)PORTB |=(1<<STEP) ;                        //nếu là nhịp nằm trong phần cao trong xung STEP
    else PORTB &=~(1<<STEP);                                                 //nếu là nhịp nằm trong phần thấp của xung STEP
    if (Count_timer1 > Count_BOT1) {
      Count_timer1 = 0;                             //nếu là nhịp cuối của 1 xung STEP
      if (Dir_M1 > 0)Step1++;
      else if (Dir_M1 < 0)Step1--;
    }
}
}