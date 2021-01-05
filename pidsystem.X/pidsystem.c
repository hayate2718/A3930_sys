// PIC16F1827 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include<stdio.h>

//tim1の上のレジスタのカウント時間は32usこれをカウントアップ時間として使う
//tim0のオーバーフローで制御周期を与える
//ccp1がオーバーフローした場合は停止とみなす
#define dutyL CCP2CONbits.DC2B //10bitPWMの下位2bit
#define dutyH CCPR2L //10bitPWMの下位　格納可能なのは63まで



//グローバル変数宣言
const double kp = 15; //pゲインを設定する
const double ki = 1.5; //iゲインを設定する
const double kd = 10; //dゲインを設定する
const unsigned long int alpha = 52083; //20/(磁極の数*カウント時間) 
const int dutyconvers = 1; // 255/リポの電圧　電圧duty変換
long int derr = 1000; //微分制御の1周期前の値を入れる変数
long int ierr = 0; //積分制御の加算の変数
unsigned long int count = 0; //timer1のカウントを入れる変数
long int speed_tar = 1000; //目標速度の設定 保証領域400～3000
long int pidsum = 0; //pid制御の操作量を加算する変数
char sysf = 1; //制御周期まで待つためのフラグ
char countstop = 1; //timer1がオーバーフローしたときの処理用変数
char pid_f = 1; //pid制御の低速域のバグ除去
char tmr1_of = 1; //timer1オーバーフロー

//関数宣言
void picinit();
void __interrupt() isr(void);
long int pidsys();
long int hensa(long int rpm, unsigned long int cou); 
unsigned int  ffsys(long int rpm);
long int gen_duty();
void set_duty();

//関数
//pic初期化
void picinit(){
    //クロック周波数設定
    OSCCON = 0b01110000;
    //割り込み設定
    INTCON = 0b11100000;
    PIE1 = 0b00000101;
    PIE2 = 0;
    PIE3 = 0;
    PIE4 = 0;
    //timer0設定
    OPTION_REG = 0b00000101; //64msでtim0がオーバーフロー
    //timer1設定
    T1CON = 0b00000000;
    T1GCON = 0;
    //ccp1の設定
    CCP1CON = 0b00000101;
    //pwm(ccp2)の設定
    CCP2CON = 0b00001100;
    CCPTMRS = 0;//timer2を使用
    PSTR2CON = 0b00000001;
    PR2 = 63; //pwm周波数
    //timer2の設定
    T2CON = 0b00000100;
    //io設定
    APFCON0 = 0b00001001;
    TRISA = 0b00000000;
    ANSELA = 0;
    TRISB = 0b00000001;
    ANSELB = 0;
}

//偏差を計算
long int hensa(long int rpm,unsigned long int cou){
    long int er = 0;
    unsigned long int speed = 0;
    pid_f = 1;
    if(cou > 250 || cou == 0 || tmr1_of == 1){
        speed = 0;
        count = 0;
        pid_f = 0;
        tmr1_of = 0;
    }else{
        speed = alpha/cou;
    }
    er = rpm - speed;
    return er;
}

//pidの操作量を計算
long int pidsys(long int err){
    long int pid = 0;
    long int p = 0;
    long int i = 0;
    long int d = 0;
    p = kp*err;
    i = ki*(ierr + err);
    d = kd*(err - derr);
    pid = p + pid_f * i + d; 
    pid = pid >> 8; //kv値で割ってる(270kv=256)
    if(pid >= 255){
        pid = 255;
    }else if(pid <= -255){
        pid = -255;
    }
    ierr = ierr + err;
    if(ierr > 4000){
        ierr = 4000;
    }
    derr = err;
    return pid;
}

//ff制御操作量の計算
unsigned int ffsys(long int rpm){
    unsigned long int ffduty = 0;
    ffduty = 21*rpm;
    ffduty = ffduty >> 8;
    return ffduty;
}

//duty指令値の計算
long int gen_duty(){
    int duty = 0;
    int pid_val = 0;
    pid_val = pidsys(hensa(speed_tar,count));
    pidsum = pidsum + pid_val;
    if(pidsum > 255){
        pidsum = 255;
    }
    duty = pidsum + ffsys(speed_tar);;
    if(duty <= 0){
        duty = 0;
    }else if(duty >= 255){
        duty = 255;
    }
    return duty;
}

//dutyを反映する関数
void set_duty(){
    unsigned int duty = 0;
    duty = gen_duty();
    CCP2CONbits.DC2B = duty & 0x0003;
    CCPR2L = duty >> 2;
}

void __interrupt() isr(void){
    INTCONbits.GIE = 0;
    if(PIR1bits.CCP1IF){
        if(CCP1CONbits.CCP1M == 5){
            TMR1 = 0;
            CCP1CONbits.CCP1M = 4;
            PIR1bits.CCP1IF = 0;
        }else{
            count = TMR1 >> 6;
            TMR1 = 0;
            CCP1CONbits.CCP1M = 5;
            PIR1bits.CCP1IF = 0;
        }
    }else if(PIR1bits.TMR1IF){
        tmr1_of = 1;
        PIR1bits.TMR1IF =0;
    }else if(INTCONbits.TMR0IF){
        sysf = 0;
        INTCONbits.TMR0IF = 0;
    }
    INTCONbits.GIE = 1;
}

void main(){
    picinit();
    T1CONbits.TMR1ON = 1;
    while(1){
        set_duty();
        while(sysf);
        sysf = 1;
    }
}