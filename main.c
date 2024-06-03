#include "stm32f10x.h"

#define ADC1_DR_ADDRESS ((uint32_t)0x4001244C) // Base address + offset
#define BUFFER_SIZE 4
#define NUM_CHANNELS 4

#define TIM_Prescaler 	100; 			// Prescale timer clock from 72MHz to 720kHz by prescaler = 100
#define TIM_Period 			14399; 		// TIM_Period = (timer_clock / PWM_Frequency) - 1 = (720kHz / 50Hz) - 1 = 14399

uint16_t adcValue[BUFFER_SIZE];
volatile uint16_t adcKal[NUM_CHANNELS];
volatile int trungian1 = 0;
volatile int trungian2;
int ad;
int k=1000;

// Struct to store Kalman filter state
typedef struct {
    float Q; // Process noise covariance
    float R; // Measurement noise covariance
    float P; // Estimate error covariance
    float K; // Kalman gain
    float X; // Value
} KalmanState;

// Array of KalmanState for each ADC channel
KalmanState kalmanStates[NUM_CHANNELS];

// Initialize KalmanState for each channel
void Kalman_Init_All(void) {
    for (int i = 0; i < NUM_CHANNELS; i++) {
        kalmanStates[i].Q = 0.01;
        kalmanStates[i].R = 5;
        kalmanStates[i].P = 0;
        kalmanStates[i].K = 0;
        kalmanStates[i].X = 0;
    }
}

// Kalman filter update for a single value
float Kalman_Update(KalmanState* state, float measurement) {
    // Predict phase
    state->P = state->P + state->Q;

    // Update phase
    state->K = state->P / (state->P + state->R);
    state->X = state->X + state->K * (measurement - state->X);
    state->P = (1 - state->K) * state->P;

    return state->X;
}

// Process ADC values using Kalman filter
void Process_ADC_Values(void) {
    for (int i = 0; i < NUM_CHANNELS; i++) {
        adcKal[i] = (uint16_t)(Kalman_Update(&kalmanStates[i], adcValue[i]));
    }
}

void DMA_Config(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN; // Enable clock for DMA1

    // Disable DMA1_Channel1 before configuring it
    DMA1_Channel1->CCR &= ~DMA_CCR1_EN;

    // Set peripheral address to ADC1 data register
    DMA1_Channel1->CPAR = (uint32_t)ADC1_DR_ADDRESS;

    // Set memory address to the adcValue array
    DMA1_Channel1->CMAR = (uint32_t)&adcValue[0];

    // Set the number of data items to transfer
    DMA1_Channel1->CNDTR = BUFFER_SIZE;

    // Configure the DMA channel
    DMA1_Channel1->CCR = DMA_CCR1_PL_1       // Priority level: High
                       | DMA_CCR1_MSIZE_0    // Memory size: 16-bit
                       | DMA_CCR1_PSIZE_0    // Peripheral size: 16-bit
                       | DMA_CCR1_MINC       // Memory increment mode
                       | DMA_CCR1_CIRC       // Circular mode
                       | DMA_CCR1_EN;        // Enable DMA
}

void GPIO_Config(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2); // PA2
    GPIOA->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3); // PA3
    GPIOA->CRL &= ~(GPIO_CRL_CNF4 | GPIO_CRL_MODE4); // PA4
    GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5); // PA5
}

void ADC_Config(void) {
    // Enable ADC1 clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Configure ADC prescaler
    RCC->CFGR &= ~RCC_CFGR_ADCPRE;
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6; // Set ADC prescaler to 6

    // Configure ADC
    ADC1->CR1 = ADC_CR1_SCAN; // Enable scan mode
    ADC1->CR2 = ADC_CR2_CONT  // Enable continuous conversion mode
              | ADC_CR2_DMA   // Enable DMA mode
              | ADC_CR2_ADON; // Enable ADC

    // Set sequence length to 4 conversions
    ADC1->SQR1 = ((4 - 1) << 20); // L[3:0] = 4-1 = 3 (4 conversions)

    // Set sequence: PA2 -> PA3 -> PA4 -> PA5
    ADC1->SQR3 = (2 << 0)   // SQ1: ADC channel 1 (PA2)
               | (3 << 5)   // SQ2: ADC channel 2 (PA3)
               | (4 << 10)  // SQ3: ADC channel 3 (PA4)
               | (5 << 15); // SQ4: ADC channel 4 (PA5)

    // Set sample time for each channel (55.5 cycles)
    ADC1->SMPR2 = ADC_SMPR2_SMP2_1 | ADC_SMPR2_SMP2_0
                | ADC_SMPR2_SMP3_1 | ADC_SMPR2_SMP3_0
                | ADC_SMPR2_SMP4_1 | ADC_SMPR2_SMP4_0
                | ADC_SMPR2_SMP5_1 | ADC_SMPR2_SMP5_0;

    // Calibrate ADC
		
		ADC1->CR2 |= ADC_CR2_ADON;
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		ADC1->CR2 |= ADC_CR2_ADON;
	
    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while (ADC1->CR2 & ADC_CR2_RSTCAL);
    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL);

    // Start conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

void PWM_Timer2_Reister()
{
	// enable clock cho timer2
	RCC->APB1ENR |= 0x01;
	
	// dat gia tri reload, no tuong duong voi chon tan so, chon do rong xung 
	TIM2->ARR = 10000;
	
	// chia clock, set tgian cho 1 tick
	TIM2->PSC = 72*2-1;
	
	// chon pwm mode 1 (xung thuan) : dat bit [6:4] = 110
	TIM2->CCMR1 = 0x6060;
	
	/* dat do rong muc cao/thap mong muon, CCR1 la dung cho chan PA0, max la 4 chan 
	tuong duong CCR2,3,4 dieu khien cho PB0, PC0, PD0 */
	TIM2->CCR1 = 10000/20;
	TIM2->CCR2 = 10000/20;
	
	/* bit cc1e trong thanh ghi chon input/output cho dau phat
	con bit cc1p trong thanh ghi chon xem muc mong muon la muc cao hay muc thap (0:cao, 1:thap) */
	TIM2->CCER = 0x1111;
	
	// enable counter
	TIM2->CR1 = 0x01;
	
	// enable bit UDIS de cap nhap su kien
	TIM2->EGR = 0x01;
}

void ConfigAlternate(void)
{
	RCC->APB2ENR |= 0x04;
	GPIOA->CRL &= ~0xFF;
	GPIOA->CRL |= 0xBB;
}

void UART_Write(char ch)
{
		USART3->DR = (uint32_t)(ch & 0xFF); // ghi data
		while ((USART3->SR & USART_SR_TXE) != USART_SR_TXE);
}

void printString(const char *str)
{
	int len = strlen(str);
	for (int i = 0; i < len; i++)
	{
		UART_Write(str[i]);
	}
	UART_Write('\n');
}

void reverse(char *str, int length) {
    int start = 0;
    int end = length - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
}

 char* intToStringStatic(int num) {
    static char str[50]; // B? nh? tinh, s? b? ghi d? trong l?n g?i h?m ti?p theo
    int i = 0;
    int isNegative = 0;

    if (num < 0) {
        isNegative = 1;
        num = -num;
    }

    do {
        str[i++] = (num % 10) + '0';
        num = num / 10;
    } while (num != 0);

    if (isNegative) {
        str[i++] = '-';
    }

    str[i] = '\0';
    reverse(str, i);
    return str;
}

void Delay(uint32_t count)
{
	while(count--);
}

void UART_Config(void)
{
	RCC->APB2ENR |= 0x08; // enable clock portB
	RCC->APB1ENR |= 0x40000; // enable clock USART2
	
	GPIOB->CRH &= ~0xFF00; // clear PortA2A3
	GPIOB->CRH |= 0x4A00; // PA2:1010 PA3:0100
	
	USART3->CR1 |= 0x2000; // enable uart
	USART3->CR1 |= 0x08; // enable bit Tx
	
	USART3->BRR = 0x0EA6; // 9600 baud rate
}

void delayMs(uint32_t time)
{
	while (time--);
}
int abs(int x)
{
	if(x>0) return x;
	else return -x;
}
int main(void) {
  GPIO_Config();
  DMA_Config();
  ADC_Config();
  Kalman_Init_All();
	ConfigAlternate();
	PWM_Timer2_Reister();
	UART_Config();
	
	int tg1;
	int tg2;
	
	while(1)
	{
		printString(intToStringStatic(adcValue[0]+adcValue[1])+adcValue[2]+adcValue[3]);
		delayMs(10000);
		while((abs(adcValue[0] - adcValue[1]) > 50) || (abs(adcValue[2] - adcValue[3]) > 50))
		{
			if (trungian1 > 1000)
				trungian1 = 1000;
			else if (trungian1 <300) 
				trungian1 = 300;
			else
				trungian1 = TIM2->CCR1;
				
			if (trungian2 > 1250)
				trungian2 = 1250;
			else if (trungian2 <300) 
				trungian2 = 300;
			else
				trungian2 = TIM2->CCR2;
			
			if(abs(adcValue[0] - adcValue[1]) > 100)
			{
				if(adcValue[0] > adcValue[1]) 
					tg1 = 1;
				else 
					tg1 = -1;
				TIM2->CCR1 = trungian1 + 10*tg1;
				delayMs(10000);
			}
			
			if(abs(adcValue[2] - adcValue[3]) > 100)
			{
				if(adcValue[2] < adcValue[3]) 
					tg2 = 1;
				else 
					tg2 = -1;
				TIM2->CCR2 = trungian2 + 5*tg2;
				delayMs(10000);
			}
			printString(intToStringStatic(adcValue[0]));
			delayMs(100000);
		}
	}
}
