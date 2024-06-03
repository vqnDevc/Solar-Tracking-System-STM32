#include "stm32f10x.h"
#include "string.h"

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
    static char str[50]; // B? nh? tinh, s? b? ghi dè trong l?n g?i hàm ti?p theo
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

int main(void)
{
	UART_Config();
	while(1)
	{
		for (int i = 0; i < 4 ; i++)
		{
			printString(intToStringStatic(i + 1234));
		}
		
		Delay(10000000);
	}
	
}