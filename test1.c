
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"

/*******************************************************************/
#define BAUDRATE 9600

//команда логическому анализатору, чтобы он нам показал сигнал на выходе нулевого бита порта A. 
//la porta&0x01
//седьмой бит того же порта, надо было бы написать
//la porta&0х80

/** \brief Элементы, принимающие или посылающие данные в BT-канал
 *
 *  Команды посылаемые от Пульта к КМУ: 0x01 - 0x7F
 *  Команды посылаемые от КМУ к Пульту: 0x80 - 0xFF
 */
 enum Element
{
    powerButton     = 0x01,     /**< Питание (вкл/выкл) (1/0) */
    lightButton     = 0x02,     /**< Свет (вкл/выкл) (1/0) */
    soundSignal     = 0x03,     /**< Гудок (вкл/выкл) (1/0) */
    speedButton     = 0x04,     /**< Скорость (медленно/быстро) (0/1..9) */
    pillarUp        = 0x05,     /**< Поворот стойки - по часовой стрелке (1..9) */
    pillarDown      = 0x06,     /**< Поворот стойки - против часовой стрелке (1..9) */
    derrickUp       = 0x07,     /**< Подъемная стрела - подъем (1..9) */
    derrickDown     = 0x08,     /**< Подъемная стрела - опускание (1..9) */
    outriggerUp     = 0x09,     /**< Выносная стрела - подъем (1..9) */
    outriggerDown   = 0x0A,     /**< Выносная стрела - опускание (1..9) */
    telescopicUp    = 0x0B,     /**< Телескопическая стрела - выдвижение (1..9) */
    telescopicDown  = 0x0C,     /**< Телескопическая стрела - втягивание (1..9) */
    hookUp          = 0x0D,     /**< Лебедка(крюк) - подъем (1..9) */
    hookDown        = 0x0E,     /**< Лебедка(крюк) - опускание (1..9) */
    leftCrutchUp    = 0x0F,     /**< Левая опора - подъем (1..9) */
    leftCrutchDown  = 0x10,     /**< Левая опора - опускание (1..9) */
    rightCrutchUp   = 0x11,     /**< Правая опора - подъем (1..9) */
    rightCrutchDown = 0x12,     /**< Правая опора - опускание (1..9) */

    powerStatus     = 0x80,     /**< Питание (вкл/выкл) (1/0) */
    lightStatus     = 0x81,     /**< Свет (вкл/выкл) (1/0) */                   
    highTemperature = 0x54,     /**< Перегрев ОЖ (нет/есть) (0/1) */																		//********* 0x82
    hookWarning     = 0x48,     /**< Ограничитель подъема (срабатывание/норм.работа) (1/0) */						//********* 0x83

		on     					= 0x31,     /**< включить */																												//********* 0x01
		off    					= 0x30,     /**< выключить */																												//********* 0x00
    incorrectCommand = 0xFF,    /**< Некорректная команда */
};
/*******************************************************************/
GPIO_InitTypeDef port;
USART_InitTypeDef usart;
//Переменная для хранения передаваемых данных
uint8_t usartData[2];
uint16_t button;
bool receivedStatusHook=FALSE;
 
/*******************************************************************/
void initAll()
{
    //Включаем тактирование
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
    //Пины PA9 и PA10 в режиме альтернативных функций –
    //Rx и Tx USART’а
    GPIO_StructInit(&port);
    port.GPIO_Mode = GPIO_Mode_AF_PP;// Alternate function push-pull 
    port.GPIO_Pin = GPIO_Pin_9;// Девятый вывод порта
    port.GPIO_Speed = GPIO_Speed_2MHz;// Скорость на 2Мгц
    GPIO_Init(GPIOA, &port);
 
    port.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    port.GPIO_Pin = GPIO_Pin_10;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &port);
 
    //Настройка USART, все поля оставляем дефолтными, кроме скорости обмена
    USART_StructInit(&usart);
    usart.USART_BaudRate = BAUDRATE;
    USART_Init(USART1, &usart);
    //Запускаем сам USART
    USART_Cmd(USART1, ENABLE);
    //Включаем прерывания по приему байта
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART1_IRQn);
 
		//Здесь будет висеть датчик перегрева ОЖ (PA_0) ***********
    GPIO_StructInit(&port);
    port.GPIO_Mode = GPIO_Mode_IPD;
    port.GPIO_Pin = GPIO_Pin_0;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &port);
		// настройка прерывания
    AFIO->EXTICR[1] = AFIO_EXTICR1_EXTI0_PA;//настройка порта и выбор пина назначенного для прерывания
    EXTI->FTSR |= EXTI_FTSR_TR0; //разрешаем срабатывание по заднему фронту 
		EXTI->RTSR |= EXTI_RTSR_TR0; //разрешаем срабатывание по переднему фронту
    EXTI->IMR |= EXTI_IMR_MR0; // использовать для прерывания линию 0
    NVIC_EnableIRQ(EXTI0_IRQn);		
		// Объявляем структуру для настройки вывода отладки для сигнализирования срабатывания прерывания от датчика ОЖ
    GPIO_StructInit(&port);
		port.GPIO_Mode = GPIO_Mode_Out_PP; // Режим двухтактный выход
		port.GPIO_Pin = GPIO_Pin_0; // Выводы
		port.GPIO_Speed = GPIO_Speed_2MHz; // Макс. частота
		GPIO_Init(GPIOB, &port);// Настроить выводы порта B
		
		//Здесь будет висеть концевой датчик лебедки (PA_1) ************
    GPIO_StructInit(&port);
    port.GPIO_Mode = GPIO_Mode_IPD;
    port.GPIO_Pin = GPIO_Pin_1;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &port);
		// настройка прерывания
    AFIO->EXTICR[1] = AFIO_EXTICR1_EXTI1_PA;//настройка порта и выбор пина назначенного для прерывания
    EXTI->FTSR |= EXTI_FTSR_TR1; //разрешаем срабатывание по заднему фронту 
		EXTI->RTSR |= EXTI_FTSR_TR1; //разрешаем срабатывание по переднему фронту
    EXTI->IMR |= EXTI_IMR_MR1; // использовать для прерывания линию 0
    NVIC_EnableIRQ(EXTI1_IRQn);		
		// Объявляем структуру для настройки вывода отладки для сигнализирования срабатывания прерывания от датчика ОЖ
    GPIO_StructInit(&port);
		port.GPIO_Mode = GPIO_Mode_Out_PP; // Режим двухтактный выход
		port.GPIO_Pin = GPIO_Pin_1; // Выводы
		port.GPIO_Speed = GPIO_Speed_2MHz; // Макс. частота
		GPIO_Init(GPIOB, &port);// Настроить выводы порта B
		
 
}
/*******************************************************************/
//Отправка данных в порт USART1
void sendUsartData()
{
    USART_SendData(USART1, usartData[0]);
		/* Loop until the end of transmission */
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		{
		}
		
    USART_SendData(USART1, usartData[1]);
		/* Loop until the end of transmission */
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		{
		}
}
/*******************************************************************/

/*******************************************************************/
//Функция при срабатывании концевого датчика лебедки(PA_1)
void setHookWarning(bool status)
{
    if(status)
    {
				//срабатывание датчика
        usartData[0] = hookWarning;
        usartData[1] = on;
    }
    else
    {
        usartData[0] = hookWarning;
        usartData[1] = off;
    };
		sendUsartData();
}
/*******************************************************************/


/*******************************************************************/
//Функция при срабатывании датчика перегрева(PA_0)
void setTemperatureWarning(bool status)
{
    if(status)
    {
				//срабатывание датчика
        usartData[0] = highTemperature;
        usartData[1] = on;
    }
    else
    {
			//кнопка отжата
        usartData[0] = highTemperature;
        usartData[1] = off;
    }
		sendUsartData();
}

int main()
{
    __enable_irq ();
    initAll();
    while(1)
    {
    }
}
/*******************************************************************/
 
/*******************************************************************/
//Прерывание при получении данных с UART1
void USART1_IRQHandler()
{
		// Обработка события RXNE
    if ( USART_GetITStatus(USART1, USART_IT_RXNE) ) {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        // ... Код обработчика ...
    };

    // Обработка события TXE
    if ( USART_GetITStatus(USART1, USART_IT_TXE) ) {
        USART_ClearITPendingBit(USART1, USART_IT_TXE);
        // ... Код обработчика ...
    };

    // Обработка других событий... 

}
/*******************************************************************/

/*******************************************************************/
//Прерывание при от GPIO PA_0
void EXTI0_IRQHandler(void)
{
    if (EXTI->PR & (1<<0)) //проверяем прерывание от EXTI0
    {
				button = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
				if(button == 1 && receivedStatusHook == 0)
				{
					GPIO_SetBits(GPIOB, GPIO_Pin_0);
					setTemperatureWarning(TRUE);
				}
				else
				{
					GPIO_ResetBits(GPIOB, GPIO_Pin_0);
					setTemperatureWarning(FALSE);
				}
        EXTI->PR |= EXTI_PR_PR0; //сброс флага прерывания
    }
}
/*******************************************************************/

/*******************************************************************/
//Прерывание при от GPIO PA_1
void EXTI1_IRQHandler(void)
{
    if (EXTI->PR & (1<<1)) //проверяем прерывание от EXTI1
    {
				button = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);
				if(button == 1 && receivedStatusHook == 0)
				{
					GPIO_SetBits(GPIOB, GPIO_Pin_1);
					setHookWarning(TRUE);
				}
				else
				{
					GPIO_ResetBits(GPIOB, GPIO_Pin_1);
					setHookWarning(FALSE);
				}
        EXTI->PR |= EXTI_PR_PR1; //сброс флага прерывания
    }
}
/*******************************************************************/

/****************************End of file****************************/
