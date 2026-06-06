#include "stm32f10x.h"

#include "USART.h"

//作通讯时状态机之用
//0为空闲,1为读取中,2为读到尾部但数据还未取走

//电脑端可以通过COM7传出控制口将蓝牙与电脑进行配对。
volatile uint8_t isUSART3_TState = 0;

volatile uint8_t isUSART3_DState = 0;

volatile uint8_t USART3Flag_RX = 0;

volatile uint16_t U3Data = 0;

volatile uint16_t BufferCnt = 0;

#define USART3_BUFFER_SIZE 128
#define USART3_TX_BUFFER_SIZE 512

int8_t DataBuffer[128];

char TextBuffer[USART3_BUFFER_SIZE];
static char ReadyTextBuffer[USART3_BUFFER_SIZE];
static char LastTextBuffer[USART3_BUFFER_SIZE];
static volatile uint8_t ReadyTextValid = 0;

static uint8_t TxBuffer[USART3_TX_BUFFER_SIZE];
static volatile uint16_t TxHead = 0;
static volatile uint16_t TxTail = 0;

static uint16_t USART3_NextTxIndex(uint16_t index)
{
	index++;
	if(index >= USART3_TX_BUFFER_SIZE)
	{
		index = 0;
	}
	return index;
}

static void CopyText(char *dest, const char *src, uint16_t destSize)
{
	uint16_t i = 0;

	if(destSize == 0)
	{
		return;
	}

	while(i < (uint16_t)(destSize - 1) && src[i] != '\0')
	{
		dest[i] = src[i];
		i++;
	}
	dest[i] = '\0';
}

//SET指令可以设置当前的SpeedRank。
//DT:方向;SR,目标速度等级,RT:角度.
const char * COMMANDS[] = {
			"SR_ACC",
			"SR_DEC",
			"SR_SET",
			"SR_PAU",
			"RT_TO",
			"DT_1",
			"DT_0",
			"RST",
			"DT_STA",
			"DT_TUR",
			"ST_PK",
			"ST_ER",
			"ST_SB",
			"ST_KP",
			"ST_KI",
			"ST_KD"
};

void USART3_Init()
{
	//配置USART2的RX与TX
	GPIO_InitTypeDef InitGPIOB;
	
	//配置发送口
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	//USART算是外设所以用的复用推挽输出
	InitGPIOB.GPIO_Mode = GPIO_Mode_AF_PP;
	
	InitGPIOB.GPIO_Pin = 0X00000000;

	InitGPIOB.GPIO_Pin = GPIO_Pin_10;
	
	InitGPIOB.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB,&InitGPIOB);
	
	//配置接收口
	InitGPIOB.GPIO_Mode = GPIO_Mode_IPU;
	
	InitGPIOB.GPIO_Pin = 0X00000000;

	InitGPIOB.GPIO_Pin = GPIO_Pin_11;
	
	InitGPIOB.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB,&InitGPIOB);
	
	//开启UART3的内部时钟。
	//串口使用的是独立时钟因此不占用内部定时器资源。
	//配置USART3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	USART_InitTypeDef USART_InitStructure;
	
	//这个初始化函数会根据DIV与内部时钟频率自动换算对接硬件的波特率
	USART_InitStructure.USART_BaudRate = 115200;
	
	//硬件流控制要进中断,这里不用
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	
	USART_InitStructure.USART_Parity = USART_Parity_No;
	
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	
	USART_Init(USART3, &USART_InitStructure);
	
		//--------------配置NVIC,因为是集成在内部的串口读取功能因此不需要配置中断上下降沿啥的。
	
		
	//配置USART3的中断口
	//注意这里的ITConfig与NVIC需要在UART配置完之后配置。其函数内部逻辑需要使用上面对USART的配置值。
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	//配置中断优先级。
	//这个组别编号越小优先级分的越细。2组分了4个优先级。
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART3,ENABLE);
}

//-----以下为写入数据-----//

void USART3_SendByte(uint8_t Byte)
{
	uint16_t nextHead;

	while(1)
	{
		__disable_irq();
		nextHead = USART3_NextTxIndex(TxHead);
		if(nextHead != TxTail)
		{
			TxBuffer[TxHead] = Byte;
			TxHead = nextHead;
			USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
			__enable_irq();
			return;
		}
		__enable_irq();
	}
	//检测输出缓冲寄存器中的值是否转移到了移位寄存器中,转移完成才可以认为其可进行下一次读取。
	
	//TXE意思是可以TDR空了直接可以再传动东西进去,但TC是发送完了再传值,没这个快(TC要等一个字节发送周期结束才可以继续发送,属于把缓冲区当摆设)
//	if(SET == USART_GetFlagStatus(USART3,USART_FLAG_TXE))
//	{
//		USART_SendData(USART3,Byte);
//	}
		//必须阻塞式检测吗。。虽然一般不会卡主进程就是但要是中断进的频繁。。
	
}

void USART3_SendArray(int8_t Array[],uint32_t LEN)
{
	for(uint32_t i = 0;i<LEN;i++)
	{
		USART3_SendByte(Array[i]);
	}
}

void USART3_SendString(const char Array[])
{
	for(uint32_t i = 0;Array[i]!='\0';i++)
	{
		USART3_SendByte(Array[i]);
	}
}

//这种是比较浪费内存和时间的写法。
void USART3_SendNum(int32_t num)
{
	uint8_t Nums[10];
	uint8_t i = 0;
	do
	{
		Nums[i] = num%10;
		num/=10;
		i++;
	}
	while(num);
	
	while(i--)
	{
		USART3_SendByte(Nums[i]+'0');
	}
}

void USART3_SendSingedNum(int32_t num)
{
	if(num<0)
	{
		num = -num;
		USART3_SendByte('-');
	}
	USART3_SendNum(num);
}

//函数重写(复写原始stdio库)
int fputc(int ch, FILE * serial) 
{
	//这个是printf底层,它是输出多个字符的元操作,原来是输出到stdin上的,现在输出到串口上了。
	USART3_SendByte(ch);
	return ch;
}

//这个封装的方案不改printf底层但要开缓冲区
void USART3_printf(const char * format,...)
{
	char str[100];
	
	va_list ap;
	
	va_start(ap, format);
	
	vsnprintf((char *)str,100,format,ap);
	
	USART3_SendString(str);
	
	va_end(ap);
}

//-----下面是读入数据-----//

uint8_t USART_ReadByte()
{
	//或者读取采用轮询的方案
	//先检测是否读取完成(接收缓冲区已经接收完毕)
	if(USART_GetFlagStatus(USART3,USART_FLAG_RXNE) ==SET)
	{
		//对缓冲区读取数据并将RXNE位置0。
		return USART_ReceiveData(USART3);		
	}
	
	return 0;
	//等待报出数据已收到(SET),终止函数等待下一次接收。
//	while(USART_GetFlagStatus(USART3,USART_FLAG_RXNE) ==RESET);
//	
//	return USART_ReceiveData(USART3);	
}

void SendArrayPackage(int8_t Array[],uint32_t LEN)
{
	USART3_SendByte(DPACKAGE_HEAD);
	USART3_SendArray( Array,LEN);
	USART3_SendByte(DPACKAGE_TAIL);
}

void SendStringPackage(const char Array[])
{
	USART3_SendByte(TPACKAGE_HEAD);
	USART3_SendString(Array);
	USART3_SendByte(TPACKAGE_TAIL0);
	USART3_SendByte(TPACKAGE_TAIL1);
}

int8_t * GetUSART3DataBuffer()
{
	return DataBuffer;
}

char * GetUSART3TextBuffer()
{
	return LastTextBuffer;
}

uint8_t USART3_ReadText(char *buffer, uint16_t bufferSize)
{
	uint16_t i = 0;
	uint8_t ready = 0;

	if(buffer == NULL || bufferSize == 0)
	{
		return 0;
	}

	__disable_irq();
	if(ReadyTextValid)
	{
		while(i < (uint16_t)(bufferSize - 1) && ReadyTextBuffer[i] != '\0')
		{
			buffer[i] = ReadyTextBuffer[i];
			i++;
		}
		buffer[i] = '\0';
		CopyText(LastTextBuffer, buffer, USART3_BUFFER_SIZE);
		ReadyTextValid = 0;
		ready = 1;
	}
	__enable_irq();

	return ready;
}

uint16_t GetUSART3BufferCnt()
{
	uint16_t t;

	__disable_irq();
	t = BufferCnt;
	BufferCnt = 0;
	__enable_irq();

	return t;
}

//uint8_t GetUSART3RXFlag()
//{
//	if(USART3Flag_RX)
//	{
//		//标志位清理防止后续反复进入读取状态
//		USART3Flag_RX = 0;
//		return 1;
//	}
//	return 0;
//}

uint16_t GetUSART3Data()
{
	return U3Data;
}

//经典一次性标志位接口。目的是检查接口读到数据之后,对数据进行处理
//后面读数据包有大用。
//若数据已经读取完毕,那么这里会获得有效标识,主循环里可以开始读取。
uint8_t GetUSART3RXDState()
{
	uint8_t ready = 0;

	__disable_irq();
	if(isUSART3_DState == 2)
	{
		//标志位清理防止后续反复进入读取状态
		isUSART3_DState = 0;
		ready = 1;
	}
	__enable_irq();

	return ready;
}

uint8_t GetUSART3RXTState()
{
	uint8_t ready = 0;

	__disable_irq();
	if(ReadyTextValid)
	{
		//标志位清理防止后续反复进入读取状态
		CopyText(LastTextBuffer, ReadyTextBuffer, USART3_BUFFER_SIZE);
		ReadyTextValid = 0;
		ready = 1;
	}
	__enable_irq();

	return ready;
}

//还有一种Read方法用的是中断触发。
void USART3_IRQHandler()
{
	if (USART_GetITStatus(USART3, USART_IT_TXE) == SET)
	{
		if(TxTail != TxHead)
		{
			USART_SendData(USART3, TxBuffer[TxTail]);
			TxTail = USART3_NextTxIndex(TxTail);
		}
		else
		{
			USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
		}
	}

		//检查中断标志位是否为SET,即缓冲区接收完毕
	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{		
		//自定义标志位可以保存接收状态,这个在后续的数值处理中有用
		USART3Flag_RX = 1;
		
		U3Data = USART_ReadByte();

		//状态机跳跃:初始,读取尾就绪,缓冲区数据没被取走
//		if(U3Data == DPACKAGE_HEAD && isUSART3_DState == 0 && BufferCnt == 0)
//		{
//			isUSART3_DState = 1;
//		}			
//		//中途,正常读取数据
//		else if(isUSART3_DState == 1)
//		{
//			//为尾部,终止读取。
//			if(U3Data == DPACKAGE_TAIL)
//			{
//				isUSART3_DState = 2;
//			}
//			//否则读取数据(及时是和头部一样的数据也没关系。)
//			else
//			{
//				DataBuffer[BufferCnt++] = U3Data;
//			}
//		}
		
		//文本交互模式。使用指令集定义执行。
		if(U3Data == TPACKAGE_HEAD)
		{
			isUSART3_TState = 1;
			BufferCnt = 0;
			TextBuffer[0] = '\0';
		}			
		//中途,正常读取数据
		else if(isUSART3_TState == 1)
		{
			//为尾部,终止读取。
			if(U3Data == TPACKAGE_TAIL0)
			{
				TextBuffer[BufferCnt] = '\0';
				if(!ReadyTextValid)
				{
					CopyText(ReadyTextBuffer, TextBuffer, USART3_BUFFER_SIZE);
					ReadyTextValid = 1;
				}
				BufferCnt = 0;
				isUSART3_TState = 0;
				TextBuffer[0] = '\0';
			}
			//否则读取数据(及时是和头部一样的数据也没关系。)
			else if(U3Data != TPACKAGE_TAIL1)
			{
				if(BufferCnt < (USART3_BUFFER_SIZE - 1))
				{
					TextBuffer[BufferCnt++] = (char)U3Data;
				}
				else
				{
					BufferCnt = 0;
					isUSART3_TState = 0;
					TextBuffer[0] = '\0';
				}
			}
		}
		
		//是的话清除标志位否则无限跳入中断造成卡死。
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);		
	}
}

