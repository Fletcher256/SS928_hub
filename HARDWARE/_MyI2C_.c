#include "_MyI2C_.h"

static i2cbus_struct *P_this; // 全局指针

/////////////////////////以下是为方便移植///////////////////////////////
// 注意如果要
#define I2C_SCL_WRITE(x) GPIO_WriteBit(P_this->scl_gpio, P_this->scl_pin, (BitAction)x)
#define I2C_SDA_WRITE(x) GPIO_WriteBit(P_this->sda_gpio, P_this->sda_pin, (BitAction)x)
#define I2C_SDA_READ()   GPIO_ReadInputDataBit(P_this->sda_gpio, P_this->sda_pin)
/////////////////////////以上是为方便移植///////////////////////////////

// #define _I2CDelay(x) HAL_Delay(x)
#define _I2CDelay(time) for (int i = 0; i < time; i++)
inline static void i2c_scl_write(uint8_t x)
{ // 提供scl快速写函数，可以在里面进行修改以提高性能
    // if (x)
    //     GPIOA->BSRR = GPIO_Pin_2;
    // else
    //     GPIOA->BRR = GPIO_Pin_2;

    // 以上为寄存器操作，以下为库函数操作

    I2C_SCL_WRITE(x);
    _I2CDelay(P_this->delay_time); // 延时
}

inline static void i2c_sda_write(uint8_t x)
{ // 提供sda快速写函数，可以在里面进行修改以提高性能
    // if (x)
    //     GPIOA->BSRR = GPIO_Pin_1;
    // else
    //     GPIOA->BRR = GPIO_Pin_1;

    // 以上为寄存器操作，以下为库函数操作

    I2C_SDA_WRITE(x);
    _I2CDelay(P_this->delay_time); // 延时
}

inline static uint8_t i2c_sda_read()
{ // 提供sda快速读函数，可以在里面进行修改以提高性能
    uint8_t temp = I2C_SDA_READ();
    _I2CDelay(P_this->delay_time); // 延时
    return temp;
}

static void _SI2C_Start()
{
    i2c_sda_write(1);
    i2c_scl_write(1);
    i2c_sda_write(0);
    i2c_scl_write(0);
}

static void _SI2C_Stop()
{
    i2c_sda_write(0);
    i2c_scl_write(1);
    i2c_sda_write(1);
}

static void _SI2C_WriteByte(uint8_t Byte)
{
    for (uint8_t i = 0; i < 8; i++) {
        i2c_sda_write(Byte & (0x80 >> i)); // for example 1010 1100  -> 1010 1100 -> 1010 1100
                                           //             |              |      		|
        i2c_scl_write(1);
        i2c_scl_write(0);
    }
}

static uint8_t _SI2C_ReceiveByte()
{
    uint8_t i, Byte = 0x00;
    i2c_sda_write(1);
    for (i = 0; i < 8; i++) {
        i2c_scl_write(1);
        if (i2c_sda_read() == 1) { Byte |= (0x80 >> i); }
        i2c_scl_write(0);
    }
    return Byte;
}
// 0应答ACK , 1 is NACK
static void _SI2C_WriteAck(uint8_t AckBit)
{
    i2c_sda_write(AckBit);
    i2c_scl_write(1);
    i2c_scl_write(0);
}

// 0成功   1失败(failed)
static uint8_t _SI2C_ReceiveAck()
{ // receive ask
    i2c_sda_write(1);
    i2c_scl_write(1);
    uint8_t AckBit = i2c_sda_read();
    i2c_scl_write(0);
    return AckBit;
}

/////////////////////////////////你应该看以下的函数，上面是iic底层通信协议///////////////////////////////////////
/*i2c_bus i2cbus_struct对象
 *scl_gpio scl的IO口
 *scl_pin scl引脚号
 *sda_gpio sda的IO口
 *sda_pin sda引脚号
 *Address 设备地址
 *delay_time 延时时间
 */
void MyI2C_Init(i2cbus_struct *i2c_bus,
                GPIO_TypeDef *scl_gpio, uint16_t scl_pin,
                GPIO_TypeDef *sda_gpio, uint16_t sda_pin,
                uint8_t Address, uint16_t delay_time)
{
    P_this              = i2c_bus; // 获取指针对象
    i2c_bus->scl_gpio   = scl_gpio;
    i2c_bus->scl_pin    = scl_pin;
    i2c_bus->sda_gpio   = sda_gpio;
    i2c_bus->sda_pin    = sda_pin;
    i2c_bus->mode_16bit = 0;            /// 默认使用8位操作模式
    i2c_bus->address    = Address << 1; // 获取7位设备地址
    i2c_bus->delay_time = delay_time;   // 延时时间

    if (scl_gpio == GPIOA || sda_gpio == GPIOA)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if (scl_gpio == GPIOB || sda_gpio == GPIOB)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if (scl_gpio == GPIOC || sda_gpio == GPIOC)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    else if (scl_gpio == GPIOD || sda_gpio == GPIOD)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    else if (scl_gpio == GPIOE || sda_gpio == GPIOE)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    else if (scl_gpio == GPIOF || sda_gpio == GPIOF)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
    else if (scl_gpio == GPIOG || sda_gpio == GPIOG)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);

    GPIO_InitTypeDef GPIO_Init_Struct;
    GPIO_Init_Struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init_Struct.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_Init_Struct.GPIO_Pin   = scl_pin;
    GPIO_Init(scl_gpio, &GPIO_Init_Struct);
    GPIO_Init_Struct.GPIO_Pin = sda_pin;
    GPIO_Init(sda_gpio, &GPIO_Init_Struct);

    GPIO_WriteBit(scl_gpio, scl_pin, (BitAction)1);
    GPIO_WriteBit(sda_gpio, sda_pin, (BitAction)1);
}

//(0:8bit模式  1:16bit模式)
void MYI2C_Set_Mode(i2cbus_struct *i2c_bus, uint8_t status)
{
    P_this              = i2c_bus;
    i2c_bus->mode_16bit = status;
}

void MYI2C_Write_Reg(i2cbus_struct *i2c_bus, uint8_t RegAddress, uint16_t Data)
{
    P_this = i2c_bus;
    _SI2C_Start();
    _SI2C_WriteByte(i2c_bus->address);
    _SI2C_ReceiveAck();
    _SI2C_WriteByte(RegAddress);
    _SI2C_ReceiveAck();

    if (i2c_bus->mode_16bit == 1) {            // 如果是16位操作模式
        _SI2C_WriteByte((uint8_t)(Data >> 8)); // 发送高位
        _SI2C_ReceiveAck();
        _SI2C_WriteByte((uint8_t)Data); // 发送低位
    } else {
        _SI2C_WriteByte((uint8_t)Data);
    }
    _SI2C_ReceiveAck();
    _SI2C_Stop();
}
uint16_t MYI2C_Read_Reg(i2cbus_struct *i2c_bus, uint8_t RegAddress)
{
    P_this        = i2c_bus;
    uint16_t Data = 0;
    _SI2C_Start();
    _SI2C_WriteByte(i2c_bus->address);
    if (_SI2C_ReceiveAck() == 1) return 0xFFFF;
    _SI2C_WriteByte(RegAddress);
    if (_SI2C_ReceiveAck() == 1) return 0xFFFF;

    _SI2C_Start();
    _SI2C_WriteByte(i2c_bus->address | 0x01); //|0x01读命令
    if (_SI2C_ReceiveAck() == 1) return 0xFFFF;
    if (i2c_bus->mode_16bit == 1) { // 如果是16位操作模式
        Data = (uint16_t)_SI2C_ReceiveByte() << 8;
        _SI2C_WriteAck(0); // 接收应答
        Data |= _SI2C_ReceiveByte();
    } else {
        Data = _SI2C_ReceiveByte();
    }
    _SI2C_WriteAck(1); // 直接写1结束这次通信
    _SI2C_Stop();
    return Data;
}

void MYI2C_Write_Reg_Continue(i2cbus_struct *i2c_bus, uint8_t RegAddress, uint8_t *data_array, uint16_t array_size)
{    
    P_this = i2c_bus;
    _SI2C_Start();
    
    // 发送设备地址（写模式）
    _SI2C_WriteByte(i2c_bus->address);
    if (_SI2C_ReceiveAck() != 0) {
        _SI2C_Stop();
        return; // 未收到应答，退出
    }
    
    // 发送寄存器地址
    _SI2C_WriteByte(RegAddress);
    if (_SI2C_ReceiveAck() != 0) {
        _SI2C_Stop();
        return; // 未收到应答，退出
    }
    
    // 连续写入数据
    for (uint16_t i = 0; i < array_size;) {
        if (i2c_bus->mode_16bit == 1 && i + 1 < array_size) {
            // 16位模式发送高字节
            _SI2C_WriteByte(data_array[i]);
            if (_SI2C_ReceiveAck() != 0) {
                _SI2C_Stop();
                return;
            }
            
            // 16位模式发送低字节
            _SI2C_WriteByte(data_array[i + 1]);
            if (_SI2C_ReceiveAck() != 0) {
                _SI2C_Stop();
                return;
            }
            
            i += 2;
        } else {    
            // 8位模式发送
            _SI2C_WriteByte(data_array[i]);
            if (_SI2C_ReceiveAck() != 0) {
                _SI2C_Stop();
                return;
            }
            i++;
        }
    }
    
    _SI2C_Stop();
}

void MYI2C_Read_Reg_Continue(i2cbus_struct *i2c_bus, uint8_t RegAddress, uint16_t read_len, uint8_t *data_buf) // IIC 读起始地址Address里的Data[]
{
    P_this = i2c_bus; // 全局指针赋值（与原函数完全一致）
    _SI2C_Start();
    _SI2C_WriteByte(i2c_bus->address);
    if (_SI2C_ReceiveAck() == 1){
        _SI2C_Stop();
        return;
    }
    _SI2C_WriteByte(RegAddress);
    if (_SI2C_ReceiveAck() == 1){
        _SI2C_Stop();
        return;
    }

    _SI2C_Start();
    _SI2C_WriteByte(i2c_bus->address | 0x01); //|0x01读命令
    if (_SI2C_ReceiveAck() == 1){
        _SI2C_Stop();
        return;
    }

    for (uint16_t i = 0; i < read_len; i++) {
        data_buf[i] = _SI2C_ReceiveByte();
        if (i < read_len - 1)
            _SI2C_WriteAck(0); // ACK
        else
            _SI2C_WriteAck(1); // NACK
    }
    _SI2C_Stop();
}

uint8_t MYI2C_Add_Scan(i2cbus_struct *i2c_bus)
{
    P_this = i2c_bus;
    // 遍历7位I2C地址范围 (0x08-0x77)
    for (uint8_t address = 0x08; address <= 0x7F; address++) {

        _SI2C_Start();                    // 起始信号
        _SI2C_WriteByte(address << 1);    // 地址
        uint8_t ack = _SI2C_ReceiveAck(); // 读取ACK信号
        _SI2C_Stop();                     // 停止

        if (ack == 0) // 设备存在返回地址
            return address;
    }
    // 未找到设备
    return 0xFF;
}

