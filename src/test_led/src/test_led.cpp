#include <test_led/test_led.h>

using namespace std;
using namespace boost::asio;
//串口相关对象
boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/ttyACM1");
boost::system::error_code err;

/********************************************************
            串口发送接收相关常量、变量、共用体对象
********************************************************/
const unsigned char ender[2] = {0x0d, 0x0a};
const unsigned char header[2] = {0x55, 0xaa};


uint8_t modeSend;
uint8_t modeRecv;

/********************************************************
函数功能：串口参数初始化
入口参数：无
出口参数：
********************************************************/
void serialInit()
{
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));    
}

/********************************************************
函数功能：将对机器人的左右轮子控制速度，打包发送给下位机
入口参数：机器人线速度、角速度
出口参数：
********************************************************/

void writeSpeed(uint8_t value)
{
    unsigned char buf[1];
    buf[0] = value;

    // 串口发送一个字节
    boost::asio::write(sp, boost::asio::buffer(buf, 1));
}


/**********************************************************
函数功能：从下位机读取数据
入口参数：机器人左轮轮速、右轮轮速、角度，预留控制位
出口参数：bool
**********************************************************/
bool readSpeed(uint8_t &mode)
{
    try
    {
        boost::asio::streambuf response;
        // 读取一行，以 \r\n 结尾
        boost::asio::read_until(sp, response, "\r\n");

        std::istream is(&response);
        std::string line;
        std::getline(is, line);

        // 打印调试
        ROS_INFO("STM32 says: %s", line.c_str());

        // 解析 "receive: %d"
        int data;
        if (sscanf(line.c_str(), "receive: %d", &data) == 1)
        {
            // 这里根据实际需要赋值
            mode = data;
            return true;
        }
        else
        {
            ROS_ERROR("Parse error: %s", line.c_str());
            return false;
        }
    }
    catch(boost::system::system_error &err)
    {
        ROS_ERROR("read_until error: %s", err.what());
        return false;
    }
}

/**********************************************************
函数功能：获得8位循环冗余校验值
入口参数：数组地址、长度
出口参数：校验值
**********************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while(len--)
    {
        crc ^= *ptr++;
        for(i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc=(crc>>1)^0x8C;
            else 
                crc >>= 1;
        }
    }
    return crc;
}
