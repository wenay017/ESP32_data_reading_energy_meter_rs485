
#define MODBUS_RX_PIN       13
#define MODBUS_TX_PIN       14
#define MODBUS_SERIAL_BAUD  9600         //or 9600

#define BUF_SIZE            (500)
#define PACKET_READ_TICS    (500 / portTICK_RATE_MS) // Update to 500 milliseconds

#define ECHO_TASK_STACK_SIZE (2048)
#define ECHO_TASK_PRIO       (10)
#define BUF_SIZE1 1024
// Modbus register addresses and settings
#define MODBUS_SENSOR_ADDRESS       1
#define MODBUS_REGISTER_ADDRESS     3009

static const int uart_num = UART_NUM_1;     //uart port 1 used for gsm 
static const int tx_pin = 17;                
static const int rx_pin = 16;
static const char *TAG1 = "GSM";
#define GPIO_OUTPUT   23
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT)
int deviceid; 
uint64_t globalMacAddress; 
 uint8_t response[500]; 
uint16_t value1,value2,value3 ,value4 ,value5 ,value6,value7,value8,value9,value10;
char buffer[1024];
int len;
#define sample_read_delay  60000    
#define post_time 60000*5 
#define Restart_delay 60000*600   //10hrs
#define DEBUG 1
#define TAG "RS485_ECHO_APP"
