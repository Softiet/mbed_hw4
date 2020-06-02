// INCLDE :: GENERAL
#include "mbed.h"

// INCLUDE :: WIFI
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"

// INCLUDE :: ACCELORO
#include "fsl_port.h"
#include "fsl_gpio.h"
#define UINT14_MAX        16383
      // FXOS8700CQ I2C address
#define FXOS8700CQ_SLAVE_ADDR0 (0x1E<<1) // with pins SA0=0, SA1=0
#define FXOS8700CQ_SLAVE_ADDR1 (0x1D<<1) // with pins SA0=1, SA1=0
#define FXOS8700CQ_SLAVE_ADDR2 (0x1C<<1) // with pins SA0=0, SA1=1
#define FXOS8700CQ_SLAVE_ADDR3 (0x1F<<1) // with pins SA0=1, SA1=1
      // FXOS8700CQ internal register addresses
#define FXOS8700Q_STATUS 0x00
#define FXOS8700Q_OUT_X_MSB 0x01
#define FXOS8700Q_OUT_Y_MSB 0x03
#define FXOS8700Q_OUT_Z_MSB 0x05
#define FXOS8700Q_M_OUT_X_MSB 0x33
#define FXOS8700Q_M_OUT_Y_MSB 0x35
#define FXOS8700Q_M_OUT_Z_MSB 0x37
#define FXOS8700Q_WHOAMI 0x0D
#define FXOS8700Q_XYZ_DATA_CFG 0x0E
#define FXOS8700Q_CTRL_REG1 0x2A
#define FXOS8700Q_M_CTRL_REG1 0x5B
#define FXOS8700Q_M_CTRL_REG2 0x5C
#define FXOS8700Q_WHOAMI_VAL 0xC7

// GENERAL VARIABLES
InterruptIn btn2(SW2);
InterruptIn btn3(SW3);
Serial pc(USBTX, USBRX);

// GLOBAL VARIABLES :: WIFI
WiFiInterface *wifi;
volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool wifi_closed = false;
const char* topic = "Mbed";

Thread mqtt_thread(osPriorityHigh);
EventQueue mqtt_queue;

// GLOBAL VARIABLES :: ACCELEROMETER
I2C i2c( PTD9,PTD8);
int m_addr = FXOS8700CQ_SLAVE_ADDR1;
uint8_t who_am_i, acc_data[2], acc_res[6];
int16_t acc16;
float acc_t[3];

// GLOBAL VARIABLE :: XBEE
RawSerial xbee(D12, D11);
EventQueue xbee_queue(32 * EVENTS_EVENT_SIZE);
Thread xbee_t;

// FUNCTIONS DECLARE :: WIFI
void messageArrived(MQTT::MessageData& md);
void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client);
inline void close_mqtt() {wifi_closed = true;}

// FUNCTIONS DECLARE :: ACCELEROMETER
void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len);
void FXOS8700CQ_writeRegs(uint8_t * data, int len);
void acc_print();

// FUNCRION DECLARE :: XBEE
void xbee_rx_interrupt(void);
void xbee_rx(void);
void reply_messange(char *xbee_reply, char *messange);
void check_addr(char *xbee_reply, char *messenger);

int main() {
      // ACCELERO INITIALIZING
      pc.baud(115200);
            // Enable the FXOS8700Q
      FXOS8700CQ_readRegs( FXOS8700Q_CTRL_REG1, &acc_data[1], 1);
      acc_data[1] |= 0x01;
      acc_data[0] = FXOS8700Q_CTRL_REG1;
      FXOS8700CQ_writeRegs(acc_data, 2);
            // Get the slave address
      FXOS8700CQ_readRegs(FXOS8700Q_WHOAMI, &who_am_i, 1);
      pc.printf("Here is %x,\r\n ACC ALL SYSTEM GO\r\n", who_am_i);

      // WIFI INITIALIZING
      wifi = WiFiInterface::get_default_instance();
      if (!wifi) {
            printf("ERROR: No WiFiInterface found.\r\n");
            return -1;
      }
      printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
      int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
      if (ret != 0) {
            printf("\nConnection error: %d\r\n", ret);
            return -1;
      }
      NetworkInterface* net = wifi;
      MQTTNetwork mqttNetwork(net);
      MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);
      //TODO: revise host to your ip
      const char* host = "192.168.43.189";
      printf("Connecting to TCP network...\r\n");
      int rc = mqttNetwork.connect(host, 1883);
      if (rc != 0) {
            printf("Connection error.");
            return -1;
      }
      printf("Successfully connected!\r\n");
      MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
      data.MQTTVersion = 3;
      data.clientID.cstring = "Mbed";
      if ((rc = client.connect(data)) != 0){
            printf("Fail to connect MQTT\r\n");
      }
      if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
      }
      mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));
      
      // XBEE INITIALIZING
      char xbee_reply[4];
            // XBee setting
      xbee.baud(9600);
      xbee.printf("+++");
      xbee_reply[0] = xbee.getc();
      xbee_reply[1] = xbee.getc();
      if(xbee_reply[0] == 'O' && xbee_reply[1] == 'K'){
            pc.printf("enter AT mode.\r\n");
            xbee_reply[0] = '\0';
            xbee_reply[1] = '\0';
      }
      xbee.printf("ATMY <REMOTE_MY>\r\n");
      reply_messange(xbee_reply, "setting MY : <REMOTE_MY>");
      xbee.printf("ATDL <REMOTE_DL>\r\n");
      reply_messange(xbee_reply, "setting DL : <REMOTE_DL>");
      xbee.printf("ATID <PAN_ID>\r\n");
      reply_messange(xbee_reply, "setting PAN ID : <PAN_ID>");
      xbee.printf("ATWR\r\n");
      reply_messange(xbee_reply, "write config");
      xbee.printf("ATMY\r\n");
      check_addr(xbee_reply, "MY");
      xbee.printf("ATDL\r\n");
      check_addr(xbee_reply, "DL");
      xbee.printf("ATCN\r\n");
      reply_messange(xbee_reply, "exit AT mode");
      xbee.getc();
      // start
      pc.printf("start\r\n");
      xbee_t.start(callback(&xbee_queue, &EventQueue::dispatch_forever));
      // Setup a serial interrupt function of receiving data from xbee
      xbee.attach(xbee_rx_interrupt, Serial::RxIrq);

      // GENERAL CODE
      btn2.rise(mqtt_queue.event(&publish_message, &client));
      btn3.rise(&close_mqtt);




      return 0;

}


// FUNCTIONS :: WIFI
void messageArrived(MQTT::MessageData& md) {
      MQTT::Message &message = md.message;
      char msg[300];
      sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
      printf(msg);
      wait_ms(1000);
      char payload[300];
      sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
      printf(payload);
      ++arrivedcount;
}

void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client) {
      message_num++;
      MQTT::Message message;
      char buff[100];
      sprintf(buff, "QoS0 Hello, Python! #%d", message_num);
      message.qos = MQTT::QOS0;
      message.retained = false;
      message.dup = false;
      message.payload = (void*) buff;
      message.payloadlen = strlen(buff) + 1;
      int rc = client->publish(topic, message);
      printf("rc:  %d\r\n", rc);
      printf("Puslish message: %s\r\n", buff);
}

// FUNCTIONS :: ACCELERO
void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len) {
   char t = addr;
   i2c.write(m_addr, &t, 1, true);
   i2c.read(m_addr, (char *)data, len);
}

void FXOS8700CQ_writeRegs(uint8_t * data, int len) {
   i2c.write(m_addr, (char *)data, len);
}

void acc_print(){
      FXOS8700CQ_readRegs(FXOS8700Q_OUT_X_MSB, acc_res, 6);
      acc16 = (acc_res[0] << 6) | (acc_res[1] >> 2);
      if (acc16 > UINT14_MAX/2)
         acc16 -= UINT14_MAX;
      acc_t[0] = ((float)acc16) / 4096.0f;
      acc16 = (acc_res[2] << 6) | (acc_res[3] >> 2);
      if (acc16 > UINT14_MAX/2)
         acc16 -= UINT14_MAX;
      acc_t[1] = ((float)acc16) / 4096.0f;
      acc16 = (acc_res[4] << 6) | (acc_res[5] >> 2);
      if (acc16 > UINT14_MAX/2)
         acc16 -= UINT14_MAX;
      acc_t[2] = ((float)acc16) / 4096.0f;
      printf("FXOS8700Q ACC: X=%1.4f(%x%x) Y=%1.4f(%x%x) Z=%1.4f(%x%x)\r\n",\
            acc_t[0], acc_res[0], acc_res[1],\
            acc_t[1], acc_res[2], acc_res[3],\
            acc_t[2], acc_res[4], acc_res[5]\
      );
}

// FUNCTIONS :: XBEE
void xbee_rx_interrupt(void){
  xbee.attach(NULL, Serial::RxIrq); // detach interrupt
  queue.call(&xbee_rx);
}

void xbee_rx(void){
  char buf[100] = {0};
  char outbuf[100] = {0};
  while(xbee.readable()){
    for (int i=0; ; i++) {
      char recv = xbee.getc();
      if (recv == '\r') {
        break;
      }
      buf[i] = pc.putc(recv);
    }
    RPC::call(buf, outbuf);
    pc.printf("%s\r\n", outbuf);
    wait(0.1);
  }
  xbee.attach(xbee_rx_interrupt, Serial::RxIrq); // reattach interrupt
}

void reply_messange(char *xbee_reply, char *messange){
  xbee_reply[0] = xbee.getc();
  xbee_reply[1] = xbee.getc();
  xbee_reply[2] = xbee.getc();
  if(xbee_reply[1] == 'O' && xbee_reply[2] == 'K'){
    pc.printf("%s\r\n", messange);
    xbee_reply[0] = '\0';
    xbee_reply[1] = '\0';
    xbee_reply[2] = '\0';
  }
}

void check_addr(char *xbee_reply, char *messenger){
  xbee_reply[0] = xbee.getc();
  xbee_reply[1] = xbee.getc();
  xbee_reply[2] = xbee.getc();
  xbee_reply[3] = xbee.getc();
  pc.printf("%s = %c%c%c\r\n", messenger, xbee_reply[1], xbee_reply[2], xbee_reply[3]);
  xbee_reply[0] = '\0';
  xbee_reply[1] = '\0';
  xbee_reply[2] = '\0';
  xbee_reply[3] = '\0';
}

   







