/*
The sketch is an empty template for Bluetooth Low Energy 4.
Simply remove what you dont need, and fill in the rest.
*/

/*
 Copyright (c) 2014 OpenSourceRF.com.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <RFduinoBLE.h>

#if 0
# define DEBUG(x) Serial.print(x)
# define DEBUGLN(x) Serial.println(x)
# define DEBUG_START() Serial.begin(9600)
# define DEBUG_END() Serial.end()
#else
# define DEBUG(x) 
# define DEBUGLN(x) 
# define DEBUG_START()
# define DEBUG_END()
#endif

//#define DEBUG(x) Serial.print(x)
//#define DEBUGLN(x) Serial.println(x)

struct { // this is what is send out
  uint32_t current_time_ms;
  uint32_t event_time_ms;
} __attribute__((packed)) event;

#define DEBOUNCE_MS           500
#define ADVERTISMENT_INTERVAL MILLISECONDS(500)
#define ADVERTISMENT_RETRIES  120

#define BUTTON_PIN 6
#define LED_PIN    1

/* gearing the power consumption with those:
**  one advertisment costs       5.0mA for 2.5ms ADVERTISMENTS times every 500ms
**  sending data                 5.0mA for   1ms per data packet
**  kepping ble and cpu active   0.8mA
*
* batteries in the dresden case default to three lr626 (1.55v) batteries in
* series. Capacity for LR626 (Alkaline) is 12-18mA, for SR262 (Silver-oxide) is
* 26mA. At .8mA for active operation at 16mAh capacity the battery should last
* for 20hours of continous operation.
*/

#define EVQ_SIZE 512
volatile struct { // this is our event queue
  volatile int32_t head, tail;
  uint32_t timestamp[EVQ_SIZE+1];
} evq = {0,0};

/* figure out when the chip got reset because of a powerloss */
static uint32_t events_since_powerup = 0;

/* store the startup value of the ADC register, to make sure it is turned
** off during low-power mode. */
static uint32_t adc_default_settings;

uint32_t evq_len()
{
  int32_t n = (evq.tail-evq.head) % (EVQ_SIZE+1);
  return n < 0 ? n + (EVQ_SIZE+1) : n;
}

void setup()
{
  // setup pin monitoring, careful: pin 0/1 are UART pins!
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // setup LED pin
  pinMode(LED_PIN, OUTPUT);

  // for battery level measurement
  analogReference(VBG); // bandgap 1.2v selection
  analogSelection(VDD_1_3_PS); // 1/3 prescaling
  adc_default_settings = NRF_ADC->CONFIG;
}

char* power_status()
{
  NRF_ADC->TASKS_STOP =  0;
  static char str[32] = {0};
  uint32_t voltage;

  /* first call to analogRead starts the ADC, discard the first read, wait for
  ** ADC to be charged and read again, then make sure it is properly turned
  ** off and store the result in a string */
  voltage = (uint32_t) (analogRead(2) * (3.6/1023.0) * 1000);
  RFduino_ULPDelay(100);
  voltage = (uint32_t) (analogRead(2) * (3.6/1023.0) * 1000);
  snprintf(str, sizeof(str), "%d %s", voltage,
                events_since_powerup < 2 ? "empty" : "");

  NRF_ADC->TASKS_START =  0; // make sure ADC is stopped
  NRF_ADC->TASKS_STOP =  1; // make sure ADC is stopped
  NRF_ADC->CONFIG = adc_default_settings;
  return str;
}

volatile bool connected = false;

#define MyDelay(x) do {RFduino_ULPDelay(x); RFduino_resetPinWake(BUTTON_PIN);} while(0);

int wakeup(long unsigned int source)
{
  return 1;
}

void loop()
{
  // only wakeup when button was triggered!
  RFduino_pinWakeCallback(BUTTON_PIN, LOW, wakeup);
  MyDelay(INFINITE);

  DEBUG_START();
  DEBUGLN("woke up");

  // ignore any further pin changes and check if pin is low after
  // DEBOUNCE Timeout
  RFduino_pinWakeCallback(BUTTON_PIN, DISABLE, NULL);
  MyDelay(DEBOUNCE_MS);
  if (digitalRead(BUTTON_PIN)) {
    DEBUGLN("sleeping...");
    DEBUG_END();
    return;
  }

  DEBUGLN("and going for it");

  evq.timestamp[evq.tail] = millis();
  evq.tail = (evq.tail+1)%(EVQ_SIZE+1);
  events_since_powerup += 1;

  // setup BLE device
  RFduinoBLE.deviceName = "iLitIt 1.3";
  RFduinoBLE.advertisementData = power_status();
  DEBUGLN(RFduinoBLE.advertisementData);
  RFduinoBLE.advertisementInterval = ADVERTISMENT_INTERVAL;
  RFduinoBLE.txPowerLevel = -20;  // (-20dbM to +4 dBm)
  RFduinoBLE.customUUID = "595403fb-f50e-4902-a99d-b39ffa4bb134";
  RFduinoBLE.begin();
  DEBUG("advertising");

  // 2min connection timeout
  for (uint32_t i=0; !connected && i < ADVERTISMENT_RETRIES; i++) {
    DEBUG(".");
    digitalWrite(LED_PIN, HIGH);
    MyDelay(1);
    digitalWrite(LED_PIN, LOW);
    MyDelay(ADVERTISMENT_INTERVAL);
  }

  if(!connected) {
    DEBUGLN("timed out");
    goto end;
  }

  DEBUGLN();
  DEBUG(evq_len());
  DEBUGLN(" packets to send");

  for(uint16_t timeout = 0; connected && evq_len() > 0 && timeout < 10;)
  {
    // dequeue and send events
    event.current_time_ms = millis();
    event.event_time_ms   = evq.timestamp[evq.head];

    DEBUGLN(event.event_time_ms);

    // send all events
    if (RFduinoBLE.send( (char*) &event, sizeof(event) ))
      evq.head = (evq.head+1)%(EVQ_SIZE+1);
    else
    {
      MyDelay(ADVERTISMENT_INTERVAL);
      timeout++;
    }
  }

end:
  digitalWrite(LED_PIN, LOW);
  DEBUG("shutting down");
  connected = false;
  MyDelay(500);

  // wait at least five minutes for the next cigarette
  MyDelay(5 * 60 * 1000);
  RFduinoBLE.end();
  DEBUG("done");
  DEBUG_END();
}

void RFduinoBLE_onConnect()    { connected=true;  DEBUGLN("connected"); }
void RFduinoBLE_onDisconnect() { connected=false; DEBUGLN("disconnected"); }
