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

#if 1
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

#define DEBOUNCE_MS           5500
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

#define EVQ_SIZE 64
volatile struct { // this is our event queue
  volatile int32_t head, tail;
  uint32_t timestamp[EVQ_SIZE+1];
} evq = {0,0};

uint32_t evq_len()
{
  int32_t n = (evq.tail-evq.head) % (EVQ_SIZE+1);
  return n < 0 ? n + (EVQ_SIZE+1) : n;
}

void setup()
{
  // setup pin monitoring, careful: pin 0/1 are UART pins!
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  RFduino_pinWakeCallback(BUTTON_PIN, LOW, storeTime);

  // setup LED pin
  pinMode(LED_PIN, OUTPUT);

}

int storeTime(uint32_t pin)
{
  static uint32_t lastcall = -DEBOUNCE_MS;
  int32_t diff = millis() - lastcall;

  if (diff < DEBOUNCE_MS) {
    //RFduino_resetPinWake(BUTTON_PIN);
    return 1; // do not wake-up
  }

  evq.timestamp[evq.tail] = lastcall = millis();
  evq.tail = (evq.tail+1)%(EVQ_SIZE+1);

  return 1; // ring, ring, wake up!
}

volatile bool connected = false;

#define MyDelay(x) do {RFduino_ULPDelay(x); RFduino_resetPinWake(BUTTON_PIN);} while(0);

void loop()
{
  // only wakeup when button was triggered!
  MyDelay(INFINITE);

  DEBUG_START();

  DEBUG("wakey eak ");
  DEBUG(evq.head);
  DEBUG(" ");
  DEBUG(evq.tail);
  DEBUG(" ");
  DEBUGLN(evq_len());

  if (evq_len() == 0)
    goto end;

  // setup BLE device
  RFduinoBLE.deviceName = "iLitIt 1.0";
  RFduinoBLE.advertisementData = "time";
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
  RFduinoBLE.end();
  DEBUG("done");
  DEBUG_END();
}

void RFduinoBLE_onConnect()    { connected=true;  DEBUGLN("connected"); }
void RFduinoBLE_onDisconnect() { connected=false; DEBUGLN("disconnected"); }
