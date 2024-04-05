#include <ESPping.h>
#include <pt.h>

const char* domain = "google.com";



bool check_conn() {
      bool ret = Ping.ping(domain, 3);
      if (ret == true) {
        Serial.print("Ping successful");
        conn_state = CONNECTED;
        return true;
      } else {
        Serial.println("Ping failed");
        blinkLED();
        conn_state = NO_NET;
        return false;
      }
}