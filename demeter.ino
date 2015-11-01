#include <Wire.h>
#include <Time.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

// Parameters you'll want to configure
const int tz_adjust = -8; // Timezone adjustment from UTC in hours (e.g., PDT is UTC-8h)
const char* wifi_ssid = "wifissid"; // Wifi SSID
const char* wifi_pass = "wifipass"; // Wifi passphrase
const char* ntp_hostname = "time.nist.gov"; // NTP server hostname

// Define electronic component locations
const int pin_sda = 12;
const int pin_scl = 13;
const int lcd_addr = 0x71;

// Define networking and time constants
const int delay_ntp_sync = 5 * 60; // 5 minutes of seconds
const int timeout_ntp = 10 * 1000000; // 10 seconds of microseconds
const int delay_wifi_status = 50; // 50 milliseconds
const int delay_udp_parse = 10; // 10 milliseconds
const unsigned int port_listen = 2390;
const unsigned int ntp_packet_size = 48;
const unsigned long seventy_years = 2208988800UL; // 70 years of seconds
const int secs_per_hr = 60 * 60; // seconds in an hour

// Initialize packet buffer and time variables
byte packet_buffer[ntp_packet_size];
time_t time_displayed;

// Make output pretty with these variables
const int col_max = 32; // max columns
int col_loc = 0;

Adafruit_7segment matrix = Adafruit_7segment();
WiFiUDP udp;
IPAddress ntp_address;

void setup() {

  Wire.begin(pin_sda, pin_scl);
  matrix.begin(lcd_addr);
  matrix.writeDisplay();
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n\nWelcome to __demeter__!");

  WiFi.begin(wifi_ssid, wifi_pass);
  Serial.print("Connecting to SSID ");
  Serial.print(wifi_ssid);
  Serial.println(".");
  
  col_loc = 0; // Reset our column counter
  while (WiFi.status() != WL_CONNECTED) {
    print_serial_dot_loop(); // Print some dots to serial to show that we're waiting
    delay(delay_wifi_status); // Delay before checking Wifi status again
  }
  Serial.println();
  Serial.print("Wifi connected with IP address ");
  Serial.print(WiFi.localIP());
  Serial.println(".");

  udp.begin(port_listen);
  Serial.print("UDP listening on port ");
  Serial.print(udp.localPort());
  Serial.println(".");

  // Draw a colon on the display to show that Wifi is connected and UDP is listening
  matrix.drawColon(1);
  matrix.writeDisplay();
  Serial.println("Initialization complete. Colon drawn on 7-segment display.");
  // Initialize our currently-displayed time as being 0.
  time_displayed = 0;

  // time.h timekeeping library; tell it that get_ntp_time() is the provider. 
  setSyncProvider(get_ntp_time);
  setSyncInterval(delay_ntp_sync);
  
}

void loop() {
  if (timeStatus() != timeNotSet) {
    if (now() != time_displayed) {
      print_time_7seg();
      time_displayed = now();
    }
  }
}

int print_serial_dot_loop() {
    if (col_loc < col_max) {
      Serial.print(".");
      col_loc++;
    } else {
      Serial.println(".");
      col_loc = 0;
    }
};

int print_time_7seg() {
  int HH = hour();
  int mm = minute();
  matrix.writeDigitNum(0, int(HH/10));
  matrix.writeDigitNum(1, HH % 10);
  matrix.drawColon(1);
  matrix.writeDigitNum(3, int(mm/10));
  matrix.writeDigitNum(4, mm % 10);
  matrix.writeDisplay();
}

time_t get_ntp_time() {
  // Resolve NTP hostname to IP address
  WiFi.hostByName(ntp_hostname, ntp_address);
  Serial.print("Resolved ");
  Serial.print(ntp_hostname);
  Serial.print(" to IP ");
  Serial.print(ntp_address);
  Serial.println(".");
  // Send NTP request to IP address
  send_ntp_packet(ntp_address);
  // Record the time at which the request was sent
  int time_pkt_sent = micros();
  int time_elapsed = 0;
  int pkt_length = 0;
  Serial.print("Sent request; waiting for NTP response from ");
  Serial.print(ntp_address);
  Serial.println(".");
  col_loc = 0; // Reset our column counter
  while (pkt_length == 0 & time_elapsed <= timeout_ntp) {
    delay(delay_udp_parse); // Wait to check for a UDP response
    pkt_length = udp.parsePacket(); // Try to parse the UPD response (if any)
    print_serial_dot_loop(); // Print some dots to serial to show that we're waiting
    time_elapsed = micros() - time_pkt_sent; // Update the amount of time that has passed
  }
  Serial.println();
  if (pkt_length < ntp_packet_size & time_elapsed <= timeout_ntp) {
    Serial.println("Malformed NTP response received.");
    return 0;
  } else if (pkt_length < ntp_packet_size) {
    Serial.println("No NTP response before timeout.");
    return 0;
  } else {
    Serial.print("Received packet of length ");
    Serial.print(pkt_length);
    Serial.print(" after ");
    // convert microseconds to milliseconds
    Serial.print(time_elapsed/1000);
    Serial.print(" ms (resolution ");
    Serial.print(delay_udp_parse);
    Serial.println(" ms).");
    // read the packet into the buffer
    udp.read(packet_buffer, ntp_packet_size);
    // The NTP transmit timestamp is 64 bits (8 bytes or 4 words) long.
    // Transmit timestamp is located at bytes 40 - 47.
    // The first 4 bytes (2 words) are the integer part.
    // Ref: https://tools.ietf.org/html/rfc958#appendix-B
    // Extract these 4 bytes, cast into two words.
    unsigned long word_high = word(packet_buffer[40], packet_buffer[41]);
    unsigned long word_low = word(packet_buffer[42], packet_buffer[43]);
    // Combine the 2 words into a long int (seconds since 1900-01-01)
    unsigned long time_ntp = word_high << 16 | word_low;
    return time_t (time_ntp - seventy_years + tz_adjust * secs_per_hr);
  }
}

int send_ntp_packet(IPAddress &ntp_address) {
  // set all bytes in the buffer to 0
  memset(packet_buffer, 0, ntp_packet_size);
  // Initialize values needed to form NTP request
  packet_buffer[0]  = 0b11100011; // LI, Version, Mode
  packet_buffer[1]  = 0;          // Stratum, or type of clock
  packet_buffer[2]  = 6;          // Polling Interval
  packet_buffer[3]  = 0xEC;       // Peer Clock Precision
  // 8 bytes of zero for Root Delay and Root Dispersion
  packet_buffer[12] = 49;
  packet_buffer[13] = 0x4E;
  packet_buffer[14] = 49;
  packet_buffer[15] = 52;
  // All NTP fields have values assigned, now send the NTP request to port 123
  udp.beginPacket(ntp_address, 123);
  udp.write(packet_buffer, ntp_packet_size);
  udp.endPacket();
}
