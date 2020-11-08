#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>


char mqttServer[] = "mqtt.thingspeak.com";
int16_t mqttServerPort = 1883;

// ThingSpeak channel id and write api key
long channelId = 1234567890;
char writeAPIKey[] = "0123456789ABCDEF";

#endif // CONFIG_H