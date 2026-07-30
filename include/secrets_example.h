// Copie este ficheiro para "secrets.h" (mesma pasta) e preencha os valores.
// secrets.h fica de fora do git (ver .gitignore) porque tem passwords.

#pragma once

#define WIFI_SSID "NOME_DA_TUA_REDE"
#define WIFI_PASSWORD "PASSWORD_DA_REDE"

// Endereço do broker MQTT. Se usares o add-on "Mosquitto broker" dentro do
// Home Assistant, é normalmente o IP do próprio HA, porta 1883.
#define MQTT_HOST "192.168.1.XXX"
#define MQTT_PORT 1883
#define MQTT_USER "cascata"
#define MQTT_PASSWORD "PASSWORD_DO_MQTT"

// Identificador único deste dispositivo (usado nos tópicos MQTT e no
// Home Assistant). Muda se tiveres mais de uma cascata na mesma rede.
#define DEVICE_ID "cascata_uc01786"
