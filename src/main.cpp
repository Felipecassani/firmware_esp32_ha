/*
 * UC01786 - Automatização de uma Cascata com Aquecimento Solar
 * Firmware experimental ESP32 + Home Assistant (MQTT Discovery)
 *
 * Duas máquinas de estados independentes (cascata + circuito solar),
 * ligadas por WiFi/MQTT ao Home Assistant. Todas as entidades aparecem
 * automaticamente no HA (Configurações > Dispositivos e serviços > MQTT).
 *
 * Isto é a versão "de casa", com hardware real, separada do entregável
 * da escola (esse é 100% Arduino IDE, sem rede, em ../cascata_UC01786).
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "secrets.h"

// ===================== PINOS (ESP32 DevKit 30 pinos) =====================
// Pinos escolhidos para evitar os "strapping pins" (0, 2, 5, 12, 15) e os
// pinos ligados à flash interna (6-11). GPIO34-39 seriam ideais para
// entradas analógicas mas ficaram livres porque só sobrou 1 sensor
// analógico (corrente) - ver nota da ACS712 mais abaixo.

const int PIN_START = 32;
const int PIN_STOP = 33;
const int PIN_EMERGENCIA = 25;
const int PIN_SELETOR_AUTO = 26; // HIGH (aberto, pull-up) = automático, LOW = manual

const int PIN_TRIG_NIVEL_SUP = 27;
const int PIN_ECHO_NIVEL_SUP = 14;
const int PIN_TRIG_NIVEL_INF = 16;
const int PIN_ECHO_NIVEL_INF = 17;

const int PIN_CAUDAL = 4; // sensor de pulso YF-S201, com interrupção

const int PIN_ONEWIRE_TEMP = 13; // DS18B20 x2 no mesmo barramento (entrada + saída)
const int PIN_CORRENTE = 36;     // ACS712, alimentar o módulo a 3.3V (ver README)

const int PIN_BOMBA_IMPULSAO = 18;
const int PIN_BOMBA_RETORNO = 19;
const int PIN_BOMBA_RECIRC = 21;
const int PIN_SINAL_VERDE = 22;
const int PIN_SINAL_AMARELO = 23;
const int PIN_SINAL_VERMELHO = 5;
const int PIN_ALARME = 15;

// ===================== SENSOR DE CAUDAL =====================

volatile unsigned long contadorPulsos = 0;
void IRAM_ATTR isrCaudal() { contadorPulsos++; }

// ===================== SENSORES DE TEMPERATURA (DS18B20) =====================

OneWire oneWireBus(PIN_ONEWIRE_TEMP);
DallasTemperature sensoresTemp(&oneWireBus);
// Índices no barramento: 0 = entrada da serpentina, 1 = saída.
// Confirmar esta ordem com o guia de calibração no README (pode variar
// consoante a ordem em que o OneWire encontra os sensores).
const uint8_t IDX_TEMP_ENTRADA = 0;
const uint8_t IDX_TEMP_SAIDA = 1;

// ===================== CALIBRAÇÃO (ajustar após montagem real) =====================

// Altura útil de cada depósito, do sensor ultrassónico (montado no topo)
// até ao fundo. Usado para converter distância -> % de enchimento.
float ALTURA_TANQUE_SUP_CM = 60.0; // TODO: medir no depósito principal (500L)
float ALTURA_TANQUE_INF_CM = 40.0; // TODO: medir no reservatório inferior

const float NIVEL_MIN_SEGURO_PCT = 8.0;  // abaixo disto = falta de água
const float NIVEL_MAX_SEGURO_PCT = 95.0; // acima disto = transbordo

// Autofill do topo (replica a lógica do painel/app): quando o depósito
// superior desce abaixo disto, liga-se a bomba de retorno para o encher
// a partir do reservatório inferior; só volta a desligar com histerese.
const float AUTOFILL_LIGA_PCT = 30.0;
const float AUTOFILL_DESLIGA_PCT = 60.0;
// Protege a bomba de retorno de trabalhar a seco.
const float NIVEL_INF_MIN_PCT = 10.0;

const float LIMIAR_CAUDAL_MIN_LMIN = 1.0; // abaixo disto em FUNCIONAMENTO = falha
const float FATOR_CALIBRACAO_CAUDAL = 7.5; // YF-S201: Hz = 7.5 * L/min (calibrar)

const int LIMIAR_CORRENTE_MAX_RAW = 2800; // ADC 12-bit (0-4095) - calibrar com pinça amperimétrica
const int CORRENTE_MIN_ARRANQUE_RAW = 2100; // "normal parado" ~ Vcc/2; ajustar
const unsigned long TIMEOUT_ARRANQUE_BOMBA_MS = 4000;

const float DELTA_TEMP_LIGA_SOLAR = 5.0;   // diferencial mínimo p/ recircular (°C)
const float DELTA_TEMP_DESLIGA_SOLAR = 1.0;
const float TEMP_MAXIMA_SERPENTINA = 65.0; // corte de segurança (°C)
const unsigned long TIMEOUT_SOLAR_SEM_EFEITO_MS = 10UL * 60UL * 1000UL; // 10 min

const int LEITURAS_INVALIDAS_PARA_AVARIA_SENSOR = 10; // ciclos consecutivos

// ===================== MÁQUINAS DE ESTADOS =====================

enum EstadoCascata
{
    DESLIGADO = 0,
    VERIFICACAO = 1,
    IMPULSAO = 2,
    ENCHIMENTO = 3,
    RETORNO = 4,
    FUNCIONAMENTO = 5,
    PARAGEM = 6,
    ESVAZIAMENTO = 7,
    PARADO = 8,
    AVARIA_CASCATA = 9
};

enum EstadoSolar
{
    ESPERA = 0,
    VERIFICACAO_SOLAR = 1,
    RECIRCULACAO = 2,
    TEMPERATURA_MAXIMA = 3,
    AVARIA_SOLAR = 4
};

EstadoCascata estadoCascata = DESLIGADO;
EstadoSolar estadoSolar = ESPERA;

const char *nomeEstadoCascata(EstadoCascata e)
{
    switch (e)
    {
    case DESLIGADO: return "DESLIGADO";
    case VERIFICACAO: return "VERIFICACAO";
    case IMPULSAO: return "IMPULSAO";
    case ENCHIMENTO: return "ENCHIMENTO";
    case RETORNO: return "RETORNO";
    case FUNCIONAMENTO: return "FUNCIONAMENTO";
    case PARAGEM: return "PARAGEM";
    case ESVAZIAMENTO: return "ESVAZIAMENTO";
    case PARADO: return "PARADO";
    case AVARIA_CASCATA: return "AVARIA";
    }
    return "?";
}

const char *nomeEstadoSolar(EstadoSolar e)
{
    switch (e)
    {
    case ESPERA: return "ESPERA";
    case VERIFICACAO_SOLAR: return "VERIFICACAO";
    case RECIRCULACAO: return "RECIRCULACAO";
    case TEMPERATURA_MAXIMA: return "TEMPERATURA_MAXIMA";
    case AVARIA_SOLAR: return "AVARIA";
    }
    return "?";
}

// ===================== VARIÁVEIS DE SENSORES =====================

float nivelSupPct = 0, nivelInfPct = 0; // % de enchimento (100 = cheio)
float tempEntrada = NAN, tempSaida = NAN, diferencialSolar = 0;
int corrente = 0; // leitura analógica bruta (0-4095)
float caudalLmin = 0;
bool falhaDetetada = false;
String descricaoFalha = "";

bool modoAutomatico = true;

// Contadores para deteção de sensor avariado / bomba bloqueada
int leiturasInvalidasNivelSup = 0, leiturasInvalidasNivelInf = 0;
int leiturasInvalidasTemp = 0, leiturasInvalidasCorrente = 0;
unsigned long tempoLigadaImpulsao = 0; // millis() de quando a bomba ligou (0 = desligada)
unsigned long tempoInicioRecirculacao = 0;
float tempSaidaNoInicioRecirc = 0;

// Comandos manuais (só têm efeito com o seletor em modo Manual)
bool cmdBombaImpulsaoManual = false;
bool cmdBombaRetornoManual = false;
bool cmdBombaRecircManual = false;

// Temporização não-bloqueante
unsigned long tempoAnteriorCiclo = 0;
const unsigned long INTERVALO_LEITURA = 200; // ms
unsigned long tempoAnteriorPublish = 0;
const unsigned long INTERVALO_PUBLISH = 2000; // ms

// ===================== WIFI / MQTT =====================

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

String topicStatus, topicState, topicCmdPrefix;
unsigned long tentativaWifiAnterior = 0, tentativaMqttAnterior = 0;
bool discoveryPublicado = false;

// ===================== PROTÓTIPOS =====================

float lerDistanciaCm(int pinTrig, int pinEcho);
float distanciaParaPercentagem(float distanciaCm, float alturaTanqueCm);
void lerSensores();
void verificarAvarias();
void verificarAvariaSolar();
void maquinaEstadosCascata();
void maquinaEstadosSolar();
void atualizarSaidas();
void registarDadosSerial();
void ligarWiFi();
void reconectarMQTT();
void mqttCallback(char *topic, byte *payload, unsigned int length);
void publicarDiscovery();
void publicarEstado();

// ===================== SETUP =====================

void setup()
{
    Serial.begin(115200);
    delay(200);

    pinMode(PIN_START, INPUT_PULLUP);
    pinMode(PIN_STOP, INPUT_PULLUP);
    pinMode(PIN_EMERGENCIA, INPUT_PULLUP);
    pinMode(PIN_SELETOR_AUTO, INPUT_PULLUP);

    pinMode(PIN_TRIG_NIVEL_SUP, OUTPUT);
    pinMode(PIN_ECHO_NIVEL_SUP, INPUT);
    pinMode(PIN_TRIG_NIVEL_INF, OUTPUT);
    pinMode(PIN_ECHO_NIVEL_INF, INPUT);

    pinMode(PIN_CAUDAL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_CAUDAL), isrCaudal, FALLING);

    pinMode(PIN_BOMBA_IMPULSAO, OUTPUT);
    pinMode(PIN_BOMBA_RETORNO, OUTPUT);
    pinMode(PIN_BOMBA_RECIRC, OUTPUT);
    pinMode(PIN_SINAL_VERDE, OUTPUT);
    pinMode(PIN_SINAL_AMARELO, OUTPUT);
    pinMode(PIN_SINAL_VERMELHO, OUTPUT);
    pinMode(PIN_ALARME, OUTPUT);
    digitalWrite(PIN_BOMBA_IMPULSAO, LOW);
    digitalWrite(PIN_BOMBA_RETORNO, LOW);
    digitalWrite(PIN_BOMBA_RECIRC, LOW);

    sensoresTemp.begin();

    topicStatus = String(DEVICE_ID) + "/status";
    topicState = String(DEVICE_ID) + "/state";
    topicCmdPrefix = String(DEVICE_ID) + "/cmd/";

    ligarWiFi();
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setCallback(mqttCallback);
    mqtt.setBufferSize(2048); // discovery payloads são grandes

    Serial.println("Sistema Cascata UC01786 (ESP32 + Home Assistant) inicializado");
}

// ===================== LOOP PRINCIPAL =====================

void loop()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        if (millis() - tentativaWifiAnterior > 10000)
        {
            tentativaWifiAnterior = millis();
            ligarWiFi();
        }
    }
    else if (!mqtt.connected())
    {
        if (millis() - tentativaMqttAnterior > 5000)
        {
            tentativaMqttAnterior = millis();
            reconectarMQTT();
        }
    }
    else
    {
        mqtt.loop();
    }

    unsigned long agora = millis();
    if (agora - tempoAnteriorCiclo >= INTERVALO_LEITURA)
    {
        tempoAnteriorCiclo = agora;

        modoAutomatico = (digitalRead(PIN_SELETOR_AUTO) == HIGH);

        lerSensores();
        verificarAvarias();
        verificarAvariaSolar();
        maquinaEstadosCascata();
        maquinaEstadosSolar();
        atualizarSaidas();
        registarDadosSerial();
    }

    if (mqtt.connected() && (agora - tempoAnteriorPublish >= INTERVALO_PUBLISH))
    {
        tempoAnteriorPublish = agora;
        publicarEstado();
    }
}

// ===================== LEITURA DE SENSORES =====================

float lerDistanciaCm(int pinTrig, int pinEcho)
{
    digitalWrite(pinTrig, LOW);
    delayMicroseconds(2);
    digitalWrite(pinTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrig, LOW);

    unsigned long duracao = pulseIn(pinEcho, HIGH, 30000); // timeout 30ms
    if (duracao == 0)
        return -1.0; // sem eco = leitura inválida
    return duracao * 0.0343 / 2.0; // cm
}

float distanciaParaPercentagem(float distanciaCm, float alturaTanqueCm)
{
    if (distanciaCm < 0)
        return -1.0; // propaga leitura inválida
    float nivelAgua = alturaTanqueCm - distanciaCm; // altura de água a partir do fundo
    float pct = (nivelAgua / alturaTanqueCm) * 100.0;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return pct;
}

void lerSensores()
{
    float distSup = lerDistanciaCm(PIN_TRIG_NIVEL_SUP, PIN_ECHO_NIVEL_SUP);
    float distInf = lerDistanciaCm(PIN_TRIG_NIVEL_INF, PIN_ECHO_NIVEL_INF);

    float pctSup = distanciaParaPercentagem(distSup, ALTURA_TANQUE_SUP_CM);
    float pctInf = distanciaParaPercentagem(distInf, ALTURA_TANQUE_INF_CM);

    leiturasInvalidasNivelSup = (pctSup < 0) ? (leiturasInvalidasNivelSup + 1) : 0;
    leiturasInvalidasNivelInf = (pctInf < 0) ? (leiturasInvalidasNivelInf + 1) : 0;
    if (pctSup >= 0) nivelSupPct = pctSup;
    if (pctInf >= 0) nivelInfPct = pctInf;

    sensoresTemp.requestTemperatures();
    float te = sensoresTemp.getTempCByIndex(IDX_TEMP_ENTRADA);
    float ts = sensoresTemp.getTempCByIndex(IDX_TEMP_SAIDA);
    bool tempInvalida = (te == DEVICE_DISCONNECTED_C || ts == DEVICE_DISCONNECTED_C);
    leiturasInvalidasTemp = tempInvalida ? (leiturasInvalidasTemp + 1) : 0;
    if (!tempInvalida)
    {
        tempEntrada = te;
        tempSaida = ts;
        diferencialSolar = tempSaida - tempEntrada; // >0 = serpentina a aquecer a água
    }

    corrente = analogRead(PIN_CORRENTE);
    bool correnteSuspeita = (corrente <= 0 || corrente >= 4095);
    leiturasInvalidasCorrente = correnteSuspeita ? (leiturasInvalidasCorrente + 1) : 0;

    // Frequência de pulso do sensor de caudal -> L/min
    float hz = (contadorPulsos * 1000.0) / INTERVALO_LEITURA;
    contadorPulsos = 0;
    caudalLmin = hz / FATOR_CALIBRACAO_CAUDAL;
}

// ===================== DETEÇÃO DE AVARIAS - CASCATA =====================

void verificarAvarias()
{
    falhaDetetada = false;
    descricaoFalha = "";

    bool bombaImpulsaoLigada = (digitalRead(PIN_BOMBA_IMPULSAO) == HIGH);

    if (leiturasInvalidasNivelSup >= LEITURAS_INVALIDAS_PARA_AVARIA_SENSOR ||
        leiturasInvalidasNivelInf >= LEITURAS_INVALIDAS_PARA_AVARIA_SENSOR ||
        leiturasInvalidasTemp >= LEITURAS_INVALIDAS_PARA_AVARIA_SENSOR ||
        leiturasInvalidasCorrente >= LEITURAS_INVALIDAS_PARA_AVARIA_SENSOR)
    {
        falhaDetetada = true;
        descricaoFalha = "Sensor avariado";
    }
    else if (nivelSupPct < NIVEL_MIN_SEGURO_PCT)
    {
        falhaDetetada = true;
        descricaoFalha = "Falta de agua";
    }
    else if (nivelSupPct > NIVEL_MAX_SEGURO_PCT || nivelInfPct > NIVEL_MAX_SEGURO_PCT)
    {
        falhaDetetada = true;
        descricaoFalha = "Transbordo";
    }
    else if (caudalLmin < LIMIAR_CAUDAL_MIN_LMIN &&
             (estadoCascata == FUNCIONAMENTO || estadoCascata == RETORNO))
    {
        falhaDetetada = true;
        descricaoFalha = "Ausencia de caudal";
    }
    else if (corrente > LIMIAR_CORRENTE_MAX_RAW)
    {
        falhaDetetada = true;
        descricaoFalha = "Sobrecorrente";
    }
    else if (bombaImpulsaoLigada &&
             tempoLigadaImpulsao > 0 &&
             (millis() - tempoLigadaImpulsao) > TIMEOUT_ARRANQUE_BOMBA_MS &&
             corrente < CORRENTE_MIN_ARRANQUE_RAW)
    {
        falhaDetetada = true;
        descricaoFalha = "Bomba bloqueada";
    }

    if (estadoSolar == AVARIA_SOLAR)
    {
        // O enunciado lista "falha do aquecimento solar" como avaria da cascata,
        // por isso propaga-se para a máquina de estados principal.
        falhaDetetada = true;
        descricaoFalha = "Falha do aquecimento solar";
    }

    if (falhaDetetada && estadoCascata != AVARIA_CASCATA)
    {
        estadoCascata = AVARIA_CASCATA;
    }
}

void verificarAvariaSolar()
{
    if (estadoSolar == RECIRCULACAO)
    {
        if (tempSaida >= TEMP_MAXIMA_SERPENTINA)
        {
            estadoSolar = AVARIA_SOLAR;
            return;
        }
        // Se está a recircular há muito tempo e a água não aquece, algo
        // está mal (bomba parada, serpentina entupida, sensor trocado).
        if (tempoInicioRecirculacao > 0 &&
            (millis() - tempoInicioRecirculacao) > TIMEOUT_SOLAR_SEM_EFEITO_MS &&
            (tempSaida - tempSaidaNoInicioRecirc) < 1.0)
        {
            estadoSolar = AVARIA_SOLAR;
        }
    }
}

// ===================== MÁQUINA DE ESTADOS - CASCATA =====================

void maquinaEstadosCascata()
{
    bool start = (digitalRead(PIN_START) == LOW);
    bool stop = (digitalRead(PIN_STOP) == LOW);
    bool emergencia = (digitalRead(PIN_EMERGENCIA) == LOW);

    if (emergencia && estadoCascata != PARAGEM && estadoCascata != ESVAZIAMENTO && estadoCascata != PARADO)
    {
        estadoCascata = PARAGEM;
    }

    switch (estadoCascata)
    {
    case DESLIGADO:
        if (start)
            estadoCascata = VERIFICACAO;
        break;

    case VERIFICACAO:
        // Não avança com avaria pendente nem sensores inválidos.
        if (leiturasInvalidasNivelSup == 0 && leiturasInvalidasNivelInf == 0 &&
            leiturasInvalidasTemp == 0 && leiturasInvalidasCorrente == 0)
        {
            estadoCascata = ENCHIMENTO;
        }
        break;

    case ENCHIMENTO:
        if (nivelSupPct >= AUTOFILL_DESLIGA_PCT)
            estadoCascata = IMPULSAO;
        break;

    case IMPULSAO:
        // A bomba de impulsão liga em atualizarSaidas(); aqui só se confirma
        // arranque (via deteção de bomba bloqueada, que corre em paralelo).
        estadoCascata = FUNCIONAMENTO;
        break;

    case FUNCIONAMENTO:
        if (stop)
        {
            estadoCascata = PARAGEM;
        }
        else if (!modoAutomatico)
        {
            // Em manual, as bombas são controladas pelos comandos MQTT;
            // a máquina de estados só trata de segurança (avarias, stop).
        }
        else if (nivelSupPct < AUTOFILL_LIGA_PCT && nivelInfPct > NIVEL_INF_MIN_PCT)
        {
            estadoCascata = RETORNO;
        }
        break;

    case RETORNO:
        // Bomba de impulsão continua ligada (cascata não pára); bomba de
        // retorno também liga (ver atualizarSaidas) para repor o depósito
        // superior a partir do reservatório inferior.
        if (stop)
        {
            estadoCascata = PARAGEM;
        }
        else if (nivelSupPct >= AUTOFILL_DESLIGA_PCT || nivelInfPct <= NIVEL_INF_MIN_PCT)
        {
            estadoCascata = FUNCIONAMENTO;
        }
        break;

    case PARAGEM:
        estadoCascata = ESVAZIAMENTO;
        break;

    case ESVAZIAMENTO:
        estadoCascata = PARADO;
        break;

    case PARADO:
        if (start && !emergencia)
            estadoCascata = VERIFICACAO;
        break;

    case AVARIA_CASCATA:
        // só sai daqui com reset manual (START local, ou comando MQTT
        // "reset_avaria") depois de a condição de falha desaparecer.
        if (start && !falhaDetetada)
            estadoCascata = DESLIGADO;
        break;
    }

    if (digitalRead(PIN_BOMBA_IMPULSAO) == LOW)
        tempoLigadaImpulsao = 0; // será marcado em atualizarSaidas() quando ligar
}

// ===================== MÁQUINA DE ESTADOS - SOLAR =====================

void maquinaEstadosSolar()
{
    switch (estadoSolar)
    {
    case ESPERA:
        estadoSolar = VERIFICACAO_SOLAR;
        break;

    case VERIFICACAO_SOLAR:
        if (tempSaida >= TEMP_MAXIMA_SERPENTINA)
        {
            estadoSolar = TEMPERATURA_MAXIMA;
        }
        else if (diferencialSolar >= DELTA_TEMP_LIGA_SOLAR)
        {
            estadoSolar = RECIRCULACAO;
            tempoInicioRecirculacao = millis();
            tempSaidaNoInicioRecirc = tempSaida;
        }
        else
        {
            estadoSolar = ESPERA;
        }
        break;

    case RECIRCULACAO:
        if (tempSaida >= TEMP_MAXIMA_SERPENTINA)
        {
            estadoSolar = TEMPERATURA_MAXIMA;
        }
        else if (diferencialSolar < DELTA_TEMP_DESLIGA_SOLAR)
        {
            estadoSolar = ESPERA;
            tempoInicioRecirculacao = 0;
        }
        break;

    case TEMPERATURA_MAXIMA:
        if (tempSaida < (TEMP_MAXIMA_SERPENTINA - 5.0))
            estadoSolar = ESPERA;
        break;

    case AVARIA_SOLAR:
        // Sai da avaria solar quando a avaria geral da cascata for
        // reconhecida (START após resolver o problema).
        if (!falhaDetetada)
            estadoSolar = ESPERA;
        break;
    }
}

// ===================== ATUALIZAÇÃO DE SAÍDAS =====================

void atualizarSaidas()
{
    bool avaria = (estadoCascata == AVARIA_CASCATA);

    bool ligarImpulsao, ligarRetorno, ligarRecirc;

    if (avaria)
    {
        ligarImpulsao = ligarRetorno = ligarRecirc = false;
    }
    else if (!modoAutomatico)
    {
        // Modo manual: comandos vindos do Home Assistant, sempre sujeitos
        // aos intertravamentos de segurança (nunca secar nem transbordar).
        ligarImpulsao = cmdBombaImpulsaoManual && nivelSupPct > NIVEL_MIN_SEGURO_PCT;
        ligarRetorno = cmdBombaRetornoManual && nivelInfPct > NIVEL_INF_MIN_PCT &&
                       nivelSupPct < NIVEL_MAX_SEGURO_PCT;
        ligarRecirc = cmdBombaRecircManual;
    }
    else
    {
        ligarImpulsao = (estadoCascata == IMPULSAO || estadoCascata == FUNCIONAMENTO || estadoCascata == RETORNO);
        ligarRetorno = (estadoCascata == RETORNO);
        ligarRecirc = (estadoSolar == RECIRCULACAO);
    }

    bool estavaLigadaImpulsao = (digitalRead(PIN_BOMBA_IMPULSAO) == HIGH);
    if (ligarImpulsao && !estavaLigadaImpulsao)
        tempoLigadaImpulsao = millis();
    else if (!ligarImpulsao)
        tempoLigadaImpulsao = 0;

    digitalWrite(PIN_BOMBA_IMPULSAO, ligarImpulsao);
    digitalWrite(PIN_BOMBA_RETORNO, ligarRetorno);
    digitalWrite(PIN_BOMBA_RECIRC, ligarRecirc);

    digitalWrite(PIN_ALARME, avaria);
    digitalWrite(PIN_SINAL_VERMELHO, avaria);
    digitalWrite(PIN_SINAL_VERDE, (estadoCascata == FUNCIONAMENTO || estadoCascata == RETORNO) && !avaria);
    digitalWrite(PIN_SINAL_AMARELO, (estadoCascata == VERIFICACAO || estadoCascata == ENCHIMENTO));
}

// ===================== DEBUG SÉRIE =====================

void registarDadosSerial()
{
    Serial.printf(
        "Cascata=%s Solar=%s Modo=%s NivelSup=%.1f%% NivelInf=%.1f%% TempEnt=%.1f TempSai=%.1f Caudal=%.2f Corrente=%d",
        nomeEstadoCascata(estadoCascata), nomeEstadoSolar(estadoSolar),
        modoAutomatico ? "AUTO" : "MANUAL",
        nivelSupPct, nivelInfPct, tempEntrada, tempSaida, caudalLmin, corrente);
    if (falhaDetetada)
        Serial.printf(" FALHA=%s", descricaoFalha.c_str());
    Serial.println();
}

// ===================== WIFI / MQTT =====================

void ligarWiFi()
{
    Serial.printf("A ligar ao WiFi \"%s\"...\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    String msg;
    for (unsigned int i = 0; i < length; i++)
        msg += (char)payload[i];
    String t(topic);

    bool ligar = (msg == "ON" || msg == "1" || msg == "true");

    if (t == topicCmdPrefix + "bomba_impulsao_manual")
        cmdBombaImpulsaoManual = ligar;
    else if (t == topicCmdPrefix + "bomba_retorno_manual")
        cmdBombaRetornoManual = ligar;
    else if (t == topicCmdPrefix + "bomba_recirc_manual")
        cmdBombaRecircManual = ligar;
    else if (t == topicCmdPrefix + "reset_avaria")
    {
        // Equivalente remoto ao botão START quando em AVARIA.
        if (estadoCascata == AVARIA_CASCATA && !falhaDetetada)
            estadoCascata = DESLIGADO;
    }
}

void reconectarMQTT()
{
    Serial.print("A ligar ao MQTT...");
    String clientId = String(DEVICE_ID) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    bool ok = mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD,
                            topicStatus.c_str(), 0, true, "offline");
    if (ok)
    {
        Serial.println(" ligado.");
        mqtt.publish(topicStatus.c_str(), "online", true);
        mqtt.subscribe((topicCmdPrefix + "#").c_str());
        publicarDiscovery();
        discoveryPublicado = true;
    }
    else
    {
        Serial.printf(" falhou, rc=%d\n", mqtt.state());
    }
}

// Helper para publicar um "config" de MQTT Discovery do Home Assistant.
void publicarConfigEntidade(const char *componente, const char *objectId,
                             const char *nome, const char *icone,
                             const char *unidade, const char *deviceClass,
                             const char *valueTemplate, bool comando)
{
    JsonDocument doc;
    doc["name"] = nome;
    doc["unique_id"] = String(DEVICE_ID) + "_" + objectId;
    doc["object_id"] = String(DEVICE_ID) + "_" + objectId; // entity_id previsível para a app
    doc["state_topic"] = topicState;
    doc["value_template"] = valueTemplate;
    doc["availability_topic"] = topicStatus;
    if (icone) doc["icon"] = icone;
    if (unidade) doc["unit_of_measurement"] = unidade;
    if (deviceClass) doc["device_class"] = deviceClass;
    if (comando)
    {
        doc["command_topic"] = topicCmdPrefix + objectId;
        doc["payload_on"] = "ON";
        doc["payload_off"] = "OFF";
    }

    JsonObject device = doc["device"].to<JsonObject>();
    device["identifiers"][0] = DEVICE_ID;
    device["name"] = "Cascata UC01786";
    device["manufacturer"] = "Projeto pessoal";
    device["model"] = "ESP32 + Home Assistant";

    String payload;
    serializeJson(doc, payload);

    String topic = "homeassistant/" + String(componente) + "/" + DEVICE_ID + "/" + objectId + "/config";
    mqtt.publish(topic.c_str(), payload.c_str(), true);
}

void publicarDiscovery()
{
    publicarConfigEntidade("sensor", "nivel_sup", "Nível Depósito Superior", "mdi:water", "%", nullptr, "{{ value_json.nivel_sup_pct }}", false);
    publicarConfigEntidade("sensor", "nivel_inf", "Nível Reservatório Inferior", "mdi:water-outline", "%", nullptr, "{{ value_json.nivel_inf_pct }}", false);
    publicarConfigEntidade("sensor", "temp_entrada", "Temperatura Entrada Serpentina", nullptr, "°C", "temperature", "{{ value_json.temp_entrada_c }}", false);
    publicarConfigEntidade("sensor", "temp_saida", "Temperatura Saída Serpentina", nullptr, "°C", "temperature", "{{ value_json.temp_saida_c }}", false);
    publicarConfigEntidade("sensor", "diferencial", "Diferencial Solar", "mdi:thermometer-lines", "°C", "temperature", "{{ value_json.diferencial_c }}", false);
    publicarConfigEntidade("sensor", "caudal", "Caudal", "mdi:pump", "L/min", nullptr, "{{ value_json.caudal_lmin }}", false);
    publicarConfigEntidade("sensor", "corrente", "Corrente Bomba Impulsão", "mdi:current-ac", nullptr, "current", "{{ value_json.corrente_raw }}", false);
    publicarConfigEntidade("sensor", "estado_cascata", "Estado Cascata", "mdi:state-machine", nullptr, nullptr, "{{ value_json.estado_cascata }}", false);
    publicarConfigEntidade("sensor", "estado_solar", "Estado Solar", "mdi:weather-sunny", nullptr, nullptr, "{{ value_json.estado_solar }}", false);
    publicarConfigEntidade("sensor", "modo", "Modo de Operação", "mdi:tune", nullptr, nullptr, "{{ value_json.modo }}", false);
    publicarConfigEntidade("binary_sensor", "falha", "Falha", "mdi:alert", nullptr, "problem", "{{ 'ON' if value_json.falha else 'OFF' }}", false);
    publicarConfigEntidade("sensor", "descricao_falha", "Descrição da Falha", "mdi:message-alert", nullptr, nullptr, "{{ value_json.descricao_falha }}", false);
    publicarConfigEntidade("binary_sensor", "bomba_impulsao", "Bomba de Impulsão (estado)", "mdi:pump", nullptr, "running", "{{ 'ON' if value_json.bomba_impulsao else 'OFF' }}", false);
    publicarConfigEntidade("binary_sensor", "bomba_retorno", "Bomba de Retorno (estado)", "mdi:pump", nullptr, "running", "{{ 'ON' if value_json.bomba_retorno else 'OFF' }}", false);
    publicarConfigEntidade("binary_sensor", "bomba_recirc", "Bomba Recirculadora (estado)", "mdi:pump", nullptr, "running", "{{ 'ON' if value_json.bomba_recirc else 'OFF' }}", false);
    publicarConfigEntidade("sensor", "rssi", "Sinal WiFi", "mdi:wifi", "dBm", "signal_strength", "{{ value_json.rssi }}", false);

    // Switches de comando manual (só fazem efeito com o seletor físico em Manual).
    publicarConfigEntidade("switch", "bomba_impulsao_manual", "Bomba de Impulsão (manual)", "mdi:pump", nullptr, nullptr, "{{ value_json.bomba_impulsao }}", true);
    publicarConfigEntidade("switch", "bomba_retorno_manual", "Bomba de Retorno (manual)", "mdi:pump", nullptr, nullptr, "{{ value_json.bomba_retorno }}", true);
    publicarConfigEntidade("switch", "bomba_recirc_manual", "Bomba Recirculadora (manual)", "mdi:pump", nullptr, nullptr, "{{ value_json.bomba_recirc }}", true);

    // Botão de reset remoto de avaria (equivalente ao START local).
    JsonDocument doc;
    doc["name"] = "Reset Avaria";
    doc["unique_id"] = String(DEVICE_ID) + "_reset_avaria";
    doc["object_id"] = String(DEVICE_ID) + "_reset_avaria";
    doc["command_topic"] = topicCmdPrefix + "reset_avaria";
    doc["payload_press"] = "ON";
    doc["availability_topic"] = topicStatus;
    doc["icon"] = "mdi:restart-alert";
    JsonObject device = doc["device"].to<JsonObject>();
    device["identifiers"][0] = DEVICE_ID;
    device["name"] = "Cascata UC01786";
    String payload;
    serializeJson(doc, payload);
    mqtt.publish(("homeassistant/button/" + String(DEVICE_ID) + "/reset_avaria/config").c_str(), payload.c_str(), true);
}

void publicarEstado()
{
    JsonDocument doc;
    doc["nivel_sup_pct"] = roundf(nivelSupPct * 10) / 10.0;
    doc["nivel_inf_pct"] = roundf(nivelInfPct * 10) / 10.0;
    doc["temp_entrada_c"] = roundf(tempEntrada * 10) / 10.0;
    doc["temp_saida_c"] = roundf(tempSaida * 10) / 10.0;
    doc["diferencial_c"] = roundf(diferencialSolar * 10) / 10.0;
    doc["caudal_lmin"] = roundf(caudalLmin * 10) / 10.0;
    doc["corrente_raw"] = corrente;
    doc["estado_cascata"] = nomeEstadoCascata(estadoCascata);
    doc["estado_cascata_num"] = (int)estadoCascata;
    doc["estado_solar"] = nomeEstadoSolar(estadoSolar);
    doc["estado_solar_num"] = (int)estadoSolar;
    doc["modo"] = modoAutomatico ? "AUTOMATICO" : "MANUAL";
    doc["falha"] = falhaDetetada;
    doc["descricao_falha"] = descricaoFalha;
    doc["bomba_impulsao"] = digitalRead(PIN_BOMBA_IMPULSAO) == HIGH;
    doc["bomba_retorno"] = digitalRead(PIN_BOMBA_RETORNO) == HIGH;
    doc["bomba_recirc"] = digitalRead(PIN_BOMBA_RECIRC) == HIGH;
    doc["rssi"] = WiFi.RSSI();
    doc["uptime_s"] = millis() / 1000;

    String payload;
    serializeJson(doc, payload);
    mqtt.publish(topicState.c_str(), payload.c_str(), true);
}
