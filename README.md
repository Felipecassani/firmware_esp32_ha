# Cascata UC01786 — firmware ESP32 + Home Assistant

Versão experimental, para hardware real, do projeto da cascata. **Não é o
entregável da escola** (esse mantém-se em Arduino Uno/Mega puro, sem rede,
em `../cascata_UC01786` e `../projeto cascata`, para correr no Tinkercad
conforme pedido na Ficha Prática nº1).

Aqui o ESP32 faz tudo: lê os sensores, corre as duas máquinas de estados
(cascata + circuito solar) e publica tudo por MQTT para o Home Assistant,
onde as entidades aparecem sozinhas (MQTT Discovery) — não é preciso
escrever `configuration.yaml` nenhum.

## 1. Mapa de pinos (ESP32 DevKit 30 pinos / WROOM-32)

| Sinal | Pino ESP32 | Notas |
|---|---|---|
| START (botão) | GPIO32 | INPUT_PULLUP, ativo em LOW |
| STOP (botão) | GPIO33 | INPUT_PULLUP, ativo em LOW |
| EMERGÊNCIA | GPIO25 | INPUT_PULLUP, ativo em LOW — usar botão de emergência real com retenção |
| Seletor Auto/Manual | GPIO26 | INPUT_PULLUP. Aberto = Automático, ligado a GND = Manual |
| TRIG nível superior | GPIO27 | HC-SR04 do depósito 500L |
| ECHO nível superior | GPIO14 | usar divisor de tensão 5V→3.3V no ECHO |
| TRIG nível inferior | GPIO16 | HC-SR04 do reservatório inferior |
| ECHO nível inferior | GPIO17 | idem, divisor de tensão |
| Sensor de caudal (YF-S201) | GPIO4 | interrupção por flanco descendente |
| DS18B20 (entrada + saída, mesmo barramento) | GPIO13 | resistência pull-up 4.7kΩ para 3.3V |
| ACS712 (corrente) | GPIO36 (VP) | **alimentar o módulo a 3.3V**, não a 5V (ver secção 3) |
| Bomba de impulsão (relé) | GPIO18 | |
| Bomba de retorno (relé) | GPIO19 | |
| Bomba recirculadora solar (relé) | GPIO21 | |
| Sinal verde | GPIO22 | |
| Sinal amarelo | GPIO23 | |
| Sinal vermelho | GPIO5 | pino de strapping — sem problema para saída, só não ligar nada que o force a HIGH no boot |
| Alarme (buzzer/sirene) | GPIO15 | idem |

Pinos GPIO0, GPIO2, GPIO12 e GPIO6-11 foram propositadamente evitados
(strapping/flash interna do ESP32 — usá-los para periféricos externos
pode impedir o arranque).

## 2. Bibliotecas

Instaladas automaticamente pelo PlatformIO a partir do `platformio.ini`:
`PubSubClient`, `ArduinoJson`, `OneWire`, `DallasTemperature`.

```bash
cd firmware_esp32_ha
pio run              # compila
pio run -t upload    # grava no ESP32 (por USB)
pio device monitor    # abre o monitor série (115200 baud)
```

## 3. Ligações elétricas — pontos de atenção

- **Relés**: usar módulo de relé com optoacoplador. As bombas/220V ficam do
  lado de alta tensão do relé — nunca ligar a rede diretamente ao ESP32.
- **HC-SR04 (5V)**: o pino ECHO devolve 5V; o ESP32 só aceita 3.3V nas
  entradas. Usar um divisor resistivo (ex.: 1kΩ + 2kΩ) no ECHO antes de
  ligar ao GPIO.
- **ACS712**: alimentar o módulo a 3.3V (em vez dos 5V habituais) para que
  a saída analógica (centrada em Vcc/2) já fique dentro do intervalo
  0–3.3V do ADC do ESP32, sem precisar de mais um divisor de tensão.
- **DS18B20**: sondas estanques, um único barramento de dados no GPIO13
  com resistência pull-up de 4.7kΩ para 3.3V, partilhada pelas duas sondas
  (entrada e saída da serpentina).

## 4. Configurar WiFi e MQTT

```bash
cp include/secrets_example.h include/secrets.h
```

Editar `include/secrets.h` com o SSID/password da rede e os dados do
broker MQTT. `secrets.h` está no `.gitignore` — nunca é versionado.

Se ainda não tens um broker MQTT: no Home Assistant, vai a
**Definições → Extras → Loja de add-ons** e instala o **"Mosquitto
broker"**. Depois, em **Definições → Dispositivos e serviços**, o Home
Assistant deteta o Mosquitto automaticamente e propõe configurar a
integração **MQTT** — aceita. O `MQTT_HOST` em `secrets.h` é o IP da
máquina onde corre o Home Assistant, porta `1883`.

## 5. O que aparece no Home Assistant

Assim que o ESP32 liga ao broker, publica os "config" de MQTT Discovery e
em segundos aparece um dispositivo **"Cascata UC01786"** com estas
entidades:

- Sensores: nível do depósito superior e inferior (%), temperatura de
  entrada/saída da serpentina, diferencial solar, caudal, corrente,
  estado da cascata, estado do circuito solar, modo (Automático/Manual),
  descrição da falha, sinal WiFi.
- `binary_sensor.cascata_uc01786_falha` (classe *problem* — fica vermelho
  no HA quando há avaria) e três `binary_sensor` de estado das bombas.
- `switch` para cada bomba, para comando manual remoto (só têm efeito
  real quando o seletor físico está em **Manual** — é um intertravamento
  de segurança propositado, o ESP32 ignora comandos remotos em modo
  Automático).
- `button.cascata_uc01786_reset_avaria`, equivalente remoto ao START
  local depois de resolvida a avaria.

Todas as entidades ficam indisponíveis automaticamente se o ESP32 cair
(Last Will and Testament no tópico `cascata_uc01786/status`).

## 6. Calibração (fazer com o hardware montado)

Estes valores em `src/main.cpp` são placeholders — sem isto, os alarmes de
falta de água/transbordo vão disparar errado:

- `ALTURA_TANQUE_SUP_CM` / `ALTURA_TANQUE_INF_CM`: medir a distância real
  do sensor ultrassónico (no topo de cada depósito) até ao fundo.
- `LIMIAR_CORRENTE_MAX_RAW` / `CORRENTE_MIN_ARRANQUE_RAW`: registar a
  leitura bruta do ADC com a bomba desligada, parada-mas-ligada e a
  trabalhar normalmente (usar `pio device monitor` para ver os valores em
  tempo real), depois ajustar os limiares.
- `FATOR_CALIBRACAO_CAUDAL`: o datasheet do YF-S201 dá 7.5 Hz por L/min,
  mas varia de sensor para sensor — comparar com um recipiente e
  cronómetro e ajustar.
- Ordem dos DS18B20 no barramento: o código assume índice 0 = entrada,
  1 = saída. Se estiverem trocados, basta inverter `IDX_TEMP_ENTRADA` /
  `IDX_TEMP_SAIDA`.

## 7. App de controlo

A app iOS nativa (projeto `../cascade`) fala com a API do Home Assistant
(REST + WebSocket), não diretamente com o ESP32 — por isso funciona a
partir de qualquer sítio (não só na mesma rede local), tal como as
entidades acima descritas. Ver `../cascade/README.md`.
