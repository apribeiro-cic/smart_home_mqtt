# Smart Home System (com protocolo MQTT) 🏠

Este é um projeto desenvolvido para a placa de desenvolvimento **BitDogLab**, baseada no microcontrolador **Raspberry Pi Pico W**.  
O objetivo é simular um sistema de automação residencial, permitindo o controle de diversos dispositivos como iluminação, trava, alarme, display e ar-condicionado simulado via **broker MQTT** acessado remotamente por um dashboard móvel (IoT MQTT Panel).

---

## 📌 Sobre o Projeto

O Smart Home System foi criado como **projeto prático** da **2ª fase da residência tecnológica EmbarcaTech**.  
O projeto integra diversos periféricos da BitDogLab e demonstra na prática o uso de GPIOs, I2C, ADC, PIO, Wi-Fi e comunicação por MQTT para simular uma residência conectada e controlável remotamente.

Essa versão é similar ao projeto anterior (feito com um HTTP webserver), mas migra de servidor web para protocolo MQTT.

---

## 🧠 Como funciona

O sistema opera continuamente, aguardando comandos recebidos por meio de tópicos MQTT (por exemplo, enviados pelo IoT MQTT Panel conectado à mesma rede). Comandos reconhecidos ativam ou desativam dispositivos da casa inteligente:

- **Controle de LEDs** (simulam luzes de cômodos) via tópico `"/light/set"` e publica estado em `"/light/state"`.  
- **Trava de porta** (Matriz de LEDs simula trancado/destrancado) via tópico `"/lock/set"` e publica estado em `"/lock/state"`.  
- **Alarme sonoro** (Buzzer), ativado por `"/alarm/set"` ou simulado via joystick; publica estado em `"/alarm/state"`.  
- **Matriz de LEDs WS2812B** (feedback visual da fechadura eletrônica); controlada localmente a partir de `lock_state` e variáveis de cor.  
- **Display OLED** (feedback visual de informações do sistema, incluindo status de A/C); exibe “Alarme: ON/OFF”, “Fechadura: ON/OFF”, “A/C: ON/OFF” e temperatura.  
- **Joystick** (simula um sensor de movimento para ativação do alarme).  
- **Sistema de Refrigeração Simulado**: simula um ar-condicionado controlado remotamente, com controle de temperatura via tópico `"/cooling/temp"` e estado via `"/cooling/set"`.

### 🔄 Comandos disponíveis

A dashboard MQTT envia payloads simples a tópicos específicos, por exemplo:
- `/light/set` → `"1"` ou `"0"` (liga/desliga luz)  
- `/lock/set` → `"1"` ou `"0"` (travar/destravar porta)  
- `/alarm/set` → `"1"` ou `"0"` (ativar/desativar alarme)  
- `/cooling/temp` → valor em °C (ajusta `simulated_temp`)  
- `/cooling/set` → `"1"` ou `"0"` (liga/desliga A/C)  

Esses comandos são interpretados pelo firmware na placa, que executa ações locais instantaneamente, inclusive publicando nos tópicos `"/light/state"`, `"/lock/state"`, `"/alarm/state"`, `"/cooling/state"` e `"/cooling/temp/state"` (retained) para manter o dashboard sincronizado com os valores atuais.

---

## 📁 Utilização

Atendendo aos requisitos de organização da 2ª fase da residência, o arquivo CMakeLists.txt está configurado para facilitar a importação do projeto no Visual Studio Code.  
Segue as instruções:

1. Na barra lateral, clique em **Raspberry Pi Pico Project** e depois em **Import Project**.

   ![image](https://github.com/user-attachments/assets/4b1ed8c7-6730-4bfe-ae1f-8a26017d1140)

2. Selecione o diretório do projeto e clique em **Import** (utilizando a versão **2.1.1** do Pico SDK).

   ![image](https://github.com/user-attachments/assets/be706372-b918-4ade-847e-12706af0cc99)

3. **IMPORTANTE**! Para o código funcionar é necessário trocar os parâmetros de SSID, SENHA do Wi-Fi, host MQTT, usuário e senha do MQTT (Linhas 24 a 28 do smart_home_mqtt.c) para os da sua rede local. Lembrando também de rodar o broker MQTT remotamente, por exemplo, em um celular android, através do app Termux.

4. No IoT MQTT Panel (ou similar), configure a dashboard (passando o endereço IP do dispositivo rodando via Termux) e crie widgets para:  
   - Botões que publicam em `/light/set`, `/lock/set`, `/alarm/set`, `/cooling/set`  
   - Slider que publica em `/cooling/temp` (com Retain habilitado)  
   - Indicadores que leem `/light/state`, `/lock/state`, `/alarm/state`, `/cooling/state`, `/cooling/temp/state`, `/temperature`

5. Agora, basta **compilar** e **rodar** o projeto, com a placa **BitDogLab** conectada e o broker MQTT ativo no Termux.

---
