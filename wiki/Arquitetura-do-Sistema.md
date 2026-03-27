# Arquitetura do Sistema AEB

> **Página:** Arquitetura do Sistema
> **Relacionado:** [Home](Home.md) | [Módulos C Embarcado](Modulos-C-Embarcado.md) | [Máquina de Estados](Maquina-de-Estados.md) | [Controlador PID](Controlador-PID.md)

---

## Filosofia de design em 3 camadas

A principal decisão arquitetural deste projeto é a **separação completa entre lógica de segurança embarcada e middleware de comunicação**. Esta escolha é deliberada e reflete práticas da indústria automotiva por três razões fundamentais:

### 1. Portabilidade e certificação do núcleo de segurança

O núcleo de lógica AEB (Camada 1) é escrito em C puro, compatível com C99, sem dependências externas além da biblioteca padrão limitada. Isso significa que o mesmo código-fonte pode ser compilado para:

- Um microcontrolador automotivo real (ex.: Renesas RH850, NXP S32K) com um compilador certificado MISRA
- O ambiente de simulação ROS2 (via wrapper C++)
- Uma bancada de testes *Software-in-the-Loop* (SIL) em Python via CFFI
- Um *Hardware-in-the-Loop* (HIL) com dSPACE ou National Instruments

A ISO 26262 exige que o código de nível ASIL seja desenvolvido com rastreabilidade de requisitos e que modificações de plataforma não alterem a lógica de segurança. A separação em camadas **garante esse isolamento**: a Camada 1 nunca "sabe" se está rodando em um MCU ou em uma VM Linux.

### 2. Testabilidade independente

Cada camada pode ser testada de forma isolada:

- **Camada 1** pode ser testada com *unit tests* em C puro (ex.: Unity framework), simulando entradas via structs
- **Camada 2** pode ser testada com mocks de tópicos ROS2, sem Gazebo
- **Camada 3** pode ser testada com nós ROS2 dummy, sem o controlador real

### 3. Fidelidade ao design de ECU de produção

Em uma ECU automotiva real, o código de aplicação (nível AUTOSAR Application Layer ou equivalente) é executado em ciclos fixos pelo RTOS e nunca acessa hardware diretamente. A Camada 2 faz o papel do **AUTOSAR RTE** (*Runtime Environment*), abstraindo sensores e atuadores. A Camada 3 faz o papel do **ambiente veicular** (barramento CAN, sensores físicos).

---

## Camada 1: Núcleo C Embarcado

### Módulos e responsabilidades

A Camada 1 consiste em **6 módulos C** com interfaces bem definidas:

| Módulo | Arquivo | Responsabilidade principal |
|--------|---------|---------------------------|
| `aeb_config` | `include/aeb_config.h` | Todos os parâmetros de calibração (#define). Nenhuma lógica. |
| `aeb_types` | `include/aeb_types.h` | Tipos de dados compartilhados: enums, structs. Nenhuma lógica. |
| `aeb_perception` | `src/aeb_perception.c` | Validação e filtragem dos dados brutos do sensor |
| `aeb_ttc` | `src/aeb_ttc.c` | Cálculo do Time-to-Collision e distância de frenagem |
| `aeb_fsm` | `src/aeb_fsm.c` | Máquina de estados (7 estados), escalonamento de ameaça |
| `aeb_pid` | `src/aeb_pid.c` | Controlador PID com anti-windup e limitador de jerk |
| `aeb_alert` | `src/aeb_alert.c` | Geração de comandos de alerta visual e sonoro |
| `aeb_can` | `src/aeb_can.c` | Codificação/decodificação de frames CAN (8 bytes, escala ×100) e detecção de timeout |
| `aeb_uds` | `src/aeb_uds.c` | Diagnósticos ISO 14229: sessões, DTCs, ReadDID, WriteDataByIdentifier, Security Access |
| `aeb_main` | `src/aeb_main.c` | Orquestrador: `aeb_init()` e `aeb_cycle_10ms()` |

### Interfaces de entrada e saída

```
ENTRADAS (a cada ciclo de 10 ms):
  SensorData_t {
    float range_m;          // distância ao alvo [m]
    float target_speed_mps; // velocidade do alvo [m/s]
    uint8_t valid;          // flag de validade do sensor
  }
  float v_ego_mps;          // velocidade do veículo hospedeiro [m/s]

SAÍDAS (a cada ciclo de 10 ms):
  float brake_pct;          // pressão de freio [0-100%]
  AEB_State_t state;        // estado atual da FSM
  uint8_t alert_visual;     // flag de alerta visual
  uint8_t alert_audible;    // flag de alerta sonoro
```

### Regras MISRA C:2012 aplicadas

- **Sem `malloc`/`free`**: toda memória é estática ou em pilha, alocada em tempo de compilação
- **Sem recursão**: todas as funções têm profundidade de chamada estática e verificável
- **Tipos de largura fixa**: `uint8_t`, `int16_t`, `float32_t` em vez de `int`, `double`
- **Todas as variáveis inicializadas**: inicialização explícita no ponto de declaração
- **Sem `goto`**: fluxo de controle estruturado apenas com `if/else`, `switch`, `for`, `while`
- **Limites de array verificados**: todos os acessos a arrays têm checagem de bounds

---

## Camada 2: Wrapper ROS2 C++ (`aeb_controller_node`)

### Papel do nó controlador

O `aeb_controller_node.cpp` é o **ponto de integração** entre o ecossistema ROS2 e o núcleo C embarcado. Ele realiza:

1. **Subscrição de tópicos de entrada:**
   - `/radar/detection` — dados brutos do radar virtual (range, velocidade relativa)
   - `/ego_vehicle/odom` — odometria do veículo ego (velocidade longitudinal)

2. **Chamada periódica do ciclo C:**
   - Um timer ROS2 de 10 ms (`wall_timer`) chama `aeb_cycle_10ms()` exatamente a 100 Hz
   - Os dados de sensores mais recentes são copiados para a struct `SensorData_t` antes da chamada

3. **Publicação de resultados:**
   - `/aeb/brake_command` — pressão de freio calculada (tipo `std_msgs/Float32`)
   - `/aeb/state` — estado atual da FSM (tipo `std_msgs/Int32`)
   - `/aeb/alerts` — flags de alerta (tipo `std_msgs/UInt8MultiArray`)
   - `/can/aeb_output` — mensagem CAN simulada com todos os campos (tipo customizado)

### Guardas de prontidão (readiness guards)

O nó mantém duas flags booleanas que previnem chamadas ao ciclo C com dados inválidos:

```cpp
bool radar_ready_ = false;   // true após primeiro callback de /radar/detection
bool ego_ready_   = false;   // true após primeiro callback de /ego_vehicle/odom

void timer_callback() {
    if (!radar_ready_ || !ego_ready_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "AEB aguardando sensores...");
        return;  // não chama aeb_cycle_10ms() até ambos estarem prontos
    }
    // ... copia dados e chama o ciclo
}
```

Sem essas guardas, o primeiro ciclo seria executado com `range_m = 0.0` e `v_ego = 0.0`, potencialmente ativando a FSM com dados inválidos.

### Sincronização de `v_target_cached_`

Um problema sutil de sincronização ocorre porque o radar publica `range_m` e `target_speed_mps` separadamente em alguns cenários. A variável `v_target_cached_` armazena a última velocidade do alvo recebida, garantindo consistência entre os campos da struct passada ao ciclo C:

```cpp
float v_target_cached_ = 0.0f;

void radar_callback(const RadarMsg::SharedPtr msg) {
    sensor_data_.range_m = msg->range;
    sensor_data_.target_speed_mps = msg->target_speed;
    v_target_cached_ = msg->target_speed;  // cache explícito
    radar_ready_ = true;
}

void timer_callback() {
    // ...
    sensor_data_.target_speed_mps = v_target_cached_;  // usa cache
    aeb_cycle_10ms(&sensor_data_, v_ego_);
}
```

### Publicação de mensagens CAN simuladas

O nó publica uma mensagem CAN sintética que reproduz o formato de uma ECU real. O identificador CAN `0x3A0` carrega:

| Byte | Campo | Tipo |
|------|-------|------|
| 0-1 | `brake_pressure_pct` (x10) | `uint16_t` |
| 2 | `aeb_state` | `uint8_t` |
| 3 | `alert_flags` (bit 0=visual, bit 1=audible) | `uint8_t` |
| 4-5 | `ttc_ms` (TTC em milissegundos) | `uint16_t` |
| 6-7 | reservado | `uint8_t` |

---

## Camada 3: Nós Gazebo de Simulação

### `perception_node` — Fusão radar + lidar

O `perception_node` simula o comportamento de um sensor fusionado (como seria em um veículo com sensor de fusão radar/câmera). Ele:

- Subscreve os *plugins* Gazebo de **radar** (range + velocidade radial) e **lidar** (nuvem de pontos)
- Realiza uma fusão simples: usa o radar para velocidade relativa e o lidar para refinamento de distância em alcance curto (< 15 m)
- Publica no tópico `/radar/detection` com a struct consolidada

A fusão é necessária porque o *plugin* de radar Gazebo tem ruído significativo em distâncias curtas (< 5 m), enquanto o lidar é mais preciso nesses casos.

### `scenario_controller` — Controle do cenário de teste

O `scenario_controller` implementa os cenários Euro NCAP CCR:

| Cenário | Descrição |
|---------|-----------|
| `CCR-s` | Alvo estacionário, ego a 10-50 km/h |
| `CCR-m` | Alvo em movimento (25% da velocidade do ego) |
| `CCR-b` | Alvo em movimento (75% da velocidade do ego), travagem súbita |

Ele controla dinamicamente a velocidade do veículo alvo via serviço ROS2, publica `/scenario/active` e registra métricas de resultado (colisão evitada, distância residual, TTC mínimo atingido).

### `dashboard_node` — Visualização em tempo real

O `dashboard_node` exibe no terminal (via `ncurses`) um painel atualizado a 10 Hz com:

- Estado atual da FSM (com código de cor)
- Velocidade do ego e do alvo (m/s e km/h)
- Distância ao alvo e TTC calculado
- Pressão de freio atual e alvo (barra de progresso)
- Histórico de transições de estado (últimas 10)

---

## Fluxo de dados: diagrama do ciclo de 10 ms

O diagrama abaixo mostra o pipeline completo desde o sensor Gazebo até o comando de freio, em um único ciclo de 10 ms:

```
  GAZEBO WORLD (física simulada)
       │
       │  plugin sensor (50 Hz → decimado para 100 Hz)
       ▼
  perception_node
  [fusão radar+lidar]
       │
       │  /radar/detection  (ROS2 topic, 100 Hz)
       ▼
  aeb_controller_node  ◄─── /ego_vehicle/odom (ROS2 topic, 100 Hz)
  [timer 10ms dispara]
       │
       │  copia SensorData_t + v_ego para estruturas C
       ▼
  ╔══════════════════════════════════════╗
  ║   aeb_cycle_10ms()  — Camada 1 C    ║
  ║                                      ║
  ║  1. aeb_perception_update()          ║
  ║     ├─ valida range [0.5, 300] m     ║
  ║     ├─ valida speed [0, 50] m/s      ║
  ║     ├─ detecta rate-of-change excs.  ║
  ║     └─ saída: PerceptionData_t       ║
  ║                                      ║
  ║  2. aeb_ttc_calculate()              ║
  ║     ├─ v_rel = v_ego - v_target      ║
  ║     ├─ TTC = d / v_rel (se v_rel>0.5)║
  ║     ├─ d_brake = v²/(2×DECEL_L3)     ║
  ║     └─ saída: TTCResult_t            ║
  ║                                      ║
  ║  3. aeb_fsm_update()                 ║
  ║     ├─ evaluate_threat()             ║
  ║     ├─ regras de transição + hist.   ║
  ║     ├─ WARNING_TO_BRAKE_MIN check    ║
  ║     └─ saída: FSMOutput_t            ║
  ║                                      ║
  ║  4. aeb_pid_update()                 ║
  ║     ├─ error = target - actual_decel ║
  ║     ├─ PI com anti-windup            ║
  ║     ├─ jerk limiter (1%/ciclo = 6 m/s³, max 10 m/s³) ║
  ║     └─ saída: brake_pct [0-100]      ║
  ║                                      ║
  ║  5. aeb_alert_update()               ║
  ║     └─ saída: visual, audible flags  ║
  ╚══════════════════════════════════════╝
       │
       │  brake_pct, state, alerts
       ▼
  aeb_controller_node
  [publica tópicos ROS2 + msg CAN]
       │
       ├──► /aeb/brake_command
       ├──► /aeb/state
       ├──► /aeb/alerts
       └──► /can/aeb_output (0x3A0)
                │
                ▼
          GAZEBO WORLD
          [aplica torque de freio ao modelo do veículo]
```

**Latência total do ciclo:** < 1 ms em hardware de desenvolvimento (i7, 16 GB RAM), deixando 9 ms de folga no período de 10 ms.

---

## Por que esta arquitetura espelha uma ECU automotiva real

| Aspecto | ECU de Produção | Este Projeto |
|---------|----------------|--------------|
| Linguagem do núcleo | C (MISRA C) em AUTOSAR BSW/SWC | C (MISRA C) na Camada 1 |
| Ciclo de execução | RTOS com `OsTask` de período fixo | Timer ROS2 de 10 ms |
| Abstração de sensores | AUTOSAR Sensor Abstraction Layer | `aeb_controller_node` callbacks |
| Comunicação de atuação | CAN via MCAL | Tópico `/can/aeb_output` |
| Ambiente de validação | SIL (Simulink) → HIL (dSPACE) → veículo | SIL Python → Gazebo |
| Rastreabilidade de requisitos | DOORS + Polarion | IDs FR-xxx no código e nesta wiki |
| Gestão de falhas | DEM (Diagnostic Event Manager) AUTOSAR | 3-cycle fault latch em `aeb_perception` |

A principal diferença é que uma ECU real utilizaria um RTOS certificado (ex.: AUTOSAR OS, FreeRTOS certificado) e compilador certificado (ex.: Green Hills MULTI, IAR Embedded Workbench). Este projeto usa GCC/Clang com análise estática MISRA via PC-lint ou cppcheck como aproximação educacional.

---

*Próxima seção recomendada: [Módulos C Embarcado](Modulos-C-Embarcado.md)*
