# Barramento CAN — Documentação Técnica

> **Arquivo DBC:** `can/aeb_system.dbc`
> **Velocidade do barramento:** 500 kbps
> **Nós:** `AEB_Controller`, `Vehicle_ECU`, `Radar_ECU`

---

## Visão Geral

O barramento CAN do sistema AEB é definido formalmente no arquivo `can/aeb_system.dbc`. Na simulação ROS2, cada mensagem CAN é mapeada para um tópico dedicado — um por frame DBC — preservando a estrutura exata de sinalização que seria usada em produção. Nenhuma informação é transportada como `Float64` genérico: cada campo tem comprimento de bit, fator de escala e valor enumerado explícito, exatamente como exigido por ferramentas de análise CAN (Vector CANdb++, PEAK PCAN-Explorer, Kvaser Database Editor).

```
  Radar_ECU ─────► /can/radar_target  (AEB_RadarTarget  0x120, 20 ms)
  Vehicle_ECU ───► /can/ego_vehicle   (AEB_EgoVehicle   0x100, 10 ms)
  AEB_Controller ► /can/brake_cmd     (AEB_BrakeCmd     0x080, 10 ms, ASIL-B)
  AEB_Controller ► /can/fsm_state     (AEB_FSMState     0x200, 50 ms)
  AEB_Controller ► /can/alert         (AEB_Alert        0x300, evento)
```

---

## Detalhamento das Mensagens

### 1. AEB_RadarTarget — `0x120` | 20 ms | Radar_ECU → AEB_Controller

Contém os dados do alvo primário detectado pelo radar 77 GHz, após fusão com o lidar no `perception_node`. É a entrada principal do algoritmo de cálculo de TTC.

**DBC (extrato):**
```
BO_ 288 AEB_RadarTarget: 8 Radar_ECU
 SG_ TargetDistance : 0|16@1+  (0.01,0)      [0|655.35]   "m"    AEB_Controller
 SG_ RelativeSpeed  : 16|16@1+ (0.01,-327.68) [-327.68|327.67] "m/s" AEB_Controller
 SG_ TTC            : 32|16@1+ (0.001,0)     [0|65.535]   "s"    AEB_Controller
 SG_ Confidence     : 48|8@1+  (1,0)         [0|15]       ""     AEB_Controller
 SG_ Reserved       : 56|8@1+  (1,0)         [0|255]      ""     AEB_Controller
```

> **Nota sobre CAN ID:** O DBC usa o ID decimal `288` = `0x120`. O campo `can_id` da mensagem ROS2 carrega `0x120` explicitamente para rastreabilidade.

| Campo ROS2 | Sinal DBC | Bits | Fator/Offset | Unidade | Descrição |
|---|---|---|---|---|---|
| `can_id` | — | — | — | hex | Identificador do frame (`0x120`) |
| `alive_counter` | *(add-on ROS2)* | 4 | 1/0 | — | Contador rolante 0→15→0 |
| `target_distance` | `TargetDistance` | 16 | 0,01 / 0 | m | Distância fusionada ao alvo |
| `relative_speed` | `RelativeSpeed` | 16 | 0,01 / −327,68 | m/s | Velocidade relativa (positivo = fechando) |
| `ttc` | `TTC` | 16 | 0,001 / 0 | s | Time-to-Collision calculado |
| `confidence` | `Confidence` | 8 | 1/0 | — | Confiança: 15=ambos válidos, 8=um sensor, 0=falha |
| `sensor_fault` | *(add-on ROS2)* | 1 | — | bool | Flag FR-PER-007 |

**Período de publicação:** 20 ms (50 Hz), controlado pelo `timer_20ms` do `perception_node`.

---

### 2. AEB_EgoVehicle — `0x100` | 10 ms | Vehicle_ECU → AEB_Controller

Contém a dinâmica longitudinal do veículo ego. Publicado pelo `perception_node` a partir dos dados de odometria do Gazebo.

**DBC (extrato):**
```
BO_ 256 AEB_EgoVehicle: 8 Vehicle_ECU
 SG_ VehicleSpeed   : 0|16@1+  (0.01,0)       [0|655.35]     "m/s"   AEB_Controller
 SG_ LongAccel      : 16|16@1+ (0.001,-32)    [-32|33.535]   "m/s^2" AEB_Controller
 SG_ YawRate        : 32|16@1+ (0.01,-327.68) [-327.68|327.67] "deg/s" AEB_Controller
 SG_ SteeringAngle  : 48|16@1+ (0.1,-3276.8)  [-3276.8|3276.7] "deg" AEB_Controller
```

| Campo ROS2 | Sinal DBC | Unidade | Descrição |
|---|---|---|---|
| `can_id` | — | hex | `0x100` |
| `alive_counter` | — | — | Contador rolante 0→15→0 |
| `vehicle_speed` | `VehicleSpeed` | m/s | Velocidade longitudinal do ego |
| `long_accel` | `LongAccel` | m/s² | Aceleração longitudinal (negativo = frenagem) |
| `yaw_rate` | `YawRate` | deg/s | Taxa de guinada |
| `steering_angle` | `SteeringAngle` | deg | Ângulo do volante (positivo = esquerda) |
| `brake_pedal` | — | bool | Pedal de freio pressionado pelo motorista |

**Limitações da simulação:** `yaw_rate`, `steering_angle` e `brake_pedal` são sempre `0`/`False` porque o `planar_move` plugin não expõe esses estados via odometria (veja [Limitações Conhecidas](Simulacao-Gazebo.md#limitações-conhecidas)).

---

### 3. AEB_BrakeCmd — `0x080` | 10 ms | AEB_Controller → Vehicle_ECU | **ASIL-B**

Mensagem de segurança crítica. Carrega o comando de frenagem calculado pelo algoritmo C embarcado. Protegida por `AliveCounter` e CRC de 4 bits.

**DBC (extrato):**
```
BO_ 128 AEB_BrakeCmd: 4 AEB_Controller
 SG_ BrakeRequest  : 0|1@1+   (1,0)   [0|1]      ""    Vehicle_ECU
 SG_ BrakePressure : 1|15@1+  (0.1,0) [0|3276.7] "bar" Vehicle_ECU
 SG_ BrakeMode     : 16|3@1+  (1,0)   [0|7]      ""    Vehicle_ECU
 SG_ AliveCounter  : 24|4@1+  (1,0)   [0|15]     ""    Vehicle_ECU
 SG_ CRC           : 28|4@1+  (1,0)   [0|15]     ""    Vehicle_ECU
```

| Campo ROS2 | Sinal DBC | Unidade | Descrição |
|---|---|---|---|
| `can_id` | — | hex | `0x080` |
| `alive_counter` | `AliveCounter` | — | Contador rolante 0→15→0 |
| `crc` | `CRC` | — | CRC de 4 bits (ver seção abaixo) |
| `brake_request` | `BrakeRequest` | bool | `1` se qualquer frenagem ativa |
| `brake_pressure` | `BrakePressure` | bar | Pressão de frenagem solicitada |
| `brake_mode` | `BrakeMode` | enum | Modo de freio (ver tabela abaixo) |

**Enumeração BrakeMode (DBC `VAL_`):**

| Valor | Nome DBC | Estado FSM Correspondente |
|---|---|---|
| 0 | `Off` | OFF / STANDBY |
| 1 | `Warning` | WARNING |
| 2 | `Partial_L1` | BRAKE_L1 |
| 3 | `Moderate_L2` | BRAKE_L2 |
| 4 | `Full_L3` | BRAKE_L3 |
| 5 | `PostBrakeHold` | POST_BRAKE |

**Codificação de brake_pressure:**

```
brake_pressure [bar] = brake_pct [%] × 0,1

Exemplos:
  BRAKE_L1 (20%) → 2,0 bar
  BRAKE_L2 (40%) → 4,0 bar
  BRAKE_L3 (60%) → 6,0 bar
  100%           → 10,0 bar (máximo físico)
```

O campo `BrakePressure` no DBC tem resolução de 0,1 bar e range 0–3276,7 bar (15 bits). Na prática, o sistema utiliza no máximo 10,0 bar.

---

### 4. AEB_FSMState — `0x200` | 50 ms | AEB_Controller → Vehicle_ECU, Radar_ECU

Transmite o estado atual da máquina de estados para monitoramento e diagnóstico. Publicado pela thread de 50 ms do `aeb_controller_node`.

**DBC (extrato):**
```
BO_ 512 AEB_FSMState: 4 AEB_Controller
 SG_ FSMState    : 0|8@1+  (1,0) [0|6]    "" Vehicle_ECU,Radar_ECU
 SG_ AlertLevel  : 8|8@1+  (1,0) [0|3]    "" Vehicle_ECU,Radar_ECU
 SG_ BrakeActive : 16|8@1+ (1,0) [0|1]    "" Vehicle_ECU,Radar_ECU
 SG_ TTCThreshold: 24|8@1+ (0.1,0) [0|25.5] "s" Vehicle_ECU,Radar_ECU
```

**Enumeração FSMState (DBC `VAL_`):**

| Valor | Nome | Descrição |
|---|---|---|
| 0 | `OFF` | Sistema desligado ou falha de sensor |
| 1 | `STANDBY` | Aguardando ameaça (v_ego válida, sem alvo crítico) |
| 2 | `WARNING` | Alerta ao motorista, sem frenagem autônoma ainda |
| 3 | `BRAKE_L1` | Frenagem leve: −2 m/s², TTC ≤ 3,0 s |
| 4 | `BRAKE_L2` | Frenagem moderada: −4 m/s², TTC ≤ 2,2 s |
| 5 | `BRAKE_L3` | Frenagem total: −6 m/s², TTC ≤ 1,8 s |
| 6 | `POST_BRAKE` | Manutenção pós-parada por 2 s |

**Enumeração AlertLevel:**

| Valor | Nome | Estados FSM |
|---|---|---|
| 0 | `NONE` | OFF, STANDBY, POST_BRAKE |
| 1 | `WARNING` | WARNING |
| 2 | `BRAKE_LOW` | BRAKE_L1 |
| 3 | `BRAKE_HIGH` | BRAKE_L2, BRAKE_L3 |

O campo `ttc_threshold` informa o limiar TTC ativo para o estado atual: 4,0 / 3,0 / 2,2 / 1,8 s conforme `aeb_config.h`.

---

### 5. AEB_Alert — `0x300` | Evento | AEB_Controller → Vehicle_ECU

Publicado apenas em mudanças de estado FSM (event-driven). Comanda o HMI de alertas visuais e sonoros.

**DBC (extrato):**
```
BO_ 768 AEB_Alert: 2 AEB_Controller
 SG_ AlertType  : 0|8@1+ (1,0) [0|3] "" Vehicle_ECU
 SG_ AlertActive: 8|1@1+ (1,0) [0|1] "" Vehicle_ECU
 SG_ BuzzerCmd  : 9|3@1+ (1,0) [0|7] "" Vehicle_ECU
```

| Campo ROS2 | Sinal DBC | Descrição |
|---|---|---|
| `can_id` | — | `0x300` |
| `alert_type` | `AlertType` | 0=NONE, 1=VISUAL, 2=AUDIBLE, 3=BOTH |
| `alert_active` | `AlertActive` | `True` se visual ou sonoro ativo |
| `visual_active` | — | Ativo em WARNING, BRAKE_L1/L2/L3 (não em POST_BRAKE — FR-ALR-005 removido SRS v3) |
| `audible_active` | — | Ativo em WARNING, BRAKE_L1/L2/L3 (não em POST_BRAKE) |
| `buzzer_cmd` | `BuzzerCmd` | Padrão de buzzer (ver tabela abaixo) |

**Enumeração BuzzerCmd (DBC `VAL_`):**

| Valor | Nome | Estado FSM |
|---|---|---|
| 0 | `Off` | OFF / STANDBY / POST_BRAKE |
| 1 | `SingleBeep` | WARNING |
| 2 | `DoubleBeep` | BRAKE_L1 |
| 4 | `FastPulse` | BRAKE_L2 |
| 3 | `ContinuousBeep` | BRAKE_L3 |

---

## Mecanismo de Alive Counter

O `AliveCounter` é um campo de 4 bits presente em `AEB_RadarTarget` e `AEB_BrakeCmd`. Ele incrementa a cada mensagem transmitida, reiniciando de 15 para 0 (contagem rolante módulo 16):

```c
alive_counter = (alive_counter + 1U) & 0x0FU;
```

**Sequência esperada:** `0, 1, 2, ..., 14, 15, 0, 1, 2, ...`

O receptor detecta falha de comunicação quando dois valores consecutivos não diferem por exatamente 1 (módulo 16). Uma diferença de 2+ indica perda de frame; valor estático indica travamento do transmissor. No contexto do projeto, o `perception_node` mantém `alive_radar` e `alive_ego` separados para os dois frames que publica.

---

## CRC de 4 Bits (AEB_BrakeCmd)

O campo `CRC` de 4 bits na mensagem `AEB_BrakeCmd` (0x080) protege a integridade do conteúdo do comando de freio. É calculado por XOR simples sobre os três primeiros bytes de payload:

```c
/* aeb_controller_node.cpp, função compute_crc4() */
static uint8_t compute_crc4(uint8_t b0, uint8_t b1, uint8_t b2)
{
    uint8_t crc = (b0 ^ b1 ^ b2) & 0x0FU;
    return crc;
}
```

**Bytes usados no cálculo:**

| Byte | Conteúdo |
|---|---|
| `b0` | `alive_counter & 0x0F` — nibble inferior do contador |
| `b1` | `brake_request ? 1 : 0` — flag de ativação |
| `b2` | `brake_mode` — modo de freio (0–5) |

O receptor recalcula o CRC e rejeita o frame se o valor não conferir. Embora seja um XOR de 4 bits — não um CRC polinomial automotivo (CRC-8 SAE J1939) — é suficiente para detectar erros de bit único e serve como mecanismo educacional representativo de proteção de mensagem ASIL-B.

---

## Sincronização v_target_cached_

O `aeb_controller_node` mantém o campo `v_target_cached_` para resolver um problema de temporização assíncrona entre os dois frames de entrada:

- `AEB_EgoVehicle` chega a **10 ms** (Vehicle_ECU)
- `AEB_RadarTarget` chega a **20 ms** (Radar_ECU)

A velocidade relativa (`relative_speed`) e a velocidade do ego (`vehicle_speed`) devem ser consumidas **no mesmo instante de tempo** para calcular `v_target = v_ego - v_rel` de forma consistente. Se `v_target` fosse calculado no ciclo de controle de 10 ms usando o `relative_speed` mais recente (que tem até 20 ms de defasagem) e o `vehicle_speed` atual, o resultado estaria sujeito a erro se a velocidade do ego estiver mudando rapidamente — o que pode produzir falso positivo na checagem de plausibilidade de taxa de variação (ROC fault).

**Solução implementada:**

```cpp
// Callback do radar — sincronismo garantido:
// neste momento, ego_.vehicle_speed ainda é da mesma "janela" de dados
const float v_ego_at_radar = static_cast<float>(ego_.vehicle_speed);
const float vt = v_ego_at_radar - static_cast<float>(msg->relative_speed);
v_target_cached_ = (vt > 0.0F) ? vt : 0.0F;
```

`v_target_cached_` é calculado **dentro do callback do radar**, quando `ego_.vehicle_speed` e `msg->relative_speed` são contemporâneos. No ciclo de controle de 10 ms, apenas `v_target_cached_` é passado para `aeb_cycle_10ms()` — nunca o `relative_speed` diretamente.

---

## Mapeamento Tópicos ROS2 ↔ IDs CAN

| Tópico ROS2 | Mensagem ROS2 | CAN ID | DBC Name | Período | Emissor |
|---|---|---|---|---|---|
| `/can/radar_target` | `AebRadarTarget` | `0x120` | `AEB_RadarTarget` | 20 ms | `perception_node` |
| `/can/ego_vehicle` | `AebEgoVehicle` | `0x100` | `AEB_EgoVehicle` | 10 ms | `perception_node` |
| `/can/brake_cmd` | `AebBrakeCmd` | `0x080` | `AEB_BrakeCmd` | 10 ms | `aeb_controller_node` |
| `/can/fsm_state` | `AebFsmState` | `0x200` | `AEB_FSMState` | 50 ms | `aeb_controller_node` |
| `/can/alert` | `AebAlert` | `0x300` | `AEB_Alert` | Evento | `aeb_controller_node` |

**Rationale arquitetural:** A escolha de um tópico por frame CAN, em vez de tópicos genéricos `Float64`, serve a três objetivos:

1. **Rastreabilidade:** Qualquer engenheiro com o arquivo DBC pode mapear diretamente o tópico para a mensagem de barramento equivalente sem conversão.
2. **Conformidade ASIL-B:** `AEB_BrakeCmd` carrega `alive_counter` e `CRC` como campos nativos, permitindo que qualquer nó assinante implemente verificação de integridade sem lógica adicional.
3. **Realismo de produção:** Em um sistema embarcado real, o middleware de comunicação (AUTOSAR COM) popularia exatamente esses campos. O wrapper ROS2 é um substituto direto da camada de abstração de hardware (HAL CAN).

---

## Atributos DBC Adicionais

```
BA_DEF_ BO_ "GenMsgCycleTime" INT 0 10000;

BA_ "GenMsgCycleTime" BO_ 128   10;   /* AEB_BrakeCmd  */
BA_ "GenMsgCycleTime" BO_ 256   10;   /* AEB_EgoVehicle */
BA_ "GenMsgCycleTime" BO_ 288   20;   /* AEB_RadarTarget */
BA_ "GenMsgCycleTime" BO_ 512   50;   /* AEB_FSMState  */
BA_ "GenMsgCycleTime" BO_ 768    0;   /* AEB_Alert — event-driven */
```

O valor `0` em `AEB_Alert` indica que a mensagem não é cíclica, conforme a semântica padrão do atributo `GenMsgCycleTime` em ferramentas Vector.

---

## Exemplo de Sequência Completa de Frames

Cenário `ccrs_40` no instante de transição STANDBY → WARNING (TTC ≈ 4,0 s):

```
t=4.800s  /can/ego_vehicle   [speed=11.11 m/s, accel=0.0, alive=48→0]
t=4.800s  /can/radar_target  [dist=44.4m, rel_speed=11.1m/s, ttc=4.0s,
                               confidence=15, alive=24→8]
t=4.800s  /can/brake_cmd     [request=0, pressure=0.0bar, mode=0(Off),
                               alive=48, crc=0x0]
t=4.800s  /can/alert         [type=3(BOTH), visual=True, audible=True,
                               buzzer=1(SingleBeep)]   ← mudança de estado
t=4.850s  /can/fsm_state     [state=2(WARNING), alert=1, active=0,
                               ttc_threshold=4.0s]
```

---

*Última atualização: março de 2026 — Residência Stellantis/UFPE*
