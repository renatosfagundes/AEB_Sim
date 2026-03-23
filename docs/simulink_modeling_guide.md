# Modelagem do Sistema AEB em Simulink — Guia de Implementação

Este documento descreve os 5 blocos de modelagem do sistema AEB (Autonomous Emergency Braking) em Simulink. Cada bloco deve ser implementado como um subsistema independente (ou modelo referenciado), com entradas e saídas bem definidas, permitindo simulação isolada e integrada.

---

## 1. Planta (Plant Model)

### Descrição

A planta representa o ambiente físico: o veículo ego, o veículo-alvo e a dinâmica de movimento entre eles. Este modelo recebe comandos de atuação (desaceleração de frenagem) e produz os estados cinemáticos que alimentam os sensores.

A modelagem inclui:
- **Dinâmica longitudinal do veículo ego**: massa, resistência ao rolamento, arrasto aerodinâmico e resposta ao comando de freio.
- **Dinâmica do atuador de freio**: tempo morto hidráulico (dead time ~30 ms), constante de tempo de pressurização (~50 ms), saturação de desaceleração máxima (10 m/s²).
- **Veículo-alvo**: perfil de velocidade programável (constante, desacelerando ou parado) para reproduzir os cenários Euro NCAP (CCRs, CCRm, CCRb).
- **Cinemática relativa**: distância inter-veicular, velocidade relativa e aceleração relativa.

### Entradas

| Sinal | Unidade | Origem |
|-------|---------|--------|
| `brake_pressure_cmd` | bar (0–10) | Controller (via CAN) |
| `ego_throttle` | % (0–100) | Cenário / Driver model |
| `target_speed_profile` | km/h | Configuração do cenário |
| `target_decel_trigger` | s | Configuração do cenário |

### Saídas

| Sinal | Unidade | Destino |
|-------|---------|---------|
| `ego_speed` | m/s | Perception, CAN |
| `ego_accel` | m/s² | CAN |
| `target_speed` | m/s | Cálculo interno |
| `distance` | m | Perception |
| `relative_speed` | m/s | Perception |

### Dicas de implementação

- Use blocos **Integrator** para converter aceleração → velocidade → posição.
- Modele o atuador de freio como um **Transfer Function** de 1ª ordem com **Transport Delay** (dead time): `G(s) = 1 / (τs + 1)` com `τ = 0.05s` e delay de `0.03s`.
- Adicione um bloco **Saturation** para limitar a desaceleração entre 0 e 10 m/s² e a velocidade mínima em 0 (o carro não anda para trás).
- Para os cenários Euro NCAP, use um bloco **Signal Builder** ou **From Workspace** para definir o perfil de velocidade do alvo.
- A resistência aerodinâmica pode ser simplificada como `F_drag = 0.5 * Cd * A * rho * v²` com valores típicos (Cd=0.3, A=2.2 m², rho=1.225 kg/m³).
- Gere um **arquivo Simulink (.slx)** com este subsistema. Simule isoladamente com uma entrada degrau de frenagem (ex: 5 bar) e verifique se o veículo desacelera corretamente e para.

---

## 2. Percepção (Sensor / Perception Model)

### Descrição

O bloco de percepção modela a cadeia de sensoriamento: radar e lidar. Ele recebe os estados reais da planta (distância, velocidades) e produz medições com ruído, latência e possíveis falhas — simulando o que o controlador realmente "enxerga" no mundo real.

A modelagem inclui:
- **Modelo do radar**: alcance (0.5–200 m), resolução, ruído gaussiano na medição de distância (σ ≈ 0.3 m) e velocidade relativa (σ ≈ 0.1 m/s).
- **Modelo do lidar**: alcance, ruído e taxa de atualização (tipicamente 10–20 Hz).
- **Fusão sensorial**: combinação ponderada (ou filtro de Kalman simplificado) das leituras de radar e lidar.
- **Detecção de falha**: simulação de perda de sinal, falha intermitente e degradação de confiança.
- **Latência do sensor**: atraso de processamento entre a medição real e a disponibilização do dado.

### Entradas

| Sinal | Unidade | Origem |
|-------|---------|--------|
| `true_distance` | m | Plant |
| `true_rel_speed` | m/s | Plant |
| `true_ego_speed` | m/s | Plant |
| `fault_injection` | bool | Cenário de teste |

### Saídas

| Sinal | Unidade | Destino |
|-------|---------|---------|
| `measured_distance` | m | CAN TX (0x120) |
| `measured_rel_speed` | m/s | CAN TX (0x120) |
| `measured_ego_speed` | m/s | CAN TX (0x100) |
| `sensor_confidence` | 0–100% | CAN TX (0x120) |
| `sensor_fault` | bool | CAN TX, UDS (DTC) |

### Dicas de implementação

- Use o bloco **Band-Limited White Noise** para adicionar ruído gaussiano às medições. Ajuste a variância conforme o sensor (radar é mais ruidoso em distância curta, lidar em distância longa).
- Modele a latência com um bloco **Transport Delay** (ex: 20 ms para radar, 50 ms para lidar).
- Para fusão, use um bloco **MATLAB Function** com média ponderada: `d_fused = (w_r * d_radar + w_l * d_lidar) / (w_r + w_l)`, onde os pesos dependem da confiança de cada sensor.
- Para injeção de falha, use um **Switch** controlado por `fault_injection`: quando ativo, substitui a saída por um valor fixo (ex: 300 m — "sem alvo") ou congela o último valor válido.
- Adicione um bloco **Rate Transition** ou **Zero-Order Hold** para simular a taxa de amostragem discreta dos sensores (radar a 20 ms, lidar a 50 ms).
- Gere um **arquivo Simulink (.slx)** com este subsistema. Simule com uma rampa de distância decrescente (200 m → 0 m) e verifique que o ruído, a latência e a injeção de falha funcionam corretamente nos scopes.

---

## 3. Comunicação CAN (CAN Bus Model)

### Descrição

O bloco de comunicação modela o barramento CAN que interliga os módulos do sistema. No veículo real, o controlador AEB não acessa os sensores diretamente — tudo passa pelo CAN bus com empacotamento, temporização e possíveis perdas.

A modelagem inclui:
- **Empacotamento de mensagens (Pack/Unpack)**: conversão entre sinais físicos (float) e frames CAN com fator, offset e tamanho de bits, conforme o arquivo `.dbc`.
- **Temporização cíclica**: cada mensagem tem um período definido (AEB_EgoVehicle a cada 10 ms, AEB_RadarTarget a cada 20 ms, AEB_BrakeCmd a cada 10 ms, AEB_FSMState a cada 50 ms).
- **CRC e Alive Counter**: checksum XOR de 4 bits e contador de vida para detecção de falha de comunicação.
- **Simulação de falhas**: perda de mensagem (message dropout), atraso variável (jitter), e erro de CRC.

### Mensagens CAN do sistema

| Mensagem | CAN ID | Período | Direção | Conteúdo |
|----------|--------|---------|---------|----------|
| AEB_EgoVehicle | 0x100 | 10 ms | Sensor → Controller | Velocidade, aceleração, ângulo de direção |
| AEB_RadarTarget | 0x120 | 20 ms | Sensor → Controller | Distância, vel. relativa, TTC, confiança |
| AEB_BrakeCmd | 0x080 | 10 ms | Controller → Actuator | Pressão de freio, estado ativo, CRC |
| AEB_FSMState | 0x200 | 50 ms | Controller → Dashboard | Estado FSM, limiares de TTC |
| AEB_Alert | 0x300 | Evento | Controller → Dashboard | Alertas visuais/sonoros |

### Entradas

| Sinal | Unidade | Origem |
|-------|---------|--------|
| Sinais físicos dos sensores | Variados | Perception |
| Sinais de comando do controlador | Variados | Controller |
| `can_fault_injection` | bool | Cenário de teste |

### Saídas

| Sinal | Unidade | Destino |
|-------|---------|---------|
| Frames CAN empacotados | bytes | Controller (RX) / Actuator (RX) |
| `crc_error` | bool | UDS (DTC) |
| `msg_timeout` | bool | UDS (DTC), Controller |
| `alive_counter_error` | bool | UDS (DTC) |

### Dicas de implementação

- Use o **Vehicle Network Toolbox** do Simulink se disponível — ele tem blocos nativos de **CAN Pack / CAN Unpack** que importam diretamente o arquivo `.dbc` (`aeb_system.dbc`).
- Se não tiver o toolbox, modele o empacotamento manualmente com blocos **MATLAB Function**: aplique fator, offset, e extraia bits conforme a definição do `.dbc`.
- Para a temporização, use blocos **Triggered Subsystem** ou **Rate Transition** com sample times diferentes (0.01s, 0.02s, 0.05s) para cada mensagem.
- Para simular perda de mensagem, use um **Stochastic Switch** (bloco **Uniform Random Number** + comparador) que zera o frame com probabilidade configurável (ex: 1% de dropout).
- O CRC pode ser modelado como um bloco **MATLAB Function** que calcula XOR dos bytes do payload e compara com o campo CRC recebido.
- O alive counter é um contador módulo 16 (4 bits) que incrementa a cada transmissão — modele com um **Counter Limited** e compare TX vs RX.
- Gere um **arquivo Simulink (.slx)** com este subsistema. Simule o envio de uma sequência de mensagens AEB_BrakeCmd e verifique no scope que o empacotamento/desempacotamento preserva os valores, que o CRC detecta corrupção injetada, e que o timeout é sinalizado quando uma mensagem é suprimida.

---

## 4. Controlador AEB (Controller Model)

### Descrição

O controlador é o núcleo do sistema AEB. Ele recebe dados de percepção (via CAN), calcula o risco de colisão e comanda a frenagem de emergência. Este bloco deve ser uma representação fiel do código C embarcado (`c_embedded/src/`), permitindo validação Model-in-the-Loop (MIL).

A modelagem inclui:
- **Cálculo de TTC**: `TTC = distância / velocidade_relativa` quando o veículo está se aproximando (v_rel > 0). Inclui cálculo de distância de frenagem.
- **Máquina de estados finita (FSM)**: 7 estados — OFF, STANDBY, WARNING, BRAKE_L1, BRAKE_L2, BRAKE_L3, POST_BRAKE. Transições baseadas em TTC, distância mínima de frenagem, override do motorista e histerese temporal (200 ms).
- **Controlador PID de frenagem**: PI com anti-windup e limitação de jerk (100 m/s³). Converte desaceleração-alvo em pressão de freio (0–10 bar).
- **Lógica de alertas**: mapeamento estado → alerta visual (LED) + alerta sonoro (buzzer).
- **Override do motorista**: detecção de intervenção por volante (>5°) ou pedal de freio durante frenagem; acelerador apenas em POST_BRAKE.

### Entradas (via CAN RX)

| Sinal | Unidade | Origem |
|-------|---------|--------|
| `distance` | m | CAN (0x120) |
| `rel_speed` | m/s | CAN (0x120) |
| `ego_speed` | m/s | CAN (0x100) |
| `ego_accel` | m/s² | CAN (0x100) |
| `steering_angle` | graus | CAN (0x100) |
| `brake_pedal` | bool | CAN (0x100) |
| `aeb_enabled` | bool | UDS / InfoCenter |

### Saídas (via CAN TX)

| Sinal | Unidade | Destino |
|-------|---------|---------|
| `brake_pressure` | bar (0–10) | CAN (0x080) → Plant |
| `fsm_state` | enum (0–6) | CAN (0x200) → Dashboard, UDS |
| `target_decel` | m/s² | CAN (0x200) |
| `alert_visual` | 0–3 | CAN (0x300) → Dashboard |
| `alert_audible` | 0–2 | CAN (0x300) → Dashboard |

### Dicas de implementação

- Use o **Stateflow** do Simulink para modelar a FSM — é a ferramenta ideal para máquinas de estados. Defina os 7 estados, as condições de transição (TTC, distância, flags) e as ações de entrada/saída de cada estado.
- Se não tiver Stateflow, use um bloco **MATLAB Function** com uma variável persistente de estado e um `switch-case`.
- O cálculo de TTC é simples: use um bloco **Divide** com proteção contra divisão por zero (`max(v_rel, 0.001)`). Adicione um **Saturation** para limitar TTC entre 0 e 10 s.
- Para o PID, use o bloco **PID Controller** do Simulink (modo PI, anti-windup por clamping, sample time de 0.01s) ou implemente manualmente com **Discrete-Time Integrator** + **Gain**.
- A limitação de jerk pode ser modelada com um **Rate Limiter** (rising/falling slew rate = ±100 m/s³ × dt).
- Os limiares de TTC e desaceleração-alvo devem ser **parâmetros do workspace** (não hardcoded), permitindo calibração via UDS.
- Modele o ciclo de 10 ms usando **sample time discreto** (0.01s) em todos os blocos do controlador.
- Gere um **arquivo Simulink (.slx)** com este subsistema. Simule com um perfil de TTC decrescente (10s → 0s em rampa) e verifique nos scopes: (a) as transições da FSM nos limiares corretos, (b) a pressão de freio crescendo suavemente com limitação de jerk, (c) os alertas ativando no estado correto.

---

## 5. Diagnóstico UDS (Unified Diagnostic Services)

### Descrição

O bloco de diagnóstico modela a interface de serviços UDS (ISO 14229) do sistema AEB. No veículo real, uma ferramenta de diagnóstico (tester) ou o InfoCenter se comunica com a ECU via CAN para ler parâmetros, gravar calibrações, monitorar falhas e habilitar/desabilitar funções.

A modelagem inclui:
- **Leitura de dados (ReadDataByIdentifier — SID 0x22)**: permite ao tester ler em tempo real o TTC, estado da FSM, pressão de freio, contadores de falha.
- **Escrita de dados (WriteDataByIdentifier — SID 0x2E)**: permite calibrar limiares de TTC, ganhos do PID e limites de desaceleração em tempo de execução.
- **Leitura de DTCs (ReadDTCInformation — SID 0x19)**: lista os códigos de falha armazenados (sensor timeout, erro de CRC, perda de CAN, falha de plausibilidade).
- **Limpeza de DTCs (ClearDiagnosticInformation — SID 0x14)**: reseta a memória de falhas após manutenção.
- **Controle de rotina (RoutineControl — SID 0x31)**: habilitar/desabilitar o AEB (simula a chave do InfoCenter), disparar autoteste.
- **Armazenamento de falhas (Fault Memory)**: memória não-volátil simulada que persiste DTCs entre ciclos de ignição.

### Serviços e DIDs (Data Identifiers)

| Serviço | SID | DID / Uso | Descrição |
|---------|-----|-----------|-----------|
| Read | 0x22 | 0xF100 | TTC atual (float, s) |
| Read | 0x22 | 0xF101 | Estado FSM (uint8, 0–6) |
| Read | 0x22 | 0xF102 | Pressão de freio atual (float, bar) |
| Read | 0x22 | 0xF103 | Contador de falhas de sensor |
| Write | 0x2E | 0xF200 | Limiar TTC Warning (float, s) |
| Write | 0x2E | 0xF201 | Limiar TTC L1 (float, s) |
| Write | 0x2E | 0xF202 | Ganho Kp do PID (float) |
| DTC Read | 0x19 | Sub 0x02 | Listar DTCs por status mask |
| DTC Clear | 0x14 | Group ALL | Limpar todos os DTCs |
| Routine | 0x31 | 0x0301 | Habilitar/desabilitar AEB |
| Routine | 0x31 | 0x0302 | Autoteste de sensores |
| Tester Present | 0x3E | — | Manter sessão de diagnóstico |

### DTCs (Diagnostic Trouble Codes) do sistema

| DTC | Descrição | Condição de Ativação |
|-----|-----------|----------------------|
| C1001 | Timeout de radar | Sem mensagem 0x120 por > 100 ms |
| C1002 | Timeout de lidar | Sem dado de lidar por > 150 ms |
| C1003 | Falha de plausibilidade | |radar - lidar| > 5 m por > 3 ciclos |
| C1004 | Erro de CRC no CAN | CRC recebido ≠ CRC calculado |
| C1005 | Alive counter travado | Contador não incrementa por > 5 ciclos |
| C1006 | Falha de atuador | Pressão real ≠ comandada por > 500 ms |
| C1007 | Sobre-temperatura ECU | Temperatura > 85°C |

### Entradas

| Sinal | Unidade | Origem |
|-------|---------|--------|
| `diag_request` | bytes (SID + dados) | Tester externo (simulado) |
| `fsm_state` | enum | Controller |
| `ttc` | s | Controller |
| `brake_pressure` | bar | Controller |
| `sensor_fault` | bool | Perception |
| `crc_error` | bool | CAN Bus |
| `msg_timeout` | bool | CAN Bus |

### Saídas

| Sinal | Unidade | Destino |
|-------|---------|---------|
| `diag_response` | bytes (SID + dados) | Tester externo |
| `aeb_enabled` | bool | Controller |
| `calibrated_ttc_thresholds` | float[3] | Controller |
| `calibrated_pid_gains` | float[2] | Controller |
| `dtc_count` | uint8 | Dashboard / Tester |
| `fault_lamp` | bool | Dashboard (MIL) |

### Dicas de implementação

- Modele o UDS como um bloco **MATLAB Function** principal com `persistent` variables para armazenar: (a) os DTCs ativos, (b) os parâmetros calibráveis, (c) o estado de sessão.
- A entrada `diag_request` pode ser simulada com um bloco **From Workspace** que envia sequências de requisições UDS em instantes programados (ex: leitura de TTC a cada 1s, escrita de limiar em t=5s, clear DTC em t=30s).
- Para a memória de falhas, use variáveis `persistent` dentro do MATLAB Function — elas mantêm o valor entre passos de simulação (simula a NVM/EEPROM).
- Para os DTCs, implemente uma lógica de **debounce**: a falha só é confirmada após N ciclos consecutivos (ex: 3 ciclos para sensor fault, 5 para alive counter). Use um contador por DTC.
- A lógica de habilitação/desabilitação do AEB (RoutineControl 0x0301) deve ser um sinal booleano que vai diretamente para o controlador — quando `aeb_enabled = false`, a FSM permanece em OFF.
- Modele o fluxo request/response com sample time maior (ex: 100 ms) para representar a comunicação diagnóstica real (mais lenta que o controle).
- Adicione um **scope** para visualizar a tabela de DTCs ao longo do tempo (fault set → confirmed → cleared).
- Gere um **arquivo Simulink (.slx)** com este subsistema. Simule um cenário onde: (a) o sistema opera normalmente por 5s, (b) uma falha de sensor é injetada em t=5s, (c) o DTC é registrado e confirmado, (d) a falha é removida em t=10s, (e) um tester envia ClearDTC em t=15s. Verifique nos scopes que o DTC segue o ciclo completo: inactive → pending → confirmed → cleared.

---

## Integração dos 5 Blocos

Após implementar cada subsistema individualmente, crie um **modelo top-level** (`aeb_system.slx`) que conecte todos:

```
┌──────────┐     ┌────────────┐     ┌─────────┐     ┌──────────────┐
│          │────▶│            │────▶│         │────▶│              │
│  Plant   │     │ Perception │     │ CAN Bus │     │  Controller  │
│          │◀────│            │◀────│         │◀────│              │
└──────────┘     └────────────┘     └─────────┘     └──────────────┘
                                        ▲ ▼
                                  ┌─────────────┐
                                  │     UDS     │
                                  │ Diagnostics │
                                  └─────────────┘
```

- Use **Model Reference** (`Simulink.ModelReference`) para manter cada bloco em seu próprio arquivo `.slx`.
- Configure os **sample times**: Plant e Perception em tempo contínuo (ou 1 ms), CAN e Controller em 10 ms, UDS em 100 ms.
- Use **Bus Objects** para organizar os sinais entre blocos (evita dezenas de linhas soltas).
- Execute o cenário **CCRs 50 km/h** completo e verifique a resposta do sistema nos scopes: TTC → Warning → Brake L1 → L2 → L3 → parada (ou colisão sem AEB).