# Simulação Gazebo — Documentação Técnica

> **Simulador:** Gazebo Classic 11
> **Middleware:** ROS2 Humble
> **Mundo:** `gazebo_sim/aeb_gazebo/worlds/aeb_highway.world`
> **Pacote ROS2:** `aeb_gazebo`

---

## Visão Geral da Simulação

A Camada 3 do projeto AEB é composta pelo ambiente de simulação Gazebo e pelos nós Python que orquestram os veículos, simulam sensores e fornecem monitoramento em tempo real. A arquitetura foi projetada para que o núcleo C embarcado (Camada 1) execute **sem nenhuma modificação** dentro do `aeb_controller_node` C++ (Camada 2), recebendo dados de sensores via tópicos CAN estruturados e publicando comandos que o `scenario_controller.py` aplica diretamente ao modelo físico do veículo ego.

```
┌─────────────────────────────────────────────────────────────┐
│  Gazebo Classic 11                                          │
│  aeb_highway.world                                          │
│  ┌────────────────┐      ┌──────────────────────┐          │
│  │  ego_vehicle   │      │   target_vehicle      │          │
│  │ (planar_move)  │      │   (planar_move)       │          │
│  └───────┬────────┘      └──────────┬────────────┘          │
│          │ /aeb/ego/odom            │ /aeb/target/odom      │
└──────────┼──────────────────────────┼────────────────────────┘
           │                          │
    ┌──────▼──────────────────────────▼──────┐
    │         perception_node.py             │
    │  Radar 77GHz σ=0,25m  +  Lidar σ=0,05m │
    │  Fusão: 0,3×radar + 0,7×lidar           │
    │  → /can/radar_target (20ms)             │
    │  → /can/ego_vehicle  (10ms)             │
    └──────────────────┬─────────────────────┘
                       │
    ┌──────────────────▼─────────────────────┐
    │       aeb_controller_node (C++)         │
    │  Biblioteca C embarcada (MISRA C:2012)  │
    │  → /can/brake_cmd  (10ms)              │
    │  → /can/fsm_state  (50ms)              │
    │  → /can/alert      (evento)            │
    └──────────────────┬─────────────────────┘
                       │
    ┌──────────────────▼─────────────────────┐
    │       scenario_controller.py            │
    │  Aplica velocidade ao ego e alvo        │
    │  Detecta colisão e fim de cenário       │
    └─────────────────────────────────────────┘
```

---

## Mundo Gazebo: aeb_highway.world

### Especificações Físicas

| Parâmetro | Valor |
|---|---|
| **Comprimento da pista** | 300 m (eixo X positivo) |
| **Largura da pista** | 7,4 m (2 faixas, nome: `highway`) |
| **Passo de simulação** | 1 ms (física ODE 1000 Hz) |
| **Fator de tempo real** | 1,0 |
| **Solver ODE** | Quick, 50 iterações |
| **Atrito (µ₁, µ₂)** | 0,8 |
| **Solo** | Plano 500×100 m |

### Posições de Spawn dos Veículos

| Modelo | Posição X inicial | Posição Y | Observação |
|---|---|---|---|
| `ego_vehicle` | 0,0 m | −1,85 m | Faixa direita |
| `target_vehicle` | 100,0 m | −1,85 m | 100 m à frente |

A posição de 100 m do `target_vehicle` no arquivo world é a posição padrão para todos os cenários CCRs e CCRm (`initial_gap_m = 100`). Para os cenários CCRb com gap de 12 m, o `scenario_controller` teleporta o veículo alvo via serviço `/aeb/set_entity_state` antes de iniciar — exceto quando `skip_teleport = True` (veja seção sobre skip_teleport).

### Plugin dos Veículos: planar_move

Ambos os veículos usam o plugin `libgazebo_ros_planar_move.so`, que permite controle direto de velocidade via tópico `cmd_vel` (`geometry_msgs/Twist`) sem modelagem de drivetrain. Isso simplifica a reprodução fiel de perfis de velocidade Euro NCAP, mas introduz limitações conhecidas (ver seção abaixo).

```xml
<!-- Fragmento do modelo do veículo -->
<plugin name="planar_move" filename="libgazebo_ros_planar_move.so">
  <ros>
    <namespace>/aeb/ego</namespace>
    <remapping>cmd_vel:=cmd_vel</remapping>
    <remapping>odom:=odom</remapping>
  </ros>
  <publish_rate>100.0</publish_rate>
</plugin>
```

A odometria é publicada a **100 Hz** em `/aeb/ego/odom` e `/aeb/target/odom`.

---

## perception_node.py — Simulação de Sensores e Fusão

### Modelo de Sensores

O `perception_node` simula dois sensores independentes a partir da **verdade fundamental** (`ground truth`) da odometria Gazebo e adiciona ruído gaussiano realista:

| Parâmetro | Radar 77 GHz | Lidar 905 nm |
|---|---|---|
| Desvio padrão de ruído (σ) | 0,25 m | 0,05 m |
| Frequência de atualização | 20 Hz (50 ms) | 15 Hz (67 ms) |
| Alcance mínimo | 0,5 m | 0,3 m |
| Alcance máximo | 260,0 m | 200,0 m |
| Modelo de ruído | `gauss(0, σ)` | `gauss(0, σ)` |

```python
# perception_node.py — simulação do radar
def _simulate_radar(self, now, gt_dist):
    if now - self.radar_time >= RADAR_PERIOD:   # 50 ms
        self.radar_time = now
        if RADAR_RANGE_MIN <= gt_dist <= RADAR_RANGE_MAX:
            self.radar_distance = gt_dist + random.gauss(0, RADAR_NOISE_STD)
        else:
            self.radar_distance = None
```

A distância verdadeira `gt_dist` é calculada como:
```
gt_dist = target_x - ego_x - EGO_LENGTH
```
onde `EGO_LENGTH = 4,5 m` compensa a distância entre as origens dos sistemas de coordenadas dos modelos (distância entre para-choque traseiro do alvo e para-choque dianteiro do ego).

### Algoritmo de Fusão Ponderada

A fusão é executada a 20 Hz (sincronizada com o ciclo do radar, que é o sensor mais lento):

```python
# Fusão normal: ambos os sensores válidos e concordantes
if radar_ok and lidar_ok:
    if abs(self.radar_distance - self.lidar_distance) > CROSS_SENSOR_TOL:
        # Conflito acima de 5m → usar lidar (maior precisão)
        raw_distance = self.lidar_distance
    else:
        # Ponderação: 30% radar + 70% lidar
        raw_distance = W_RADAR * self.radar_distance + W_LIDAR * self.lidar_distance
elif lidar_ok:
    raw_distance = self.lidar_distance   # degradação: só lidar
elif radar_ok:
    raw_distance = self.radar_distance   # degradação: só radar
```

| Parâmetro de Fusão | Valor | Justificativa |
|---|---|---|
| Peso do radar (`W_RADAR`) | 0,3 | Maior alcance, mas σ = 0,25 m |
| Peso do lidar (`W_LIDAR`) | 0,7 | Maior precisão, σ = 0,05 m |
| Limiar de conflito (`CROSS_SENSOR_TOL`) | 5,0 m | Discordância improvável por ruído; indica obstrução ou ghost |

### Checagem de Plausibilidade (FR-PER-006)

Após a fusão, um filtro de taxa de variação rejeita saltos físicamente impossíveis:

```python
rate = abs(raw_distance - self.prev_fused_distance) / dt
if rate > DIST_RATE_MAX:   # DIST_RATE_MAX = 60,0 m/s
    raw_distance = None    # leitura inválida — não atualiza fusão
```

### Detecção de Falha de Sensor (FR-PER-007)

```python
if raw_distance is not None:
    self.fault_counter = 0
    self.sensor_fault = False
else:
    self.fault_counter += 1
    if self.fault_counter >= FAULT_CYCLE_LIMIT:   # 3 ciclos = 60 ms a 20 Hz
        self.sensor_fault = True
        self.get_logger().warn('SENSOR FAULT (FR-PER-007)')
```

Com `sensor_fault = True`, o campo `sensor_fault` do frame `AEB_RadarTarget` é ativado; o módulo C `aeb_perception.c` lê este flag e força a transição FSM para `OFF`.

### Cálculo de TTC e Confiança

```python
# TTC
if self.v_rel > V_REL_MIN and self.fused_distance > 0:
    ttc = max(0.0, min(TTC_MAX, self.fused_distance / self.v_rel))

# Confiança
if radar_ok and lidar_ok:   confidence = 15
elif radar_ok or lidar_ok:  confidence = 8
else:                        confidence = 0
```

### Publicação dos Frames CAN

| Frame | Tópico | Período | Alive Counter |
|---|---|---|---|
| `AebRadarTarget` | `/can/radar_target` | 20 ms | `alive_radar` → 0x0F |
| `AebEgoVehicle` | `/can/ego_vehicle` | 10 ms | `alive_ego` → 0x0F |

O `alive_counter` em cada frame incrementa independentemente: `self.alive_radar = (self.alive_radar + 1) & 0x0F`.

---

## scenario_controller.py — Execução dos Cenários

O `scenario_controller` controla as velocidades de ambos os veículos, monitora fim de cenário e gerencia o posicionamento inicial do veículo alvo.

### Parâmetros ROS2

| Parâmetro | Tipo | Descrição |
|---|---|---|
| `scenario` | string | Nome do cenário (ex.: `ccrs_50`) |
| `ego_speed_kmh` | float | Velocidade inicial do ego [km/h] |
| `target_speed_kmh` | float | Velocidade inicial do alvo [km/h] |
| `initial_gap_m` | float | Gap inicial [m] |
| `target_decel` | float | Desaceleração do alvo [m/s²] (negativo) |
| `target_brake_time` | float | Instante de início da frenagem do alvo [s] |
| `skip_teleport` | bool | Pular chamada ao serviço de teleporte |

### Malha de Controle (100 Hz)

```python
# Aplicar frenagem AEB ao ego (0-100% → 0-10 m/s²)
brake_decel = (self.brake_cmd_pct / 100.0) * 10.0
if self.brake_cmd_pct > 1.0:
    self.ego_vx_cmd = max(0.0, self.ego_vx_cmd - brake_decel * dt)
```

O cenário lê `brake_cmd_pct` do frame `AEB_BrakeCmd` recebido no callback `brake_cmd_cb`. A conversão `brake_pressure [bar] → %` é: `brake_pct = brake_pressure / 0.1`.

### Log de Diagnóstico

A cada 1 segundo o `scenario_controller` emite uma linha de log formatada:

```
t=X.Xs  d=X.Xm  v_ego=X.Xkm/h  brake=X%
```

**Exemplo real (ccrs_40):**
```
[scenario_controller]: t=0.0s  d=100.0m  v_ego=0.0km/h  brake=0%
[scenario_controller]: t=1.0s  d=88.9m   v_ego=40.0km/h brake=0%
[scenario_controller]: t=3.0s  d=66.7m   v_ego=40.0km/h brake=0%
[scenario_controller]: t=6.0s  d=11.1m   v_ego=40.0km/h brake=0%
[scenario_controller]: t=6.5s  d=5.5m    v_ego=22.4km/h brake=60%
[scenario_controller]: t=7.1s  d=2.1m    v_ego=4.8km/h  brake=60%
[scenario_controller]: *** STOPPED: distance remaining = 2.1 m ***
```

### Condições de Fim de Cenário

| Condição | Resultado | Mensagem |
|---|---|---|
| `distance ≤ 1m` e `v_ego > 5 km/h` | COLISÃO | `COLLISION: v_impact=X.X km/h` |
| `distance ≤ 1m` e `v_ego ≤ 5 km/h` | PARADA | `STOPPED: gap=X.Xm v=X.Xkm/h` |
| `ego_vx_cmd < 0,15 m/s` e `elapsed > 2s` | PARADA | `STOPPED: gap=X.Xm` |
| `elapsed > 60s` | TIMEOUT | `TIMEOUT` |

---

## dashboard_node.py — Dashboard em Tempo Real

### Layout

O dashboard é construído em `matplotlib` com tema escuro e atualizado a **10 Hz** (100 ms por frame):

```
┌─────────────────────┬──────────────────┬──────────────────┐
│                     │  TTC BAR         │  VISUAL ALERT    │
│   VELOCÍMETRO       │                  │  AUDIBLE ALERT   │
│   (arco 0-80 km/h)  ├──────────────────┤                  │
│                     │  FSM STATE       ├──────────────────┤
│                     │  (painel colorido)│  BRAKE BAR       │
│                     ├──────────────────│  (0-100%)        │
│                     │  DISTANCE PANEL  │                  │
└─────────────────────┴──────────────────┴──────────────────┘
```

### Componentes Visuais

| Widget | Função | Atualização |
|---|---|---|
| **Velocímetro** | Arco semicircular 0–80 km/h; ponteiro vermelho; cor verde/laranja/vermelho por zona | `_ego_cb` (10 ms) |
| **Barra TTC** | Barra horizontal 0–6 s; cor verde > 4s, laranja 3–4s, vermelho < 3s | `_radar_cb` (20 ms) |
| **Painel FSM** | Caixa colorida com nome do estado; 7 cores distintas | `_fsm_cb` (50 ms) |
| **Painel Distância** | Distância ao alvo em m + velocidade do alvo em km/h + status final | `_radar_cb` (20 ms) |
| **Alertas** | Dois botões: VISUAL (amarelo quando ativo) e BUZZER (laranja quando ativo) | `_alert_cb` (evento) |
| **Barra de Freio** | Barra vertical 0–100% com cor laranja/vermelho | `_brake_cb` (10 ms) |

**Cores do painel FSM:**

| Estado | Cor Hex |
|---|---|
| `OFF` | `#666666` (cinza) |
| `STANDBY` | `#4CAF50` (verde) |
| `WARNING` | `#FF9800` (laranja) |
| `BRAKE_L1` | `#FF5722` (laranja-avermelhado) |
| `BRAKE_L2` | `#F44336` (vermelho) |
| `BRAKE_L3` | `#D50000` (vermelho escuro) |
| `POST_BRAKE` | `#2196F3` (azul) |

### Thread de Renderização

O dashboard usa duas threads: a thread principal de `rclpy.spin` para receber mensagens ROS2 (protegida por `threading.Lock`) e a thread principal do Python para o loop `matplotlib` a 10 Hz:

```python
spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
spin_thread.start()
# Loop matplotlib na thread principal
while rclpy.ok() and plt.fignum_exists(fig.number):
    d = node.snap()   # cópia atômica protegida por lock
    draw_speed(ax_spd, d['speed'])
    ...
    time.sleep(0.1)   # 10 Hz
```

---

## Limitações Conhecidas da Simulação

| Limitação | Causa | Impacto |
|---|---|---|
| `steering_angle` sempre `0` | `planar_move` não expõe ângulo de direção | Override de direção inativo (FR-DRV-002 não validado) |
| `brake_pedal` sempre `False` | Sem modelo de motorista | Override por pedal inativo |
| `actual_decel = 0` (open-loop) | Derivada de velocidade da odometria é muito ruidosa | PID opera em malha aberta; KP=10 compensa |
| `long_accel` ruidoso | Diferenciação numérica de velocidade sobre odom | Apenas informativo no dashboard; não usado pelo C core |
| Frequências desacopladas | Lidar a 15 Hz, radar a 20 Hz, ego a 10 Hz | Tratado por `v_target_cached_` (ver [Barramento CAN](Barramento-CAN.md)) |

---

## skip_teleport: Por que Cenários de 100 m Pulam o Teleporte

O `aeb_highway.world` posiciona o `target_vehicle` em `x = 100,0 m` por padrão — exatamente o gap dos cenários CCRs (`ccrs_20/30/40/50`) e CCRm. Se o `scenario_controller` chamasse o serviço `/aeb/set_entity_state` para mover o veículo **para a mesma posição em que ele já está**, o mecanismo de atribuição de posição do ODE introduziria um **impulso físico** no modelo, fazendo o veículo alvo derivar ~3 m para frente nos primeiros 3 s de simulação.

Esse drift atrasava o cruzamento do limiar de WARNING (TTC ≤ 4,0 s), pois a distância percebida era artificialmente maior do que o gap configurado.

**Detecção automática no launch file:**

```python
# aeb_scenario.launch.py
'skip_teleport': (abs(params['gap'] - 100.0) < 0.1),
```

Se `|gap - 100| < 0,1 m`, `skip_teleport = True` é passado ao nó. O `scenario_controller` então **não chama** `_teleport_target()` e inicia o cenário 2 s após receber o primeiro odometry:

```python
if self.skip_teleport:
    self.get_logger().info(
        f'Skipping teleport — target spawns at gap={self.initial_gap:.0f} m'
    )
    self.target_teleported = True
    self.create_timer(2.0, self._start_scenario_once)
```

Para os cenários CCRb com gap de 12 m, o teleporte **é necessário** (posição diferente de 100 m) e o serviço é chamado normalmente.

---

## Dicas de WSL2 e Gazebo

### Erros ALSA (Audio)

```
ALSA lib pcm.c:2666:(snd_pcm_open_noupdate) Unknown PCM cards.pcm.front
```

Esses erros são **normais** no WSL2 (sem dispositivo de áudio). Não afetam a simulação. O sistema de alertas sonoros do AEB (`AEB_Alert.buzzer_cmd`) é mapeado para o sistema de áudio do SO hospedeiro; em WSL2 fica sem efeito.

### Script run.sh

O script `gazebo_sim/aeb_gazebo/run.sh` executa o launch file correto e configura o ambiente:

```bash
#!/bin/bash
# Uso: ./run.sh <scenario_name>
source ~/aeb_ws/install/setup.bash
ros2 launch aeb_gazebo aeb_scenario.launch.py scenario:=$1
```

**Exemplos:**
```bash
./run.sh ccrs_50          # CCRs a 50 km/h
./run.sh ccrm             # CCRm (alvo a 20 km/h)
./run.sh ccrb_d6_g12      # CCRb desaceleração 6 m/s², gap 12 m
```

### Sequência de Inicialização Típica

```bash
# Terminal 1 — compilar (apenas na primeira vez ou após mudanças)
cd ~/aeb_ws
colcon build --packages-select aeb_gazebo --symlink-install
source install/setup.bash

# Terminal 2 — executar cenário
ros2 launch aeb_gazebo aeb_scenario.launch.py scenario:=ccrs_40

# Terminal 3 (opcional) — dashboard em tempo real
source ~/aeb_ws/install/setup.bash
ros2 run aeb_gazebo dashboard_node.py

# Terminal 4 (opcional) — inspeção de tópicos CAN
ros2 topic echo /can/fsm_state
ros2 topic echo /can/brake_cmd
```

### Verificação de Saúde dos Tópicos

```bash
# Confirmar frequências esperadas
ros2 topic hz /can/radar_target   # ~50 Hz
ros2 topic hz /can/ego_vehicle    # ~100 Hz
ros2 topic hz /can/brake_cmd      # ~100 Hz
ros2 topic hz /can/fsm_state      # ~20 Hz
```

---

*Última atualização: março de 2026 — Residência Stellantis/UFPE*
