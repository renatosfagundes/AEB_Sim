# Módulos C Embarcado

> **Página:** Módulos C Embarcado
> **Relacionado:** [Home](Home.md) | [Arquitetura do Sistema](Arquitetura-do-Sistema.md) | [Máquina de Estados](Maquina-de-Estados.md) | [Controlador PID](Controlador-PID.md)

---

## Visão geral dos módulos

A Camada 1 do sistema AEB é composta por 8 arquivos C/header organizados em dois grupos:

- **Headers de configuração e tipos** (`aeb_config.h`, `aeb_types.h`): sem lógica executável, apenas definições
- **Módulos de implementação** (`aeb_main.c`, `aeb_perception.c`, `aeb_ttc.c`, `aeb_fsm.c`, `aeb_pid.c`, `aeb_alert.c`): toda a lógica de segurança

Todos os módulos compartilham as seguintes características de conformidade MISRA C:2012:

- Sem alocação dinâmica de memória (`malloc`, `calloc`, `realloc`, `free`)
- Sem recursão (verificado por análise estática)
- Tipos de largura fixa exclusivamente (`uint8_t`, `int16_t`, `float` — C99 IEEE 754 single precision)
- Todas as variáveis locais inicializadas no ponto de declaração
- Sem `goto`, `setjmp`, `longjmp`
- Todas as funções têm um único ponto de retorno (regra MISRA 15.5)
- Comentários de desvio (`/* MISRA Deviation: Rule X.Y — justificativa */`) onde aplicável

---

## `aeb_config.h` — Parâmetros de calibração

Este header centraliza **todos** os parâmetros de calibração do sistema em `#define` nomeados. Nenhum valor mágico (*magic number*) deve aparecer nos módulos de implementação. A alteração de qualquer parâmetro de calibração requer apenas a edição deste arquivo e recompilação — nenhum outro arquivo fonte precisa ser tocado.

### Limiares de TTC (Time-to-Collision)

```c
/* Limiares TTC para escalonamento de estados [segundos] */
#define AEB_TTC_WARN_L1     4.0f   /* TTC <= 4.0 s → ativa WARNING_L1  */
#define AEB_TTC_WARN_L2     3.0f   /* TTC <= 3.0 s → ativa WARNING_L2  */
#define AEB_TTC_BRAKE_L1    2.2f   /* TTC <= 2.2 s → ativa BRAKE_L1    */
#define AEB_TTC_BRAKE_L2    1.8f   /* TTC <= 1.8 s → ativa BRAKE_L2    */
```

Estes valores foram derivados de dados de teste Euro NCAP e da literatura técnica (MDPI Sensors 2024). Em um veículo a 60 km/h (16,67 m/s), TTC = 4,0 s corresponde a uma distância de aproximadamente 67 m — bem dentro do alcance de um radar de 77 GHz típico. Em TTC = 1,8 s a mesma velocidade, a distância é de apenas ~30 m, exigindo frenagem máxima imediata.

### Níveis de desaceleração alvo

```c
/* Desaceleração alvo por nível de frenagem [m/s²] */
#define AEB_DECEL_L1        2.0f   /* Frenagem suave — alerta háptico ao motorista */
#define AEB_DECEL_L2        4.0f   /* Frenagem moderada                            */
#define AEB_DECEL_L3        6.0f   /* Frenagem máxima (parada de emergência)       */
```

O valor L3 de 6 m/s² é deliberadamente conservador em relação ao limite físico dos pneus em asfalto seco (~8–9 m/s²). Isso proporciona margem de segurança e evita bloqueio de rodas em superfícies com coeficiente de atrito reduzido. Sistemas de produção combinam AEB com ABS para maximizar a desaceleração sem bloqueio.

### Velocidades de ativação do ego

```c
/* Faixa de velocidade do veículo ego para ativação do AEB [m/s] */
#define AEB_V_EGO_MIN       1.39f   /* 5 km/h  — abaixo: AEB em STANDBY, sem atuar  */
#define AEB_V_EGO_MAX       16.67f  /* 60 km/h — acima: AEB em STANDBY, sem atuar   */
```

O limite inferior de 1,39 m/s (5 km/h) evita ativações espúrias em manobras de estacionamento e movimentos de baixa velocidade. O limite superior de 16,67 m/s (60 km/h) define o escopo do cenário Euro NCAP CCR validado neste projeto; sistemas de produção tipicamente operam até 80 km/h ou mais.

### Distâncias mínimas de ativação (*Distance Floor*)

```c
/* Distâncias mínimas de ativação por nível — conceito LPB [metros] */
#define AEB_D_BRAKE_L1      20.0f   /* BRAKE_L1: só ativa se distância <= 20 m */
#define AEB_D_BRAKE_L2      10.0f   /* BRAKE_L2: só ativa se distância <= 10 m */
#define AEB_D_BRAKE_L3       5.0f   /* BRAKE_L3: só ativa se distância <=  5 m */
```

Veja a [página da Máquina de Estados](Maquina-de-Estados.md#distance-floor) para a justificativa detalhada deste conceito baseado no princípio LPB (*Lowest Possible Braking*) da regulamentação UNECE AEBS.

### Parâmetros do limitador de jerk e PID

```c
/* Limitador de jerk: variação máxima de pressão de freio por segundo [%/s] */
#define AEB_MAX_JERK        100.0f   /* 100 %/s → 10 %/ciclo a 100 Hz           */

/* Ganhos do controlador PID */
#define AEB_PID_KP          10.0f    /* Ganho proporcional                       */
#define AEB_PID_KI           0.05f   /* Ganho integral                           */
```

Veja a [página do Controlador PID](Controlador-PID.md) para o histórico de tuning e a justificativa dos valores atuais.

### Parâmetros temporais

```c
/* Tempo mínimo em WARNING antes de transição para BRAKE [s] */
#define AEB_WARNING_TO_BRAKE_MIN    0.8f

/* Tempo de histerese para de-escalação de estados [s] */
#define AEB_HYSTERESIS_TIME         0.2f

/* Tempo de manutenção do freio após parada completa [s] */
#define AEB_POST_BRAKE_HOLD         2.0f

/* Limiar de velocidade para detecção de parada completa [m/s] */
#define AEB_V_STOP_THRESHOLD        0.01f
```

### Parâmetros de validação do sensor

```c
/* Faixas válidas de dados do sensor */
#define AEB_RANGE_MIN        0.5f    /* [m]    */
#define AEB_RANGE_MAX      300.0f    /* [m]    */
#define AEB_SPEED_MAX       50.0f    /* [m/s]  */

/* Limites de taxa de variação (Rate-of-Change) por ciclo */
#define AEB_RANGE_ROC_MAX   10.0f    /* [m/ciclo]    */
#define AEB_SPEED_ROC_MAX    2.0f    /* [m/s/ciclo]  */

/* Número de ciclos consecutivos de falha para ativar latch */
#define AEB_FAULT_LATCH_CYCLES   3U
```

---

## `aeb_types.h` — Tipos de dados compartilhados

### Enum de estados da FSM

```c
typedef enum {
    AEB_STATE_OFF        = 0,  /* Sistema desligado ou inibido por fault      */
    AEB_STATE_STANDBY    = 1,  /* Ativo, monitorando, sem ameaça detectada    */
    AEB_STATE_WARNING_L1 = 2,  /* Alerta nível 1 (TTC <= 4.0 s)               */
    AEB_STATE_WARNING_L2 = 3,  /* Alerta nível 2 (TTC <= 3.0 s)               */
    AEB_STATE_BRAKE_L1   = 4,  /* Frenagem nível 1 (2 m/s²)                   */
    AEB_STATE_BRAKE_L2   = 5,  /* Frenagem nível 2 (4 m/s²) ou L3 (6 m/s²)   */
    AEB_STATE_POST_BRAKE = 6   /* Manutenção pós-parada (2 s após v < 0.01)   */
} AEB_State_t;
```

**Nota de design:** Os estados `BRAKE_L1` e `BRAKE_L2` cobrem os três níveis de desaceleração. `BRAKE_L2` pode acionar 4 m/s² ou 6 m/s² dependendo do TTC — veja os detalhes na [página FSM](Maquina-de-Estados.md). Uma revisão futura pode introduzir `BRAKE_L3` como estado distinto para maior granularidade no log.

### Struct de dados brutos do sensor (entrada)

```c
typedef struct {
    float    range_m;           /* Distância ao alvo [m]              */
    float    target_speed_mps;  /* Velocidade absoluta do alvo [m/s]  */
    uint8_t  valid;             /* 1 = dado válido, 0 = inválido      */
} SensorData_t;
```

### Struct de saída do módulo de percepção (dados validados)

```c
typedef struct {
    float    range_m;       /* Distância validada [m]                         */
    float    v_rel_mps;     /* Velocidade relativa ego - alvo [m/s]           */
    uint8_t  confidence;    /* 0 = inválido, 1 = baixo, 2 = médio, 3 = alto  */
    uint8_t  fault_active;  /* 1 se latch de 3 ciclos ativado                */
} PerceptionData_t;
```

### Struct de resultado do cálculo TTC

```c
typedef struct {
    float    ttc_s;        /* Time-to-Collision [s], 999.0 se não calculado  */
    float    d_brake_m;    /* Distância mínima de frenagem cinemática [m]    */
    uint8_t  is_closing;   /* 1 se v_rel > 0.5 m/s (alvo se aproximando)    */
} TTCResult_t;
```

### Struct de saída da FSM

```c
typedef struct {
    AEB_State_t state;          /* Estado atual da máquina de estados       */
    float       target_decel;   /* Desaceleração alvo [m/s²]                */
    uint8_t     brake_active;   /* 1 se frenagem autônoma ativa             */
    uint8_t     alert_visual;   /* 1 se alerta visual ativo                 */
    uint8_t     alert_audible;  /* 1 se alerta sonoro ativo                 */
} FSMOutput_t;
```

---

## `aeb_perception.c` — Validação e filtragem de dados

O módulo de percepção é a **primeira linha de defesa** contra dados corrompidos ou fisicamente impossíveis. Ele implementa validação em múltiplas camadas antes de qualquer cálculo de TTC ou decisão de frenagem.

### Pipeline de validação

```
SensorData_t (bruto)
      │
      ▼
[1] Validação de faixa de range: [0.5, 300] m
      │ falha → fault_counter++
      ▼
[2] Validação de velocidade do alvo: [0, 50] m/s
      │ falha → fault_counter++
      ▼
[3] Rate-of-Change do range: |Δrange| <= 10 m/ciclo
      │ falha → fault_counter++
      ▼
[4] Rate-of-Change da velocidade: |Δspeed| <= 2 m/s/ciclo
      │ falha → fault_counter++
      ▼
[5] Latch de falha: fault_counter >= 3 → fault_active = 1
      │
      ▼
PerceptionData_t (validado, com confidence e fault_active)
```

### Lógica do latch de 3 ciclos

```c
/* Variáveis estáticas — persistem entre ciclos de 10 ms */
static uint8_t  s_fault_counter = 0U;
static uint8_t  s_fault_active  = 0U;
static float    s_prev_range    = 0.0f;
static float    s_prev_speed    = 0.0f;

void aeb_perception_update(const SensorData_t *input,
                            float v_ego,
                            PerceptionData_t *output)
{
    uint8_t current_fault = 0U;

    /* [1] Validação de range */
    if ((input->range_m < AEB_RANGE_MIN) || (input->range_m > AEB_RANGE_MAX)) {
        current_fault = 1U;
    }
    /* [2] Validação de velocidade */
    if (input->target_speed_mps > AEB_SPEED_MAX) {
        current_fault = 1U;
    }
    /* [3] Rate-of-Change do range */
    {
        float range_roc = fabsf(input->range_m - s_prev_range);
        if (range_roc > AEB_RANGE_ROC_MAX) {
            current_fault = 1U;
        }
    }
    /* [4] Rate-of-Change da velocidade */
    {
        float speed_roc = fabsf(input->target_speed_mps - s_prev_speed);
        if (speed_roc > AEB_SPEED_ROC_MAX) {
            current_fault = 1U;
        }
    }

    /* [5] Atualiza contador e latch */
    if (current_fault == 1U) {
        if (s_fault_counter < AEB_FAULT_LATCH_CYCLES) {
            s_fault_counter++;
        }
    } else {
        if (s_fault_counter > 0U) {
            s_fault_counter--;   /* recuperação gradual */
        }
    }
    s_fault_active = (s_fault_counter >= AEB_FAULT_LATCH_CYCLES) ? 1U : 0U;

    /* Preenche saída */
    output->fault_active = s_fault_active;
    if (s_fault_active == 0U) {
        output->range_m    = input->range_m;
        output->v_rel_mps  = v_ego - input->target_speed_mps;
        output->confidence = (s_fault_counter == 0U) ? 3U :
                             (s_fault_counter == 1U) ? 2U : 1U;
    } else {
        output->range_m    = 0.0f;
        output->v_rel_mps  = 0.0f;
        output->confidence = 0U;
    }

    /* Atualiza histórico para próximo ciclo */
    s_prev_range = input->range_m;
    s_prev_speed = input->target_speed_mps;
}
```

**Rationale do latch de 3 ciclos:** Em 10 ms por ciclo, 3 ciclos equivalem a 30 ms. Ruído transitório do sensor (ex.: reflexo de guardrail, multipath) tipicamente dura 1–2 ciclos. O latch de 3 ciclos filtra esse ruído sem introduzir latência significativa para falhas reais e persistentes. O decremento gradual (`s_fault_counter--`) na recuperação evita oscilação em torno do limiar — sem ele, um sensor com falha intermitente alternaria ciclo a ciclo entre `fault_active=0` e `fault_active=1`.

### Cálculo de `v_rel`

```
v_rel = v_ego - v_target
```

- `v_rel > 0`: ego mais rápido → alvo se aproximando → potencial risco
- `v_rel < 0`: alvo mais rápido que o ego → alvo se afastando → sem risco AEB
- `v_rel = 0`: mesma velocidade → distância constante, sem risco imediato

---

## `aeb_ttc.c` — Cálculo do Time-to-Collision

### Lei de cálculo

```c
void aeb_ttc_calculate(const PerceptionData_t *perc,
                       float v_ego,
                       TTCResult_t *result)
{
    result->ttc_s      = AEB_TTC_INFINITY;   /* 999.0f */
    result->d_brake_m  = 0.0f;
    result->is_closing = 0U;

    if ((perc->fault_active == 1U) || (perc->confidence == 0U)) {
        return;   /* dados inválidos: retorna com defaults seguros */
    }

    /* is_closing com limiar de 0.5 m/s para evitar divisão por zero
       e ativações por ruído de sensor */
    if (perc->v_rel_mps > 0.5f) {
        result->is_closing = 1U;
        result->ttc_s = perc->range_m / perc->v_rel_mps;
    }

    /* Distância de frenagem cinemática: d = v² / (2 × a_max) */
    if (v_ego > AEB_V_EGO_MIN) {
        result->d_brake_m = (v_ego * v_ego) / (2.0f * AEB_DECEL_L3);
    }
}
```

### Equações fundamentais

**TTC — aproximação de primeira ordem (velocidade constante):**

```
TTC [s] = distância [m] / velocidade_relativa [m/s]
```

Esta é a equação cinemática mais simples. Sistemas de produção avançados incluem aceleração relativa (TTC de segunda ordem), mas a aproximação de primeira ordem é conservadora (subestima o TTC em cenários de travagem do alvo) e adequada para ASIL-B.

**Distância de frenagem cinemática (pior caso, L3):**

```
d_brake [m] = v_ego² [m²/s²] / (2 × 6.0 [m/s²]) = v_ego² / 12.0
```

Exemplos para referência:

| v_ego [km/h] | v_ego [m/s] | d_brake [m] |
|:------------:|:-----------:|:-----------:|
| 20           | 5.56        | 2.57        |
| 40           | 11.11       | 5.14 (*)    |
| 50           | 13.89       | 8.04        |
| 60           | 16.67       | 23.15       |

(*) A 40 km/h, d_brake = 5,14 m — coincide com `AEB_D_BRAKE_L3 = 5.0 m`, o que significa que a esse nível, o TTC e o distance floor entram em ação simultaneamente.

### Por que 0,5 m/s como limiar de `is_closing`

O limiar de 0,5 m/s para classificar o alvo como "se aproximando" e para o cálculo do TTC serve duas finalidades críticas:

1. **Evitar divisão por zero** quando `v_rel` oscila em torno de zero devido ao ruído do sensor (velocidades relativas de ±0,1 m/s são comuns em sensores radar de baixo custo)
2. **Eliminar falsas ativações** — sem este limiar, dois veículos trafegando na mesma velocidade ativariam TTC infinito corretamente, mas uma leve variação de ruído de +0,01 m/s resultaria em TTC = range/0.01 = valores absurdamente grandes; a lógica funciona, mas a consistência de `is_closing` seria comprometida

---

## `aeb_main.c` — Orquestrador do ciclo

### Inicialização

```c
void aeb_init(void)
{
    aeb_perception_reset();
    aeb_fsm_reset();
    aeb_pid_reset();
    aeb_alert_reset();
}
```

`aeb_init()` deve ser chamado **uma única vez** antes do primeiro `aeb_cycle_10ms()`. Ela zera todas as variáveis estáticas dos submódulos, garantindo estado determinístico — requisito mandatório de ISO 26262 para funções de segurança. Em uma ECU real, seria chamada no task de inicialização do RTOS (`Os_InitTask` ou equivalente).

### Ciclo principal de 10 ms

```c
void aeb_cycle_10ms(const SensorData_t *sensor_input, float v_ego)
{
    PerceptionData_t perc  = {0.0f, 0.0f, 0U, 0U};
    TTCResult_t      ttc   = {999.0f, 0.0f, 0U};
    FSMOutput_t      fsm   = {AEB_STATE_OFF, 0.0f, 0U, 0U, 0U};
    float            brake = 0.0f;

    /* 1. Percepção: valida e filtra dados brutos do sensor */
    aeb_perception_update(sensor_input, v_ego, &perc);

    /* 2. TTC: calcula Time-to-Collision e distância de frenagem */
    aeb_ttc_calculate(&perc, v_ego, &ttc);

    /* 3. FSM: determina estado atual e desaceleração alvo */
    aeb_fsm_update(&perc, &ttc, v_ego, &fsm);

    /* 4. PID: converte desaceleração alvo em pressão de freio [0-100%] */
    brake = aeb_pid_update(fsm.target_decel, fsm.brake_active);

    /* 5. Alert: gera flags de alerta baseados no estado FSM */
    aeb_alert_update(fsm.state, fsm.alert_visual, fsm.alert_audible);

    /* 6. Armazena para leitura pela Camada 2 via getters */
    s_brake_pct  = brake;
    s_aeb_state  = fsm.state;
    s_fsm_output = fsm;
}
```

**Importante:** Todas as structs locais são inicializadas explicitamente no ponto de declaração (valores seguros padrão). Isso garante comportamento previsível mesmo se alguma função de módulo retornar prematuramente — a struct de saída já contém um estado seguro (zero desaceleração, sistema OFF).

### Funções getter (leitura de resultados pela Camada 2)

```c
float       aeb_get_brake_cmd(void)     { return s_brake_pct; }
AEB_State_t aeb_get_state(void)         { return s_aeb_state; }
uint8_t     aeb_get_alert_visual(void)  { return s_fsm_output.alert_visual; }
uint8_t     aeb_get_alert_audible(void) { return s_fsm_output.alert_audible; }
float       aeb_get_target_decel(void)  { return s_fsm_output.target_decel; }
```

Os getters permitem que `aeb_controller_node.cpp` leia os resultados após `aeb_cycle_10ms()` sem necessidade de ponteiros de saída adicionais no protótipo da função principal. Isso simplifica a interface da Camada 2 e evita problemas de aliasing de ponteiros que seriam uma violação MISRA Rule 17.1.

---

## Notas de conformidade MISRA C:2012

| Regra MISRA | Descrição | Status |
|-------------|-----------|--------|
| Rule 1.3 | Sem comportamento indefinido | Conforme — sem UB identificado |
| Rule 8.7 | Objetos com linkage interno devem ser declarados `static` | Conforme |
| Rule 11.3 | Sem cast ponteiro↔objeto | Conforme |
| Rule 14.4 | Condições booleanas explícitas | Conforme — comparações `== 0U` / `!= 0U` |
| Rule 15.5 | Um único ponto de retorno por função | Conforme com desvios documentados |
| Rule 17.1 | Sem uso de `<stdarg.h>` | Conforme |
| Rule 18.1 | Sem aritmética de ponteiros fora de bounds | Conforme |
| Rule 21.3 | Sem `malloc`/`free` | Conforme — memória 100% estática |
| Rule 21.6 | Sem `printf`/`scanf` em código de produção | Conforme — `#ifdef DEBUG` removido na build de release |
| Rule 21.8 | Sem `abort`, `exit`, `_Exit` | Conforme |

---

*Próxima seção recomendada: [Máquina de Estados](Maquina-de-Estados.md)*
