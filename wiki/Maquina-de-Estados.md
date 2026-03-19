# Máquina de Estados (FSM)

> **Página:** Máquina de Estados
> **Relacionado:** [Home](Home.md) | [Arquitetura do Sistema](Arquitetura-do-Sistema.md) | [Módulos C Embarcado](Modulos-C-Embarcado.md) | [Controlador PID](Controlador-PID.md)

---

## Visão geral

A Máquina de Estados Finita (*Finite State Machine* — FSM) do AEB é implementada em `aeb_fsm.c` e é o componente de maior criticidade funcional do sistema. Ela recebe os dados de percepção validados e os resultados do cálculo TTC, e determina a cada ciclo de 10 ms:

1. O **estado de operação atual** do sistema (um dos 7 estados)
2. O **nível de desaceleração alvo** a ser seguido pelo controlador PID
3. Os **flags de alerta** visual e sonoro
4. Se a **frenagem autônoma** está ativa (`brake_active`)

Todas as variáveis de temporização (histerese, WARNING_TO_BRAKE, POST_BRAKE) são mantidas como variáveis estáticas acumuladas em ciclos de 10 ms (dt = 0,01 s), sem uso de timers externos — compatível com ambiente bare-metal sem RTOS.

---

## Tabela completa de estados

| Estado | ID | Condição de entrada | `target_decel` [m/s²] | `brake_active` | `alert_visual` | `alert_audible` |
|--------|:--:|---------------------|:---------------------:|:--------------:|:--------------:|:---------------:|
| `OFF` | 0 | Fault ativo, ou sistema inibido por chave lógica | 0.0 | 0 | 0 | 0 |
| `STANDBY` | 1 | Sistema ativo, v_ego ∈ [1,39; 16,67] m/s, sem ameaça | 0.0 | 0 | 0 | 0 |
| `WARNING_L1` | 2 | TTC ≤ 4,0 s e `is_closing = 1` | 0.0 | 0 | 1 | 0 |
| `WARNING_L2` | 3 | TTC ≤ 3,0 s e `is_closing = 1` | 0.0 | 0 | 1 | 1 |
| `BRAKE_L1` | 4 | TTC ≤ 2,2 s **e** d ≤ 20 m **e** `warning_timer ≥ 0,8 s` | 2.0 | 1 | 1 | 1 |
| `BRAKE_L2` | 5 | TTC ≤ 1,8 s **e** d ≤ 10 m | 4.0 (ou 6.0 se d ≤ 5 m) | 1 | 1 | 1 |
| `POST_BRAKE` | 6 | v_ego < 0,01 m/s durante frenagem ativa | 6.0 | 1 | 1 | 0 |

**Notas:**
- `target_decel` em `BRAKE_L2` é **4,0 m/s²** quando apenas TTC ≤ 1,8 s e d ≤ 10 m; sobe para **6,0 m/s²** adicionalmente quando d ≤ 5 m (distance floor L3 ativo)
- `POST_BRAKE` mantém `target_decel = 6,0 m/s²` por toda a duração de 2 s, garantindo que o veículo permaneça parado mesmo em inclinações suaves
- O alerta sonoro em `POST_BRAKE` é desligado (`alert_audible = 0`) pois o veículo já está parado — alerta residual seria inconfortável e sem função de segurança

---

## Regras de transição completas

### Escalação (aumento de severidade)

As transições de escalação seguem a hierarquia determinada por `evaluate_threat()`. Para cada ciclo, a ameaça é reavaliada e a transição ocorre **imediatamente** (sem atraso), exceto quando o `WARNING_TO_BRAKE_MIN` está em jogo:

```
OFF ──[fault_cleared + v_ego in range]──► STANDBY

STANDBY ──[TTC ≤ 4,0 s + is_closing]──► WARNING_L1

WARNING_L1 ──[TTC ≤ 3,0 s]──► WARNING_L2

WARNING_L2 ──[TTC ≤ 2,2 s + d ≤ 20 m + warning_timer ≥ 0,8 s]──► BRAKE_L1
WARNING_L1 ──[TTC ≤ 2,2 s + d ≤ 20 m + warning_timer ≥ 0,8 s]──► BRAKE_L1

BRAKE_L1 ──[TTC ≤ 1,8 s + d ≤ 10 m]──► BRAKE_L2

BRAKE_L1 ──[v_ego < 0,01 m/s]──► POST_BRAKE
BRAKE_L2 ──[v_ego < 0,01 m/s]──► POST_BRAKE

POST_BRAKE ──[timer ≥ 2,0 s]──► STANDBY
```

A transição direta `WARNING → BRAKE_L1` sem passar por WARNING_L2 é possível se o TTC cair abruptamente (ex.: freada súbita do veículo à frente), desde que o `warning_timer` já acumule 0,8 s em qualquer estado de WARNING.

### De-escalação (redução de severidade)

De-escalações exigem que a condição de ameaça reduzida persista por `AEB_HYSTERESIS_TIME = 0,2 s` antes de ser aplicada:

```
BRAKE_L2 ──[TTC > 2,0 s por 0,2 s]──► BRAKE_L1
BRAKE_L1 ──[TTC > 2,5 s por 0,2 s]──► WARNING_L2
WARNING_L2 ──[TTC > 3,5 s por 0,2 s]──► WARNING_L1
WARNING_L1 ──[TTC > 4,5 s por 0,2 s OU !is_closing]──► STANDBY
```

Os limiares de de-escalação são **propositalmente mais altos** que os limiares de escalação correspondentes, criando uma zona de histerese assimétrica. Exemplos:

| Escalação ativa em | De-escalação ativa em | Zona de histerese |
|:------------------:|:--------------------:|:-----------------:|
| TTC ≤ 4,0 s (W_L1) | TTC > 4,5 s (W_L1→STBY) | 0,5 s de TTC |
| TTC ≤ 3,0 s (W_L2) | TTC > 3,5 s (W_L2→W_L1) | 0,5 s de TTC |
| TTC ≤ 2,2 s (B_L1) | TTC > 2,5 s (B_L1→W_L2) | 0,3 s de TTC |
| TTC ≤ 1,8 s (B_L2) | TTC > 2,0 s (B_L2→B_L1) | 0,2 s de TTC |

### Overrides globais (maior prioridade absoluta)

Estas transições são verificadas **antes** de qualquer lógica de ameaça e preemptem qualquer estado:

```c
/* Override 1: Fault ativo em percepção → força OFF */
if (perc->fault_active == 1U) {
    next_state = AEB_STATE_OFF;
    goto fsm_done;
}

/* Override 2: Velocidade fora da faixa de operação → força STANDBY
   (com exceção de iminência de colisão — veja Exception 2 abaixo) */
if ((v_ego < AEB_V_EGO_MIN) || (v_ego > AEB_V_EGO_MAX)) {
    if (imminent_collision == 0U) {
        next_state = AEB_STATE_STANDBY;
        goto fsm_done;
    }
}

/* Override 3: Driver override (acelerador pressionado) → força STANDBY */
if (driver_override_active == 1U) {
    next_state = AEB_STATE_STANDBY;
    goto fsm_done;
}
```

**Nota MISRA sobre `goto`:** O uso de `goto` forward para um label único `fsm_done:` ao fim da função é um **desvio documentado** da regra MISRA 15.1. A justificativa aceita é que `goto` forward para um único label de saída é o mecanismo mais direto, auditável e verificável para implementar prioridade de overrides em C sem introduzir aninhamento profundo de `if/else` ou flags booleanas que obscurecem o fluxo de controle. A alternativa — múltiplas flags — produziria código mais difícil de certificar.

---

## Função `evaluate_threat()`

Esta função interna determina o nível de ameaça instantâneo baseado em TTC, distância e `is_closing`. É chamada a cada ciclo dentro de `aeb_fsm_update()`:

```c
typedef enum {
    THREAT_NONE   = 0,   /* Sem ameaça detectada                */
    THREAT_WARN1  = 1,   /* Nível de alerta 1 (TTC <= 4.0 s)    */
    THREAT_WARN2  = 2,   /* Nível de alerta 2 (TTC <= 3.0 s)    */
    THREAT_BRAKE1 = 3,   /* Frenagem nível 1 (TTC <= 2.2 s)     */
    THREAT_BRAKE2 = 4,   /* Frenagem nível 2 (TTC <= 1.8 s)     */
    THREAT_BRAKE3 = 5    /* Frenagem nível 3 (d <= 5 m + B2)    */
} ThreatLevel_t;

static ThreatLevel_t evaluate_threat(const PerceptionData_t *perc,
                                     const TTCResult_t *ttc)
{
    ThreatLevel_t level = THREAT_NONE;

    /* Sem fechamento → retorna imediatamente, sem ameaça */
    if (ttc->is_closing == 0U) {
        return THREAT_NONE;
    }

    /* Escalonamento primário por TTC */
    if (ttc->ttc_s <= AEB_TTC_BRAKE_L2) {           /* <= 1.8 s */
        level = THREAT_BRAKE2;
    } else if (ttc->ttc_s <= AEB_TTC_BRAKE_L1) {    /* <= 2.2 s */
        level = THREAT_BRAKE1;
    } else if (ttc->ttc_s <= AEB_TTC_WARN_L2) {     /* <= 3.0 s */
        level = THREAT_WARN2;
    } else if (ttc->ttc_s <= AEB_TTC_WARN_L1) {     /* <= 4.0 s */
        level = THREAT_WARN1;
    } else {
        level = THREAT_NONE;
    }

    /* Upgrade para THREAT_BRAKE3 se em frenagem E distância crítica */
    if ((level >= THREAT_BRAKE2) && (perc->range_m <= AEB_D_BRAKE_L3)) {
        level = THREAT_BRAKE3;
    }

    return level;
}
```

---

## Distance Floor — Lógica de distância mínima de frenagem {#distance-floor}

### O problema que o Distance Floor resolve

Em um cenário de ultrapassagem rápida, o TTC pode temporariamente cair para 2,0 s com o alvo a 60 m de distância — nenhuma ação de frenagem seria adequada, pois o veículo está claramente mudando de faixa. Sem o distance floor, o sistema interpretaria este TTC como uma emergência e aplicaria BRAKE_L1.

O distance floor adiciona uma **condição AND de distância** nas transições de frenagem: além do TTC baixo, o alvo deve estar fisicamente próximo o suficiente para justificar frenagem autônoma.

### Implementação do gating por distância

```c
/* Aplicado após evaluate_threat(), antes de usar o nível na FSM */

/* BRAKE_L1 só é válido se d <= 20 m */
if ((threat >= THREAT_BRAKE1) && (perc->range_m > AEB_D_BRAKE_L1)) {
    threat = THREAT_WARN2;   /* rebaixa: distante demais para freiar */
}
/* BRAKE_L2 só é válido se d <= 10 m */
if ((threat >= THREAT_BRAKE2) && (perc->range_m > AEB_D_BRAKE_L2)) {
    threat = THREAT_BRAKE1;  /* rebaixa um nível */
}
/* BRAKE_L3 (via BRAKE3) só é válido se d <= 5 m */
if ((threat >= THREAT_BRAKE3) && (perc->range_m > AEB_D_BRAKE_L3)) {
    threat = THREAT_BRAKE2;  /* rebaixa um nível */
}
```

Este gating é aplicado **apenas quando `is_closing = 1`** — se o alvo não está se aproximando, `evaluate_threat()` já retornou `THREAT_NONE` antes de chegar aqui.

### Referências normativas

| Fonte | Trecho relevante |
|-------|-----------------|
| **UNECE Regulation No. 152** | Seção 5.2.2 — define distâncias mínimas de ativação de AEBS por faixa de velocidade do ego |
| **ISO 22839:2013** | Clause 6.3 — *Forward Vehicle Collision Warning Systems: functional requirements* incluem distance floor para reduzir falsas ativações |
| **MDPI Sensors 2024** | "Optimizing AEB Thresholds for NCAP" — análise quantitativa mostrando que distance floors de 20/10/5 m reduzem em 34% as falsas ativações em cenários de autoestrada sem comprometer pontuação CCR |
| **LPB — Lowest Possible Braking** | Princípio de que a intervenção autônoma deve ser a **mínima necessária** para evitar colisão, não a máxima possível |

---

## Exceção de iminência em V_EGO_MIN (Exception 2 — `fsm_update`)

### O problema

O override `v_ego < AEB_V_EGO_MIN → STANDBY` existe para prevenir ativações em manobras de baixa velocidade. Porém, ele cria um risco específico: um veículo desacelerando de 6 km/h para 4 km/h nos últimos 3 metros antes de um obstáculo parado teria o AEB desativado exatamente quando a frenagem é mais crítica.

### A exceção

```c
/* Determina iminência de colisão: alvo próximo E fechando */
uint8_t imminent_collision = 0U;
if ((perc->range_m < AEB_D_BRAKE_L3) &&   /* d < 5 m */
    (ttc->is_closing == 1U)) {
    imminent_collision = 1U;
}

/* Override de velocidade mínima — com exceção de iminência */
if ((v_ego < AEB_V_EGO_MIN) && (imminent_collision == 0U)) {
    next_state = AEB_STATE_STANDBY;
    goto fsm_done;
}
/* Se imminent_collision == 1U: NÃO aplica o override.
   O sistema mantém o estado de frenagem atual. */
```

### Critério para "iminência"

A iminência é definida pela conjunção de **dois critérios**:
1. Distância ao alvo < `AEB_D_BRAKE_L3 = 5 m` — dentro da zona de frenagem máxima
2. `is_closing = 1` — o alvo ainda está se aproximando (não é um objeto estático que o ego já passou)

Apenas distância próxima sem fechamento (ex.: ego parado a 3 m de uma parede) não configura iminência e não aciona a exceção.

---

## POST_BRAKE: estado de manutenção pós-parada

### Requisito funcional

**FR-BRK-005:** Após o veículo atingir v_ego < 0,01 m/s enquanto em estado de frenagem ativa (`BRAKE_L1` ou `BRAKE_L2`), o sistema deve manter pressão de freio equivalente a `AEB_DECEL_L3 = 6 m/s²` por `AEB_POST_BRAKE_HOLD = 2,0 s`.

### Implementação

```c
/* Timer do POST_BRAKE: variável estática, acumulada em dt */
static float s_post_brake_timer = 0.0f;

/* Transição para POST_BRAKE (detectada dentro do handler de BRAKE_Lx): */
if (v_ego < AEB_V_STOP_THRESHOLD) {   /* 0.01 m/s */
    s_post_brake_timer = 0.0f;
    next_state = AEB_STATE_POST_BRAKE;
}

/* Handler do estado POST_BRAKE: */
case AEB_STATE_POST_BRAKE:
{
    s_post_brake_timer += 0.01f;         /* dt = 10 ms */

    output->target_decel  = AEB_DECEL_L3;  /* 6.0 m/s² */
    output->brake_active  = 1U;
    output->alert_visual  = 1U;
    output->alert_audible = 0U;            /* sonoro desligado: veículo parado */

    if (s_post_brake_timer >= AEB_POST_BRAKE_HOLD) {  /* 2.0 s */
        s_post_brake_timer = 0.0f;
        next_state = AEB_STATE_STANDBY;
    }
    break;
}
```

### Justificativa dos 2,0 s de hold

| Fator | Valor | Fonte |
|-------|-------|-------|
| Tempo de reação do motorista surpreso | 1,2–2,0 s | SAE J2944 |
| Tempo exigido por UNECE 152 | ≥ 1,8 s | UNECE Reg. 152, §5.2.4 |
| Gradiente de via máximo considerado | 5% (~0,5 m/s²) | Norma Euro NCAP CCR |
| Margem sobre requisito UNECE | 11% (2,0 vs 1,8 s) | Escolha de projeto |

Com `AEB_DECEL_L3 = 6 m/s²` e gradiente máximo de 0,5 m/s², a força resultante de frenagem mesmo no pior caso é de 5,5 m/s² — suficiente para manter parado. O alerta sonoro é desligado pois não há função de segurança em manter alerta audível após parada completa (veículo imóvel não colide).

---

## WARNING_TO_BRAKE_MIN: garantia de alerta antes da frenagem

### Requisito

Antes de qualquer transição para estado BRAKE, o sistema deve ter permanecido em estado WARNING por pelo menos `AEB_WARNING_TO_BRAKE_MIN = 0,8 s`.

### Implementação

```c
static float s_warning_timer = 0.0f;

/* Em cada ciclo que o estado é WARNING_L1 ou WARNING_L2: */
if ((current_state == AEB_STATE_WARNING_L1) ||
    (current_state == AEB_STATE_WARNING_L2)) {
    s_warning_timer += 0.01f;   /* acumula dt */
} else {
    s_warning_timer = 0.0f;     /* reset ao sair de WARNING */
}

/* Condição de transição para BRAKE (gate AND): */
if ((threat >= THREAT_BRAKE1) &&
    (distance_ok == 1U) &&
    (s_warning_timer >= AEB_WARNING_TO_BRAKE_MIN)) {  /* 0.8 s */
    next_state = AEB_STATE_BRAKE_L1;
}
```

### Justificativa dos 0,8 s

- **Evitabilidade pelo motorista:** O protocolo Euro NCAP CCR define que o sistema deve dar ao motorista a oportunidade de reagir antes da frenagem autônoma. Um alerta de 0,8 s a 50 km/h cobre ~11 m — suficiente para o motorista perceber e iniciar frenagem própria
- **Prevenção de frenagem por falso positivo transitório:** Se o TTC cair brevemente (1–3 ciclos = 10–30 ms) devido a ruído de sensor, o timer de 0,8 s (80 ciclos) não será atingido, prevenindo frenagem espúria
- **Alinhamento com Euro NCAP 2024:** A pontuação máxima no teste CCR exige alerta visual+sonoro precedendo a frenagem autônoma em ≥ 700 ms. O valor de 0,8 s proporciona 14% de margem.

---

## Histerese de de-escalação

### Problema sem histerese

Sem histerese, um cenário de TTC oscilando em torno de 3,0 s causaria transições rápidas entre `WARNING_L1` e `WARNING_L2` a cada ciclo — comportamento inaceitável em termos de conforto, segurança e diagnóstico.

### Implementação

```c
static float         s_hysteresis_timer      = 0.0f;
static ThreatLevel_t s_pending_deescalation  = THREAT_NONE;

/* Ao detectar redução de ameaça: */
if (new_threat < s_current_threat) {
    if (new_threat == s_pending_deescalation) {
        /* Mesma de-escalação pendente: acumula timer */
        s_hysteresis_timer += 0.01f;
        if (s_hysteresis_timer >= AEB_HYSTERESIS_TIME) {  /* 0.2 s */
            s_current_threat      = new_threat;
            s_hysteresis_timer    = 0.0f;
        }
    } else {
        /* De-escalação diferente: reinicia timer com nova meta */
        s_pending_deescalation = new_threat;
        s_hysteresis_timer     = 0.01f;
    }
} else {
    /* Ameaça igual ou maior: cancela qualquer de-escalação pendente */
    s_pending_deescalation = s_current_threat;
    s_hysteresis_timer     = 0.0f;
}
```

### Por que 0,2 s (20 ciclos)?

- Ruído de sensor típico dura 1–3 ciclos (10–30 ms) → filtrado completamente por 20 ciclos
- 0,2 s é percebido como instantâneo pelo motorista (limiar de percepção de mudança visual ~250 ms)
- UNECE 152 não especifica valor de histerese mas recomenda que o sistema não alterne estados em menos de 100 ms — 0,2 s proporciona margem de 100%

---

## Diagrama de transição de estados

```
              ┌──────────────────────────────────────────────────┐
              │        OVERRIDES GLOBAIS (prioridade máxima)     │
              │  [1] fault_active=1          → OFF               │
              │  [2] v fora de [1.39,16.67]  → STANDBY*          │
              │  [3] driver_override=1        → STANDBY           │
              │  (* com exceção de iminência < 5 m)              │
              └──────────────────────────────────────────────────┘

  ┌────────────────────────────────────────────────────────────────────┐
  │                              OFF (0)                               │
  │  fault ativo / sistema inibido                                     │
  └──────────────────────────────┬─────────────────────────────────────┘
                                 │ fault cleared + v in range
                                 ▼
  ┌────────────────────────────────────────────────────────────────────┐
  │                           STANDBY (1)                              │
  │  v_ego ∈ [1.39, 16.67] m/s, sem ameaça ativa                     │
  └──────────┬─────────────────────────────────────────────────────────┘
             │ TTC ≤ 4.0 s + is_closing                  ▲
             ▼                                TTC > 4.5 s × 0.2 s
  ┌──────────────────────┐◄────────────────────────────────────────────
  │    WARNING_L1 (2)    │
  │  visual=1, aud=0     │──────────────────────────────────────────►│
  └──────────┬───────────┘ TTC > 3.5 s × 0.2 s          ▲          │
             │ TTC ≤ 3.0 s                               │          │ TTC > 4.5s
             ▼                                           │          │ × 0.2s
  ┌──────────────────────┐◄────────────────────────────────          │
  │    WARNING_L2 (3)    │                                           │
  │  visual=1, aud=1     │──────────────────────────────────────────►┘
  └──────────┬───────────┘ TTC > 2.5 s × 0.2 s
             │ TTC ≤ 2.2 s + d ≤ 20 m + warning_timer ≥ 0.8 s
             ▼                         ▲
  ┌──────────────────────┐    TTC > 2.5 s × 0.2 s
  │     BRAKE_L1 (4)     │◄────────────────────────────────
  │  decel=2.0, brake=1  │
  └──────────┬───────────┘──────────────────────────────►│ TTC > 2.0s × 0.2s
             │ TTC ≤ 1.8 s + d ≤ 10 m                   ▼
             ▼                             ┌──────────────────────┐
  ┌──────────────────────┐                 │    de-escalação      │
  │     BRAKE_L2 (5)     │                 └──────────────────────┘
  │  decel=4.0 (ou 6.0)  │
  │  brake=1, aud=1      │
  └──────────┬───────────┘
             │ v_ego < 0.01 m/s
             ▼
  ┌──────────────────────┐
  │   POST_BRAKE (6)     │──────────────────► STANDBY (após 2.0 s)
  │  decel=6.0, brake=1  │
  │  visual=1, aud=0     │
  └──────────────────────┘
```

---

*Próxima seção recomendada: [Controlador PID](Controlador-PID.md)*
