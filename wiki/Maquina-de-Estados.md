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

Uma versão Stateflow/Simulink fiel ao código C está disponível em `AEB_Stateflow_build.m`.

---

## Tabela completa de estados

| Estado | ID | Condição de entrada | `target_decel` [m/s²] | `brake_active` | `alert_visual` | `alert_audible` |
|--------|:--:|---------------------|:---------------------:|:--------------:|:--------------:|:---------------:|
| `OFF` | 0 | Fault ativo, ou sistema inibido por chave/InfoCenter | 0.0 | 0 | 0 | 0 |
| `STANDBY` | 1 | Sistema ativo, v_ego in [2,78; 16,67] m/s (10–60 km/h), sem ameaça | 0.0 | 0 | 0 | 0 |
| `WARNING` | 2 | TTC <= 4,0 s e `is_closing = 1` (toda ameaça passa por aqui) | 0.0 | 0 | 1 | 1 |
| `BRAKE_L1` | 3 | TTC <= 3,0 s **e** `warning_timer >= 0,8 s` | 2.0 | 1 | 1 | 1 |
| `BRAKE_L2` | 4 | TTC <= 2,2 s | 4.0 | 1 | 1 | 1 |
| `BRAKE_L3` | 5 | TTC <= 1,8 s **ou** d_brake >= distance | 6.0 | 1 | 1 | 1 |
| `POST_BRAKE` | 6 | v_ego < 0,01 m/s durante frenagem ativa | 6.0 | 1 | 0 | 0 |

**Notas:**
- Toda ameaça detectada em `STANDBY` entra primeiro em `WARNING` — mesmo que o TTC já esteja abaixo dos limiares de frenagem. Escalação para BRAKE só acontece após o mínimo de 0,8 s em WARNING (FR-ALR-003).
- `POST_BRAKE` mantém `target_decel = 6,0 m/s²` por toda a duração de 2 s, garantindo que o veículo permaneça parado mesmo em inclinações suaves.
- Em `POST_BRAKE`, **ambos os alertas são desligados** (`alert_visual = 0`, `alert_audible = 0`) — FR-ALR-005 (alerta visual em POST\_BRAKE) foi removido no SRS v3. O foco desta fase é manter o freio aplicado, não emitir alertas.

---

## Regras de transição completas

### Escalação (aumento de severidade)

As transições de escalação seguem a hierarquia determinada por `evaluate_threat()` em `aeb_fsm.c`. A ameaça é reavaliada a cada ciclo e a escalação ocorre **imediatamente** (sem debounce), com uma exceção fundamental: toda ameaça partindo de STANDBY **obrigatoriamente** entra em WARNING primeiro.

```
OFF ──[~fault]──> STANDBY

STANDBY ──[desired >= WARNING + is_closing]──> WARNING
   (qualquer TTC <= 4.0 s OU d_brake >= distance OU distance <= D_BRAKE_L1)

WARNING ──[desired >= BRAKE_L1 + warning_timer >= 0.8 s]──> BRAKE_L1
WARNING ──[desired >= BRAKE_L2 + warning_timer >= 0.8 s]──> BRAKE_L2
WARNING ──[desired = BRAKE_L3 + warning_timer >= 0.8 s]──> BRAKE_L3

BRAKE_L1 ──[desired = BRAKE_L2 ou BRAKE_L3]──> (desired)   imediato
BRAKE_L2 ──[desired = BRAKE_L3]──> BRAKE_L3                imediato

BRAKE_L1 ──[v_ego < 0.01 m/s]──> POST_BRAKE
BRAKE_L2 ──[v_ego < 0.01 m/s]──> POST_BRAKE
BRAKE_L3 ──[v_ego < 0.01 m/s]──> POST_BRAKE

POST_BRAKE ──[timer >= 2.0 s]──> STANDBY
```

**Observação crítica:** A transição STANDBY -> WARNING -> BRAKE garante que o motorista sempre receba alertas visuais e sonoros **antes** de qualquer frenagem autônoma, mesmo em cenários de TTC muito baixo (ex.: cut-in súbito).

### De-escalação (redução de severidade)

De-escalações exigem que a condição de ameaça reduzida persista por `HYSTERESIS_TIME = 0,2 s` antes de ser aplicada. A de-escalação é **sempre de um nível por vez** (single-step), conforme implementado em `aeb_fsm.c`:

```
BRAKE_L3 ──[desired = BRAKE_L2 x 0.2 s]──> BRAKE_L2
BRAKE_L3 ──[desired = BRAKE_L1 x 0.2 s]──> BRAKE_L1
BRAKE_L3 ──[desired <= WARNING  x 0.2 s]──> POST_BRAKE

BRAKE_L2 ──[desired < BRAKE_L2 x 0.2 s]──> BRAKE_L1

BRAKE_L1 ──[desired < BRAKE_L1 x 0.2 s]──> WARNING

WARNING  ──[desired = STANDBY  x 0.2 s]──> STANDBY
```

**Por que single-step?** Evita transições abruptas de BRAKE_L3 diretamente para STANDBY — o motorista sente a redução gradual de frenagem, o que é mais seguro e confortável.

**Exceção BRAKE_L3:** Se `desired <= WARNING` (TTC alto, ameaça desapareceu), a de-escalação vai para `POST_BRAKE` ao invés de percorrer L2 -> L1 -> WARNING -> STANDBY. Isso porque o veículo em BRAKE_L3 provavelmente já está quase parado — manter frenagem residual é mais seguro do que soltar o freio gradualmente.

### Por que 0,2 s (20 ciclos)?

- Ruído de sensor típico dura 1-3 ciclos (10-30 ms) -> filtrado completamente por 20 ciclos
- 0,2 s é percebido como instantâneo pelo motorista (limiar de percepção ~250 ms)
- UNECE 152 recomenda que o sistema não alterne estados em menos de 100 ms — 0,2 s proporciona margem de 100%

---

## Overrides globais (maior prioridade absoluta)

Estas verificações são executadas **no início** de `fsm_update()`, antes de qualquer lógica de ameaça:

### 1. Fault -> OFF (FR-FSM-005)

```c
if (perception->fault != 0U)
{
    s_state = AEB_OFF;
    /* Reset all timers */
    return build_output(s_state);
}
```

Qualquer falha de percepção (timeout de radar/lidar, erro de CRC, falha de plausibilidade) força o sistema para OFF imediatamente.

### 2. Velocidade fora da faixa -> STANDBY

```c
if ((perception->v_ego < V_EGO_MIN) || (perception->v_ego > V_EGO_MAX))
```

Onde `V_EGO_MIN = 2,78 m/s` (10 km/h, corrigido per SRS v3 / FR-DEC-009) e `V_EGO_MAX = 16,67 m/s` (60 km/h).

> ⚠️ **Ação pendente no código C:** `aeb_config.h` define `V_EGO_MIN = 1.39f` — deve ser atualizado para `2.78f` para conformidade com FR-DEC-009 (SRS v3).

**Exceção 1 — Vehicle stopped while braking:** Se o veículo está em BRAKE_L1/L2/L3 e `v_ego < 0,01 m/s`, a transição vai para POST_BRAKE (não STANDBY). Isso garante que o freio seja mantido após parada.

**Exceção 2 — Close-range still closing:** Se `distance <= D_BRAKE_L1 (20 m)` e `v_rel > 0` (ainda fechando), o distance floor é aplicado mesmo com velocidade fora da faixa. Evita que o AEB desative nos últimos metros antes de uma colisão iminente.

### 3. Driver override -> STANDBY (FR-DEC-006, FR-DEC-007)

```c
static uint8_t driver_override(const PerceptionData_t *p)
{
    uint8_t ovr = 0U;
    if (p->brake_pedal != 0U)
        ovr = 1U;                                      /* freio pressionado */
    if (fabsf(p->steering_angle) > STEERING_OVERRIDE_DEG)
        ovr = 1U;                                      /* |theta| > 5 deg */
    return ovr;
}
```

**Diferenciação de overrides:**
- **Durante frenagem ativa (BRAKE_L1/L2/L3):** Pedal de freio OU volante > 5 deg -> STANDBY
- **Em POST_BRAKE:** Apenas pedal de acelerador -> STANDBY (o motorista decide retomar movimento)
- **Em WARNING:** Pedal de freio OU volante > 5 deg -> STANDBY (o motorista já reagiu)

---

## Função `evaluate_threat()`

Esta função determina o nível de ameaça instantâneo. Retorna o estado-alvo (`AEB_State_t`) baseado em TTC, distância de frenagem e proximidade:

```c
static AEB_State_t evaluate_threat(const TTCResult_t      *ttc,
                                   const PerceptionData_t *perception)
{
    AEB_State_t target = AEB_STANDBY;

    /* Sem fechamento -> sem ameaça */
    if (ttc->is_closing == 0U)
        return AEB_STANDBY;

    /* Escalonamento primário por TTC + d_brake */
    if ((ttc->ttc <= TTC_BRAKE_L3) || (ttc->d_brake >= perception->distance))
        target = AEB_BRAKE_L3;
    else if (ttc->ttc <= TTC_BRAKE_L2)
        target = AEB_BRAKE_L2;
    else if (ttc->ttc <= TTC_BRAKE_L1)
        target = AEB_BRAKE_L1;
    else if (ttc->ttc <= TTC_WARNING)
        target = AEB_WARNING;
    else
        target = AEB_STANDBY;

    /* Distance floor: impede de-escalação prematura quando próximo */
    if (ttc->is_closing != 0U) {
        if (perception->distance <= D_BRAKE_L3 && target < AEB_BRAKE_L3)
            target = AEB_BRAKE_L3;
        else if (perception->distance <= D_BRAKE_L2 && target < AEB_BRAKE_L2)
            target = AEB_BRAKE_L2;
        else if (perception->distance <= D_BRAKE_L1 && target < AEB_BRAKE_L1)
            target = AEB_BRAKE_L1;
    }

    return target;
}
```

### Limiares de TTC (de `aeb_config.h`)

| Parâmetro | Valor | Descrição |
|-----------|:-----:|-----------|
| `TTC_WARNING` | 4,0 s | Limiar para WARNING |
| `TTC_BRAKE_L1` | 3,0 s | Limiar para BRAKE_L1 |
| `TTC_BRAKE_L2` | 2,2 s | Limiar para BRAKE_L2 |
| `TTC_BRAKE_L3` | 1,8 s | Limiar para BRAKE_L3 |

### Critério `d_brake >= distance`

Quando a distância de frenagem calculada (`d_brake`) ultrapassa a distância real ao alvo, o sistema escala para BRAKE_L3 independente do TTC. Isso cobre cenários onde o TTC é alto (ex.: ego lento, alvo próximo) mas a distância de parada já foi excedida.

---

## Distance Floor — Lógica de distância mínima de frenagem

### O problema que o Distance Floor resolve

Em um cenário de ultrapassagem rápida, o TTC pode temporariamente cair para 2,0 s com o alvo a 60 m de distância — nenhuma ação de frenagem seria adequada. Sem o distance floor, o sistema interpretaria este TTC como uma emergência.

O distance floor adiciona uma **condição de piso de distância**: se o alvo está dentro de uma zona crítica E se está fechando, o sistema não permite de-escalação abaixo do nível correspondente, mesmo que o TTC suba.

### Thresholds de distância (de `aeb_config.h`)

| Parâmetro | Valor | Nível mínimo mantido |
|-----------|:-----:|:--------------------:|
| `D_BRAKE_L1` | 20 m | BRAKE_L1 (2,0 m/s²) |
| `D_BRAKE_L2` | 10 m | BRAKE_L2 (4,0 m/s²) |
| `D_BRAKE_L3` | 5 m | BRAKE_L3 (6,0 m/s²) |

### Condição de aplicação

O distance floor é aplicado **apenas quando `is_closing = 1`** (alvo se aproximando). Quando `is_closing = 0` (alvo se afastando ou velocidade relativa <= 0), o floor é desabilitado — isso evita que o ego continue frenando desnecessariamente quando o alvo já está se afastando (ex.: cenário CCRm com alvo em movimento).

### Referências normativas

| Fonte | Trecho relevante |
|-------|-----------------|
| **UNECE Regulation No. 152** | sec. 5.2.2 — distâncias mínimas de ativação de AEBS por faixa de velocidade |
| **ISO 22839:2013** | Clause 6.3 — *Forward Vehicle Collision Warning Systems: functional requirements* |
| **LPB — Lowest Possible Braking** | Princípio de que a intervenção deve ser a mínima necessária para evitar colisão |

---

## POST_BRAKE: estado de manutenção pós-parada

### Requisito funcional

**FR-BRK-005:** Após o veículo atingir v_ego < 0,01 m/s enquanto em frenagem ativa (`BRAKE_L1`, `BRAKE_L2` ou `BRAKE_L3`), o sistema deve manter pressão de freio equivalente a `DECEL_L3 = 6 m/s²` por `POST_BRAKE_HOLD = 2,0 s`.

### Implementação (de `aeb_fsm.c`)

```c
case AEB_POST_BRAKE:
    if (s_state_timer >= POST_BRAKE_HOLD)   /* 2.0 s */
    {
        s_state       = AEB_STANDBY;
        s_state_timer = 0.0F;
    }
    break;
```

Com saída:
```c
case AEB_POST_BRAKE:
    out.target_decel  = DECEL_L3;   /* 6.0 m/s^2 */
    out.alert_visual  = 1U;         /* luzes de freio ativas */
    out.alert_audible = 0U;         /* sonoro desligado */
    out.brake_active  = 1U;
    break;
```

### Justificativa dos 2,0 s de hold

| Fator | Valor | Fonte |
|-------|-------|-------|
| Tempo de reação do motorista surpreso | 1,2-2,0 s | SAE J2944 |
| Tempo exigido por UNECE 152 | >= 1,8 s | UNECE Reg. 152, sec. 5.2.4 |
| Gradiente de via máximo considerado | 5% (~0,5 m/s²) | Euro NCAP CCR |
| Margem sobre requisito UNECE | 11% (2,0 vs 1,8 s) | Escolha de projeto |

### Override em POST_BRAKE

Apenas o **acelerador** pode tirar o sistema de POST_BRAKE antes dos 2,0 s — o motorista decide conscientemente retomar o movimento. Pedal de freio e volante **não** são overrides em POST_BRAKE (o veículo já está parado; o motorista pressionar o freio é redundante).

---

## WARNING_TO_BRAKE_MIN: garantia de alerta antes da frenagem

### Requisito

**FR-ALR-003:** Antes de qualquer transição para estado BRAKE, o sistema deve ter permanecido em WARNING por pelo menos `WARNING_TO_BRAKE_MIN = 0,8 s`.

### Implementação (de `aeb_fsm.c`)

```c
case AEB_WARNING:
    s_warning_accum += dt;   /* acumula tempo em WARNING */

    if (desired == AEB_STANDBY) {
        /* De-escalação com histerese de 0.2 s */
        s_debounce_timer += dt;
        if (s_debounce_timer >= HYSTERESIS_TIME)
            transition_to(AEB_STANDBY);
    } else {
        s_debounce_timer = 0.0F;
        /* Escala para braking SOMENTE após 0.8 s em WARNING */
        if (s_warning_accum >= WARNING_TO_BRAKE_MIN) {
            if (desired == AEB_BRAKE_L3)
                transition_to(AEB_BRAKE_L3);
            else if (desired == AEB_BRAKE_L2)
                transition_to(AEB_BRAKE_L2);
            else if (desired == AEB_BRAKE_L1)
                transition_to(AEB_BRAKE_L1);
        }
    }
    break;
```

### Justificativa dos 0,8 s

- **Evitabilidade:** A 50 km/h, 0,8 s cobre ~11 m — suficiente para o motorista perceber e iniciar frenagem própria
- **Prevenção de falso positivo:** Se o TTC cair brevemente (1-3 ciclos) por ruído de sensor, o timer de 0,8 s (80 ciclos) não será atingido
- **Euro NCAP 2024:** Pontuação máxima exige alerta precedendo frenagem autônoma em >= 700 ms. O valor de 0,8 s proporciona 14% de margem

---

## Diagrama de transição de estados

```
              +------------------------------------------------------------+
              |          OVERRIDES GLOBAIS (prioridade máxima)              |
              |  [1] fault                        -> OFF                   |
              |  [2] v fora de [2.78, 16.67] m/s  -> STANDBY*             |
              |  [3] brake_pedal OU |theta| > 5   -> STANDBY              |
              |  [4] accel_pedal (só POST_BRAKE)  -> STANDBY              |
              |  (* com exceções: v_ego->0 em brake -> POST_BRAKE;        |
              |     close-range closing -> mantém distance floor)          |
              +------------------------------------------------------------+

  +--------------------------------------------------------------------+
  |                              OFF (0)                                |
  |  fault ativo / sistema desabilitado via InfoCenter                  |
  +------------------------------+-------------------------------------+
                                 | ~fault
                                 v
  +--------------------------------------------------------------------+
  |                           STANDBY (1)                               |
  |  v_ego in [2.78, 16.67] m/s, sem ameaça ativa                     |
  +----------+---------------------------------------------------------+
             | desired >= WARNING + is_closing
             | (TTC <= 4.0 s OU d_brake >= distance OU d <= 20 m)
             v
  +-------------------------------------------------+
  |                  WARNING (2)                     |  <-- BRAKE_L1
  |  visual=1, audible=1, brake=0                   |      (de-escalacao
  |  warning_timer acumula a cada ciclo             |       x 0.2 s)
  +---------+---------------------------------------+
            | desired >= BRAKE_L1 + warning_timer >= 0.8 s
            v
  +-------------------------------------------------+
  |                BRAKE_L1 (3)                      |  <-- BRAKE_L2
  |  decel=2.0 m/s2, brake=1, visual=1, audible=1  |      (de-escalacao
  +---------+---------------------------------------+       x 0.2 s)
            | desired >= BRAKE_L2 (imediato)
            v
  +-------------------------------------------------+
  |                BRAKE_L2 (4)                      |  <-- BRAKE_L3
  |  decel=4.0 m/s2, brake=1, visual=1, audible=1  |      (de-escalacao
  +---------+---------------------------------------+       x 0.2 s)
            | desired = BRAKE_L3 (imediato)
            v
  +-------------------------------------------------+
  |                BRAKE_L3 (5)                      |
  |  decel=6.0 m/s2, brake=1, visual=1, audible=1  |
  +---------+---------------------------------------+
            | v_ego < 0.01 m/s (qualquer BRAKE_Lx)
            | OU desired <= WARNING x 0.2 s (de L3)
            v
  +-------------------------------------------------+
  |              POST_BRAKE (6)                      |
  |  decel=6.0 m/s2, brake=1, visual=1, audible=0  |
  |  Mantém por 2.0 s, depois -> STANDBY            |
  |  Override: apenas acelerador -> STANDBY          |
  +-------------------------------------------------+
```

### Legenda do diagrama

| Seta | Tipo | Debounce |
|------|------|:--------:|
| Escalacao | Imediata | 0 ms (exceto WARNING->BRAKE: 800 ms min.) |
| De-escalacao | Com histerese | 200 ms |
| Override | Imediata | 0 ms |

---

## Modelo Stateflow / Simulink

O arquivo `AEB_Stateflow_build.m` gera um modelo Stateflow fielmente reproduzindo o comportamento de `aeb_fsm.c`. Correções aplicadas em relação a versões anteriores:

1. **V_EGO_MIN = 2,78 m/s (10 km/h)** — corrigido per FR-DEC-009 (SRS v3); Simulink usa valor correto; código C pendente de atualização
2. **STANDBY -> WARNING first** — toda ameaça entra em WARNING antes de BRAKE
3. **Input `is_closing`** — sem frenagem para alvos se afastando
4. **Distance floors** — D_BRAKE_L1=20m, D_BRAKE_L2=10m, D_BRAKE_L3=5m
5. **De-escalação single-step** — L3->L2, L2->L1, L1->WARNING
6. **POST_BRAKE `alert_visual = false`** — FR-ALR-005 removido no SRS v3; ambos alertas desligados em POST\_BRAKE
7. **Override diferenciado** — freio/volante durante frenagem; acelerador apenas em POST_BRAKE

O modelo aceita 10 entradas (`ttc`, `d_brake`, `distance`, `v_ego`, `v_rel`, `is_closing`, `brake_pedal`, `steering_angle`, `accel_pedal`, `fault`) e produz 5 saídas (`fsm_state`, `target_decel`, `alert_visual`, `alert_audible`, `brake_active`).

---

*Última atualização: março de 2026*

*Próxima seção recomendada: [Controlador PID](Controlador-PID.md)*
