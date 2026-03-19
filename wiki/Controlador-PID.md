# Controlador PID

> **Página:** Controlador PID (`aeb_pid.c`)
> **Relacionado:** [Home](Home.md) | [Módulos C Embarcado](Modulos-C-Embarcado.md) | [Máquina de Estados](Maquina-de-Estados.md)

---

## Visão geral

O módulo `aeb_pid.c` implementa um controlador **PI** (Proporcional-Integral) com as seguintes extensões de segurança:

1. **Anti-windup** no integrador — limita o termo integral a [0, 50] para evitar saturação indesejada
2. **Limitador de jerk** — limita a variação máxima de saída por ciclo (atende FR-BRK-001)
3. **Reset automático** quando a FSM não está em estado de frenagem — previne windup entre eventos
4. **Saída saturada** em [0, 100]% de pressão de freio

O controlador recebe como entrada a `target_decel` (m/s²) determinada pela FSM e retorna `brake_pct` (%) a ser enviado ao atuador. Em simulação, a `actual_decel` é fixada em 0 por razões documentadas abaixo.

---

## Lei de controle

### Formulação completa

```
error(t)    = target_decel(t) - actual_decel(t)

P_term(t)   = KP × error(t)

I_term(t)   = I_term(t-1) + KI × error(t) × dt
              [clampado em [0, 50] — anti-windup]

raw_output  = P_term(t) + I_term(t)

delta_max   = MAX_JERK × (100 / BRAKE_MAX_DECEL) × dt
            = 100.0 × (100 / 10) × 0.01
            = 10.0  [%/ciclo]

output(t)   = clamp(output(t-1) + clamp(raw_output - output(t-1),
                                        -delta_max, +delta_max),
                    0.0, 100.0)
```

Onde:
- `KP = 10.0`
- `KI = 0.05`
- `dt = 0.01 s` (ciclo de 10 ms)
- `BRAKE_MAX_DECEL = 10.0 m/s²` (desaceleração máxima física, usada para normalização)
- `MAX_JERK = 100.0 %/s`

### Implementação em C

```c
static float s_integral    = 0.0f;
static float s_prev_output = 0.0f;

float aeb_pid_update(float target_decel, uint8_t brake_active)
{
    float output = 0.0f;

    /* Reset quando FSM não está em estado de frenagem */
    if (brake_active == 0U) {
        s_integral    = 0.0f;
        s_prev_output = 0.0f;
        return 0.0f;
    }

    /* --- Erro de controle --- */
    /* actual_decel = 0.0f em simulação (veja seção "Desafios de simulação") */
    const float actual_decel = 0.0f;
    float error = target_decel - actual_decel;

    /* --- Termo proporcional --- */
    float p_term = AEB_PID_KP * error;      /* 10.0 × error */

    /* --- Termo integral com anti-windup --- */
    s_integral += AEB_PID_KI * error * 0.01f;   /* KI × error × dt */
    if (s_integral > 50.0f) { s_integral = 50.0f; }
    if (s_integral < 0.0f)  { s_integral = 0.0f;  }

    /* --- Saída bruta --- */
    float raw_output = p_term + s_integral;

    /* --- Limitador de jerk: delta máximo por ciclo --- */
    float delta_max = AEB_MAX_JERK * (100.0f / AEB_BRAKE_MAX_DECEL) * 0.01f;
    /* delta_max = 100.0 × 10.0 × 0.01 = 10.0 %/ciclo */

    float delta = raw_output - s_prev_output;
    if (delta >  delta_max) { delta =  delta_max; }
    if (delta < -delta_max) { delta = -delta_max; }
    output = s_prev_output + delta;

    /* --- Saturação final [0, 100]% --- */
    if (output > 100.0f) { output = 100.0f; }
    if (output <   0.0f) { output =   0.0f; }

    s_prev_output = output;
    return output;
}
```

---

## Ganhos atuais: KP = 10,0 e KI = 0,05

### Justificativa do KP = 10,0

Com `actual_decel = 0` em simulação (veja próxima seção), o controlador opera essencialmente em malha aberta:

```
error = target_decel - 0 = target_decel

P_term = KP × target_decel = 10.0 × target_decel

Para DECEL_L1 = 2.0 m/s²:  P_term = 10.0 × 2.0 = 20%  de freio
Para DECEL_L2 = 4.0 m/s²:  P_term = 10.0 × 4.0 = 40%  de freio
Para DECEL_L3 = 6.0 m/s²:  P_term = 10.0 × 6.0 = 60%  de freio
```

O valor KP = 10,0 foi calibrado para que **em regime permanente em malha aberta, 100% de freio corresponda à desaceleração máxima física** de 10 m/s²:

```
output_max = KP × DECEL_FISICA_MAX = 10.0 × 10.0 = 100%
```

Esta é a chamada **calibração de ganho por mapeamento físico**: o ganho proporcional é definido como o recíproco da sensibilidade do atuador, de forma que a saída do controlador já é uma estimativa razoável da pressão necessária sem depender do integrador para corrigir erros de regime.

### Papel do KI = 0,05

Com KP = 10 e `actual_decel = 0`, o erro é constante ao longo do tempo enquanto o estado FSM não muda. O integrador, neste caso, acumula gradualmente:

```
A cada ciclo (0.01 s): ΔI = KI × error × dt = 0.05 × error × 0.01
Para DECEL_L3 = 6.0:  ΔI = 0.05 × 6.0 × 0.01 = 0.003 %/ciclo
```

O integral alcança 50% (limite do anti-windup) após:
```
t = 50 / (0.003 / 0.01) = 50 / 0.3 = ~167 s
```

Na prática, o cenário completo de frenagem dura menos de 5 s — o integrador contribui com menos de 1,5% no total. Isso é intencional: **o integrador não é o mecanismo principal de controle nesta fase de simulação**, mas está presente para corrigir pequenos erros de regime quando `actual_decel` real estiver disponível em hardware.

---

## Por que `actual_decel = 0` em simulação

### O problema com odometria Gazebo

A desaceleração real do veículo seria derivada da velocidade do ego:

```
actual_decel(t) = (v_ego(t-1) - v_ego(t)) / dt
```

No Gazebo, a velocidade é obtida via `/ego_vehicle/odom` (integração de posição da simulação física). Esta odometria tem ruído de alta frequência característico do passo de tempo do simulador (tipicamente ~1 ms), que ao ser derivado produz picos de aceleração aparente de ±50 m/s² por ciclo — completamente inutilizável como realimentação de controle.

### Alternativas consideradas e rejeitadas

| Alternativa | Razão da rejeição |
|-------------|-------------------|
| Filtro passa-baixa na odometria | Introduz atraso de fase significativo (50–100 ms) na malha fechada, comprometendo a estabilidade |
| IMU virtual do Gazebo | Mesmos problemas de ruído derivativo; IMU real teria filtro de hardware ausente na simulação |
| Velocidade de roda (encoders) | Não implementado no modelo Gazebo deste projeto |
| Estimativa via modelo de planta | Aumentaria a complexidade fora do escopo ASIL-B atual |

### Implicações em hardware real

Em uma ECU de produção, `actual_decel` seria fornecida por:
- **Sensor de desaceleração longitudinal** (acelerômetro MEMS no CG do veículo, ex.: Bosch MM5.10)
- **Estimativa via ABS**: velocidade angular das rodas → velocidade linear → derivada filtrada

Com `actual_decel` real disponível, a malha fechada com KP = 10 e KI = 0,05 precisaria de re-tuning (tipicamente KP reduziria para 2–4 e KI aumentaria para 0,5–2,0 para compensar a dinâmica da planta).

---

## Limitador de jerk

### Definição e cálculo

O *jerk* (variação da aceleração) é a derivada temporal da desaceleração. Variações abruptas de pressão de freio causam:
- Desconforto para os ocupantes
- Risco de bloqueio de rodas em superfícies com baixo coeficiente de atrito
- Danos a componentes mecânicos do sistema de freio

**Requisito FR-BRK-001:** A variação máxima de desaceleração por ciclo deve ser ≤ 2 m/s²/ciclo.

O limitador de jerk é implementado como uma **limitação de taxa de variação da saída** (`brake_pct`):

```
delta_max [%/ciclo] = MAX_JERK [%/s] × (100% / BRAKE_MAX_DECEL [m/s²]) × dt [s]

Com MAX_JERK = 100 %/s, BRAKE_MAX_DECEL = 10 m/s², dt = 0.01 s:
delta_max = 100.0 × (100/10) × 0.01 = 100.0 × 10.0 × 0.01 = 10.0 %/ciclo
```

**Interpretação física:**
- 10 %/ciclo de `brake_pct` corresponde a 1 m/s²/ciclo de desaceleração (dado o mapeamento 10% → 1 m/s²)
- Portanto, delta_max = 10 %/ciclo satisfaz FR-BRK-001 (≤ 2 m/s²/ciclo) com margem de 100%

### O que o limitador de jerk previne

Sem limitador, a resposta ao degrau de `BRAKE_L3 = 6 m/s²` seria:

```
Ciclo 1: raw_output = 10 × 6 = 60%  → delta de 60% em 1 ciclo → jerk de 60 m/s²/s
```

Com o limitador ativo, a rampa para 60% ocorre em 6 ciclos (60 ms):

```
Ciclo 1:  0%  + 10% = 10%
Ciclo 2: 10%  + 10% = 20%
Ciclo 3: 20%  + 10% = 30%
Ciclo 4: 30%  + 10% = 40%
Ciclo 5: 40%  + 10% = 50%
Ciclo 6: 50%  + 10% = 60%  ← regime permanente para DECEL_L3
```

60 ms para atingir pressão de freio máxima é fisicamente razoável e imperceptível como atraso no contexto de um TTC de 1,8 s.

---

## Mapeamento pressão de freio → desaceleração

A correspondência entre `brake_pct` e desaceleração efetiva do veículo segue o mapeamento linear simplificado:

```
desaceleração [m/s²] = brake_pct [%] × 0.1

Exemplos:
  10%  → 1.0 m/s²
  20%  → 2.0 m/s²  (DECEL_L1)
  40%  → 4.0 m/s²  (DECEL_L2)
  60%  → 6.0 m/s²  (DECEL_L3)
 100%  → 10.0 m/s² (máximo físico)
```

Em termos de pressão hidráulica:

```
pressão [bar] = brake_pct [%] × 0.1

0%   → 0.0 bar  (sem freio)
60%  → 6.0 bar  (frenagem de emergência L3)
100% → 10.0 bar (pressão máxima do sistema)
```

Este mapeamento é uma simplificação; sistemas reais têm curvas de ganho não lineares (a relação pressão → força de frenagem → desaceleração depende da geometria do disco, coeficiente de atrito do pastilha, temperatura, etc.). A curva linear é adequada para validação em simulação.

---

## Reset do integrador (prevenção de windup inter-eventos)

```c
if (brake_active == 0U) {
    s_integral    = 0.0f;
    s_prev_output = 0.0f;
    return 0.0f;
}
```

O reset ocorre em **todos os ciclos em que `brake_active = 0`**, ou seja:
- Estados `OFF`, `STANDBY`, `WARNING_L1`, `WARNING_L2`
- Qualquer transição de de-escalação que saia de um estado BRAKE

**Por que o reset é crítico:** Sem reset, o integrador acumularia erro durante um evento de frenagem. Se o veículo então desacelera mas não para (alvo desvia), a FSM retorna para STANDBY, mas o integrador ainda contém 50% de carga. No próximo evento de frenagem, a saída inicial seria 50% (proveniente do integrador residual) em vez de 0%, causando uma frenagem brusca inesperada desde o primeiro ciclo.

O reset do `s_prev_output` garante que o limitador de jerk também opere a partir de 0 no próximo evento, mantendo a rampa de subida controlada independentemente do histórico anterior.

---

## Histórico de tuning: problemas com ganhos originais

### Problema 1: KP = 4,0 (ganho original)

O ganho inicial de KP = 4 produzia saída insuficiente para DECEL_L3:

```
P_term = KP × target_decel = 4.0 × 6.0 = 24%  ← apenas 24% de freio para L3
```

Com 24% de pressão (2,4 bar), a desaceleração real seria de apenas ~2,4 m/s² — menos que o alvo de 6 m/s². O integrador levaria muito tempo para compensar esse déficit de 33,6% (considerando `actual_decel = 0`). **O veículo não conseguia parar antes da colisão em cenários CCR-s a 50 km/h.**

Diagnóstico no log de simulação: pressão de freio estabilizando em ~30–35% mesmo com DECEL_L3 ativo, resultando em colisões em 3 de 5 testes CCR-s.

**Correção:** KP elevado para 10,0, garantindo que o termo proporcional sozinho já entregue os 60% necessários para L3.

### Problema 2: MAX_JERK = 10 %/s (valor original)

O valor inicial de MAX_JERK = 10 %/s resultava em:

```
delta_max = 10.0 × (100/10) × 0.01 = 10.0 × 10.0 × 0.01 = 1.0 %/ciclo
```

Com apenas 1%/ciclo de variação máxima, o tempo para atingir 60% de freio era:

```
t = 60% / (1%/ciclo × 100 ciclos/s) = 60 / 1 = 60 s
```

**60 segundos para atingir pressão máxima** — completamente inaceitável. Em um cenário de TTC = 1,8 s, o freio chegaria a apenas ~1,8%×100 = 1,8% de pressão antes da colisão.

Diagnóstico: pressão de freio crescendo linearmente a ~1%/ciclo, nunca atingindo regime para evitar colisão.

**Correção:** MAX_JERK elevado para 100 %/s, resultando em delta_max = 10 %/ciclo e tempo de rampa de apenas 60 ms para 60%.

### Resumo do histórico de tuning

| Parâmetro | Valor original | Problema | Valor corrigido | Resultado |
|-----------|:--------------:|----------|:---------------:|-----------|
| `KP` | 4.0 | 24% de freio para L3 → desaceleração insuficiente | **10.0** | 60% de freio para L3 → parada antes da colisão |
| `MAX_JERK` | 10 %/s | 1%/ciclo → 60 s para pressão máxima | **100 %/s** | 10%/ciclo → 60 ms para pressão máxima |
| `KI` | 0.05 | Adequado; sem alteração | 0.05 | Integrador complementar, sem windup em 5 s |

---

## Tabela de estado completa do PID por estado FSM

| Estado FSM | `brake_active` | `target_decel` | `brake_pct` regime | Integrador |
|-----------|:--------------:|:--------------:|:------------------:|:----------:|
| OFF | 0 | — | 0% | Reset |
| STANDBY | 0 | — | 0% | Reset |
| WARNING_L1 | 0 | — | 0% | Reset |
| WARNING_L2 | 0 | — | 0% | Reset |
| BRAKE_L1 | 1 | 2.0 m/s² | ~20% (rampa 2 ciclos) | Ativo |
| BRAKE_L2 | 1 | 4.0 m/s² | ~40% (rampa 4 ciclos) | Ativo |
| BRAKE_L2 (L3) | 1 | 6.0 m/s² | ~60% (rampa 6 ciclos) | Ativo |
| POST_BRAKE | 1 | 6.0 m/s² | ~60% (mantido) | Ativo |

---

## Análise de estabilidade (malha aberta)

Com `actual_decel = 0` constante e `target_decel` constante por bloco (determinado pela FSM), o sistema opera como **integrador com saturação**. A função de transferência discreta de malha aberta é:

```
G(z) = KP + KI × dt / (z - 1)
     = 10.0 + 0.0005 / (z - 1)
```

O polo em z = 1 (integrador) garante erro de regime nulo para referências constantes, mas **na prática `actual_decel = 0` torna o "erro" sempre igual a `target_decel`** — o sistema não fecha a malha. O PID opera como um gerador de rampa saturada, o que é funcionalmente adequado para a simulação atual.

Para fechamento de malha real (com `actual_decel` medida), a análise de estabilidade deve ser refeita com o modelo de planta do veículo (massa + sistema de freio hidráulico), que introduz um polo adicional em torno de 5–20 Hz dependendo do sistema de freio.

---

*Fim da documentação do Controlador PID*

*Voltar para: [Home](Home.md) | [Máquina de Estados](Maquina-de-Estados.md)*
