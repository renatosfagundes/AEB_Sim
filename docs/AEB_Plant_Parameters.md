# Justificativa dos Parâmetros do Modelo da Planta AEB

Este documento descreve e justifica os parâmetros utilizados no modelo Simulink da planta (AEB_Plant.slx) para simulação do sistema de Frenagem Autônoma de Emergência.

---

## 1. Parâmetros do Veículo

### 1.1 Massa do Veículo

| Parâmetro | Valor | Unidade |
|-----------|:-----:|---------|
| `VEHICLE_MASS` | 1500 | kg |

**Justificativa:** Valor representativo de um sedã compacto típico utilizado em testes Euro NCAP (classe VW Golf, Hyundai i30, Opel Astra). A massa de teste inclui o veículo em ordem de marcha com 75 kg de carga representando o condutor. A faixa típica para veículos desta classe é 1300–1500 kg. O valor de 1500 kg está alinhado com o utilizado no MATLAB AEB Test Bench da MathWorks e com o artigo de referência MDPI Applied Sciences (2023).

**Fonte:** Euro NCAP AEB C2C Test Protocol v4.3; MathWorks AEB Test Bench.

### 1.2 Aerodinâmica

| Parâmetro | Valor | Unidade |
|-----------|:-----:|---------|
| `Cd` | 0,30 | - |
| `A_FRONTAL` | 2,25 | m² |
| `RHO_AIR` | 1,225 | kg/m³ |

**Justificativa:**
- **Cd = 0,30:** Coeficiente de arrasto típico de sedãs compactos modernos (faixa 0,25–0,35). O valor 0,30 é conservador e representativo da média do segmento.
- **A = 2,25 m²:** Área frontal típica de sedãs compactos (faixa 2,10–2,50 m²).
- **ρ = 1,225 kg/m³:** Densidade do ar ao nível do mar a 15°C (condição padrão ISA).

**Impacto no modelo:** A força de arrasto aerodinâmico é F_drag = 0,5 × Cd × A × ρ × v². A 50 km/h (13,89 m/s), F_drag ≈ 40 N, o que corresponde a uma desaceleração de apenas 0,027 m/s². O arrasto tem impacto marginal nos cenários AEB (velocidades baixas, eventos curtos), mas é incluído por completude do modelo.

**Fonte:** Wikipedia - Automobile Drag Coefficient; x-engineer.org.

### 1.3 Resistência ao Rolamento

| Parâmetro | Valor | Unidade |
|-----------|:-----:|---------|
| `Crr` | 0,012 | - |

**Justificativa:** Coeficiente de resistência ao rolamento para pneus de passageiros em asfalto seco a velocidades moderadas (40–60 km/h). A faixa SAE J2452 para pneus de passageiros é 0,006–0,015 dependendo de velocidade, pressão e temperatura. O valor 0,012 é conservador e representativo de condições de teste normais.

**Impacto no modelo:** A desaceleração por rolamento é Crr × g = 0,012 × 9,81 = 0,118 m/s². Contribuição pequena mas não desprezível — um veículo a 50 km/h levaria ~118 s para parar apenas por rolamento.

**Fonte:** SAE J2452; x-engineer.org Rolling Resistance.

### 1.4 Inclinação da Via

| Parâmetro | Valor | Unidade |
|-----------|:-----:|---------|
| `ROAD_GRADE` | 0 | % |

**Justificativa:** Testes Euro NCAP AEB são realizados em superfícies planas e niveladas. O parâmetro é configurável para análise de robustez (rampa de até ±6% pode ser testada). A inclinação zero é a condição de referência (baseline).

**Fonte:** Euro NCAP AEB C2C Test Protocol v4.3.

---

## 2. Parâmetros do Atuador de Freio

### 2.1 Modelo do Atuador

O atuador de freio é modelado como um sistema de 1ª ordem com atraso de transporte (dead time):

```
G(s) = e^(-s × dead_time) × 1/(τs + 1)
```

| Parâmetro | Valor | Unidade |
|-----------|:-----:|---------|
| `BRAKE_DEAD_TIME` | 30 | ms |
| `BRAKE_TAU` | 50 | ms |
| `BRAKE_MAX_DECEL` | 10,0 | m/s² |
| `BRAKE_BAR_MAX` | 10,0 | bar |

**Justificativa:**

- **Dead time = 30 ms:** Representa o atraso hidráulico entre o comando elétrico e o início do aumento de pressão no cilindro de freio. Em sistemas modernos com ESC/I-Booster (como o Bosch iBooster), o tempo morto é tipicamente 30–100 ms. O valor de 30 ms é otimista mas compatível com sistemas de frenagem de última geração utilizados em veículos com AEB. O `aeb_config.h` define `BRAKE_DEAD_TIME = 0.03f`.

- **τ = 50 ms:** Constante de tempo de pressurização. Determina a rapidez com que a pressão atinge o valor comandado após o dead time. Sistemas ESC/I-Booster alcançam 0→100 bar em ~100–120 ms, o que é compatível com τ = 50 ms (tempo para atingir 63% em 50 ms, 95% em ~150 ms). O `aeb_config.h` define `BRAKE_TAU = 0.05f`.

- **Desaceleração máxima = 10 m/s²:** Capacidade máxima do sistema de freios. Um veículo de passageiros moderno em asfalto seco (μ = 0,85) pode atingir desacelerações de 8–10 m/s². O valor de 10 m/s² representa a capacidade total do atuador (limite mecânico), embora o AEB aplique no máximo 6 m/s² (BRAKE_L3). O `aeb_config.h` define `BRAKE_MAX_DECEL = 10.0f`.

- **Mapeamento pressão-desaceleração:** O modelo assume mapeamento linear: 0 bar → 0 m/s², 10 bar → 10 m/s². Simplificação adequada para simulação SIL — um modelo de produção incluiria a curva não-linear pastilha/disco/caliper.

**Fonte:** MDPI Actuators 2022 - Brake System Modeling; Bosch iBooster datasheet; aeb_config.h.

### 2.2 Resposta Temporal do Atuador

Com dead_time = 30 ms e τ = 50 ms:

| Métrica | Tempo | Descrição |
|---------|:-----:|-----------|
| Início da resposta | 30 ms | Após o dead time, pressão começa a subir |
| 63% do valor final | 80 ms | dead_time + τ |
| 90% do valor final | 145 ms | dead_time + 2,3τ |
| 95% do valor final | 180 ms | dead_time + 3τ |

Para um comando de BRAKE_L3 (6 m/s²):
- Desaceleração de 5,7 m/s² (95%) atingida em ~180 ms
- A 50 km/h, isso corresponde a ~2,5 m percorridos durante o transitório

---

## 3. Coeficiente de Atrito Pneu-Estrada

| Parâmetro | Valor | Unidade |
|-----------|:-----:|---------|
| μ (implícito via `BRAKE_MAX_DECEL`) | 0,85 | - |

**Justificativa:** O coeficiente de atrito pico em asfalto seco para pneus de passageiros é tipicamente 0,80–0,91, com valor médio de 0,85. O atrito pico ocorre a ~20% de escorregamento do pneu. O valor é implícito no modelo — a desaceleração máxima é limitada por `BRAKE_MAX_DECEL` que incorpora μ × g ≈ 0,85 × 9,81 ≈ 8,3 m/s² mais a contribuição do arrasto e rolamento.

**Nota:** Em condições de chuva (μ ≈ 0,5) ou neve (μ ≈ 0,2), a desaceleração máxima seria significativamente menor. Estes cenários estão fora do escopo atual mas podem ser simulados reduzindo `BRAKE_MAX_DECEL`.

**Fonte:** Taylor & Francis - Tire-Road Friction Estimation (2014); MDPI Machines 2022.

---

## 4. Parâmetros dos Cenários

### 4.1 Cenários Euro NCAP

Os cenários são derivados dos protocolos Euro NCAP AEB Car-to-Car:

| Cenário | v_ego | v_target | Gap | Decel. alvo | t_brake |
|---------|:-----:|:--------:|:---:|:-----------:|:-------:|
| **CCRs 20** | 20 km/h | 0 | 40 m | 0 | - |
| **CCRs 30** | 30 km/h | 0 | 60 m | 0 | - |
| **CCRs 40** | 40 km/h | 0 | 80 m | 0 | - |
| **CCRs 50** | 50 km/h | 0 | 100 m | 0 | - |
| **CCRm** | 50 km/h | 20 km/h | 100 m | 0 | - |
| **CCRb** | 50 km/h | 50 km/h | 40 m | -2 m/s² | 3 s |
| **CCRb hard** | 50 km/h | 50 km/h | 40 m | -6 m/s² | 3 s |

**Justificativa:**
- **CCRs (Stationary):** Cenário mais comum e crítico — representa 80% dos acidentes traseiros urbanos. A faixa 20-50 km/h cobre o espectro exigido pela UNECE R152 (10-60 km/h).
- **CCRm (Moving):** Simula o veículo-alvo mais lento na mesma faixa. A diferença de velocidade (30 km/h = 8,33 m/s) é representativa de cenários de congestionamento.
- **CCRb (Braking):** O cenário mais desafiador — ambos os veículos trafegam à mesma velocidade e o alvo freia abruptamente. A desaceleração de -2 m/s² é o baseline Euro NCAP; -6 m/s² é o teste extremo.

### 4.2 Distância Inicial (Gap)

A distância inicial é escolhida para que o TTC no início do cenário seja ~7 s, garantindo que:
1. O sistema tenha tempo de detectar, alertar e frenar
2. O cenário represente condições realistas de tráfego
3. Todos os estados da FSM sejam exercitados (STANDBY → WARNING → BRAKE_L1/L2/L3)

Para CCRs 50 km/h: gap = 100 m, TTC_inicial = 100/13,89 ≈ 7,2 s.

**Fonte:** Euro NCAP AEB C2C Test Protocol v4.3; UNECE Regulation No. 152 §5.2.

---

## 5. Parâmetros de Simulação

| Parâmetro | Valor | Justificativa |
|-----------|:-----:|---------------|
| `SIM_STOP_TIME` | 15 s | Suficiente para os cenários mais longos (CCRs 50 km/h) |
| `SIM_DT` | 1 ms | Passo fixo de 1 ms garante resolução adequada para o atuador de freio (τ = 50 ms) e atende Nyquist para o ciclo de controle de 10 ms |
| Solver | ode4 (Runge-Kutta) | Solver de passo fixo de 4ª ordem — preciso e determinístico, adequado para validação SIL |

---

## 6. Equações do Modelo

### 6.1 Dinâmica Longitudinal do Ego

A equação de movimento longitudinal do veículo ego:

```
m × a_ego = -F_brake - F_drag - F_roll - F_grade
```

Onde:
- F_brake = brake_decel × m (desaceleração comandada pelo AEB)
- F_drag = 0,5 × Cd × A × ρ × v_ego² (arrasto aerodinâmico)
- F_roll = Crr × m × g (resistência ao rolamento)
- F_grade = m × g × sin(arctan(grade/100)) (componente gravitacional)

Dividindo por m:

```
a_ego = -brake_decel - (0,5 × Cd × A × ρ / m) × v_ego² - Crr × g - g × sin(arctan(grade/100))
```

### 6.2 Integração

```
v_ego(t) = ∫ a_ego dt,  com v_ego ≥ 0 (veículo não anda para trás)
x_ego(t) = ∫ v_ego dt
```

### 6.3 Cinemática Relativa

```
distance(t) = x_target(t) - x_ego(t),  com distance ≥ 0
v_rel(t)    = v_ego(t) - v_target(t)
TTC(t)      = distance(t) / v_rel(t),   quando v_rel > 0
```

### 6.4 Perfil do Veículo-Alvo

```
Se t < t_brake:
    v_target(t) = v_target_0  (velocidade constante)
Senão:
    v_target(t) = max(0, v_target_0 + decel_target × (t - t_brake))
```

---

## 7. Simplificações e Limitações

| Simplificação | Justificativa | Impacto |
|---------------|---------------|---------|
| Modelo 1D (longitudinal apenas) | AEB opera apenas no eixo longitudinal — controle lateral está fora do escopo | Nenhum para cenários CCR em linha reta |
| Atrito constante (μ = 0,85) | Testes Euro NCAP realizados em asfalto seco | Cenários de chuva/neve requerem ajuste de `BRAKE_MAX_DECEL` |
| Mapeamento linear pressão-desaceleração | Simplifica a curva não-linear do sistema de freios | Aceitável para SIL; modelo de produção usaria lookup table |
| Sem transferência de carga | Simplifica dinâmica de pitch/roll | Impacto < 5% na desaceleração para este nível de fidelidade |
| Sem modelo de pneu (Magic Formula) | Complexidade desnecessária para SIL acadêmico | Pneu é implícito via limite de desaceleração |
| Resistência ao rolamento constante | Na faixa 5-60 km/h, variação é < 10% | Aceitável para cenários AEB de curta duração |

---

*Última atualização: março de 2026*
*Referência: modeling/AEB_Plant_build.m, c_embedded/include/aeb_config.h*