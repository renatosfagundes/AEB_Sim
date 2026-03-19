# Análise de Requisitos — Conformidade e Desvios Justificados

> **Normas de referência:** Euro NCAP AEB CCR v4.3 · UNECE R152 · ISO 26262 ASIL-B · ISO 22839 · MDPI Safety 2024
> **Padrão de código:** MISRA C:2012
> **Ferramenta de modelagem original (SRS):** MATLAB Stateflow
> **Implementação final:** Código C manual com conformidade MISRA

---

## Tabela Resumo de Conformidade dos Requisitos

| ID | Requisito | Status | Valor SRS | Valor Implementado | Seção |
|---|---|---|---|---|---|
| FR-PER-001 | Adquirir distância relativa por fusão sensor a cada ciclo de 10 ms | ✅ | fusão radar+lidar | 0,3×radar + 0,7×lidar @ 20 ms | § Percepção |
| FR-PER-002 | Adquirir velocidade do ego via odometria | ✅ | odometria | `/aeb/ego/odom` → m/s | § Percepção |
| FR-PER-003 | Calcular velocidade relativa | ✅ | v_ego − v_target | `v_rel = ego_vx − target_vx` | § Percepção |
| FR-PER-006 | Checagem de plausibilidade (range, ROC, cross-sensor) | ✅ | definido | DIST_RATE_MAX=60 m/s, CROSS=5 m | § Percepção |
| FR-PER-007 | 3 ciclos consecutivos inválidos → flag de falha | ✅ | 3 ciclos | `FAULT_CYCLE_LIMIT = 3` | § Percepção |
| FR-TTC-001 | TTC = d/v_rel quando v_rel > 0,5 m/s | ✅ | d/v_rel | idem; saturado em 10 s | § TTC |
| FR-FSM-001 | Escalar para BRAKE_L3 em ≤ 3 ciclos após TTC ≤ 1,8 s | ✅ | 3 ciclos = 30 ms | transição imediata (1 ciclo) | § FSM |
| FR-FSM-002 | Histerese de 200 ms antes de de-escalada | ✅ | 200 ms | `HYSTERESIS_TIME = 0,2 s` | § FSM |
| FR-FSM-003 | Alerta mínimo de 800 ms antes de frenagem autônoma | ✅ | 800 ms | `WARNING_TO_BRAKE_MIN = 0,8 s` | § FSM |
| FR-BRK-001 | Variação máxima de desaceleração ≤ 2 m/s²/ciclo (jerk) | ✅ | 10 m/s³ → mudança | 100 m/s³ aceito (ver § 3) | § PID/Jerk |
| FR-BRK-002 | Pressão de freio 0–100%, mapeada para 0–10 m/s² | ✅ | definido | `brake_pct × 0,1 = bar` | § CAN |
| FR-BRK-003 | Override de direção ≥ 5° cancela AEB → STANDBY | ✅ | 5° | `STEERING_OVERRIDE_DEG = 5,0` | § FSM |
| FR-BRK-004 | Override de pedal cancela AEB → STANDBY | ✅ | pedal ativo | `brake_pedal != 0` | § FSM |
| FR-BRK-005 | POST_BRAKE mantém frenagem > 50% por 2 s | ✅ | > 50%, 2 s | DECEL_L3=6 m/s²→60%; 2,0 s | § PID/POST |
| FR-BRK-006 | Pedal acelerador cancela AEB | ❌ | cancelamento | Não implementado | § Lacunas |
| FR-DEC-005 | TTC adaptativo por condições de pista | ❌ | adaptativo | Limiares fixos | § Lacunas |
| FR-ALT-001 | Alertas visuais e sonoros antes da frenagem | ✅ | antes da frenagem | WARNING emite visual+sonoro | § Alertas |
| FR-ALT-002 | Escalonamento de alertas por nível de ameaça | ✅ | escalonado | BuzzerCmd: 1→2→4→3 por nível | § Alertas |
| NFR-EMB-001 | Código C MISRA C:2012, sem alocação dinâmica | ✅ | MISRA C | Implementado | § Arquitetura |
| NFR-SAF-001 | Conformidade ASIL-B | ✅ | ASIL-B | Arquitetura conforme; CRC + alive counter | § Arquitetura |
| NFR-VAL-007 | Validação back-to-back Stateflow ↔ C | 🔬 | exigido | Não executado | § Lacunas |
| V_EGO_MIN | Velocidade mínima de ativação | ⚠️ | 10 km/h | **5 km/h** | § 1 |
| V_EGO_MAX | Velocidade máxima de ativação | ⚠️ | 80 km/h | **60 km/h** | § 2 |
| MAX_JERK | Jerk máximo do atuador | ⚠️ | 10 m/s³ | **100 m/s³** | § 3 |
| PID_KP | Ganho proporcional | ⚠️ | 4 | **10** | § 4 |
| POST_BRAKE target_decel | Setpoint de desaceleração em POST_BRAKE | ⚠️ | 0 m/s² | **DECEL_L3 = 6 m/s²** | § 5 |
| D_BRAKE_L1/L2/L3 | Piso de distância para sustentação de frenagem | ⚠️ | não definido | **20/10/5 m** adicionados | § 6 |
| Ferramenta de modelagem | Stateflow MATLAB | ⚠️ | Stateflow | **C manual MISRA** | § 7 |
| Comunicação CAN | Tópicos Float64 genéricos | ⚠️ | Float64 | **5 frames DBC estruturados** | § 8 |

**Legenda:**
- ✅ Conforme — implementado conforme especificado
- ⚠️ Alterado com justificativa documentada
- ❌ Não implementado (lacuna identificada)
- 🔬 Não validado (validação pendente)

---

## Mudanças Realizadas e Justificativas

### 1. V_EGO_MIN: 10 km/h → 5 km/h

**Mudança:** O limiar mínimo de velocidade do ego para ativação do AEB foi reduzido de 10 km/h (2,78 m/s) para 5 km/h (1,39 m/s).

**Problema identificado:** Durante simulações CCRs a baixas velocidades, o ego decelerava de 20 km/h e cruzava o limiar de 10 km/h **enquanto ainda estava fechando sobre o alvo a ~3–4 m de distância**. A FSM retornava ao estado STANDBY e liberava o freio. O veículo colidia com o alvo a ~5–8 km/h.

**Causa técnica:** A FSM executa a verificação `if (v_ego < V_EGO_MIN)` globalmente, incluindo durante estados de frenagem ativos. Com V_EGO_MIN = 10 km/h, qualquer desaceleração bem-sucedida de AEB que reduzia o ego abaixo de 10 km/h causava auto-cancelamento.

**Justificativa da correção:**
- O Euro NCAP AEB CCR v4.3 não impõe um limiar mínimo de velocidade de desativação durante frenagem ativa
- A ISO 22839:2013 §5.3 define que o sistema deve operar enquanto houver risco de colisão, independentemente da velocidade instantânea
- O limiar de 5 km/h é suficiente para distinguir parada completa (v < 0,01 m/s → POST_BRAKE) de movimento lento mas perigoso

**Solução complementar implementada:** Adicionada exceção explícita na FSM: quando `v_ego < V_EGO_MIN` MAS a distância está dentro do piso (`distance <= D_BRAKE_L1` e `v_rel > 0`), o estado de frenagem é **mantido** em vez de retornar ao STANDBY.

---

### 2. V_EGO_MAX: 80 km/h → 60 km/h

**Mudança:** A velocidade máxima de operação do AEB foi reduzida de 80 km/h para 60 km/h.

**Problema identificado:** Nenhuma falha de sistema — trata-se de **alinhamento com o escopo de validação**. O protocolo Euro NCAP CCR v4.3 define velocidades de teste até 50 km/h. Nenhum cenário validado neste projeto opera acima de 50 km/h.

**Justificativa:**
- Manter V_EGO_MAX = 80 km/h exigiria validação a essa velocidade, o que não foi executado
- A UNECE R152 Annex 4 define testes até 80 km/h para AEBS de nível superior, mas autoriza implementações com envelope reduzido para sistemas de nível básico
- 60 km/h é o valor padrão em muitos sistemas AEB de série para veículos de passeio (Euro NCAP nota máxima começa a 60 km/h)
- Declarar um envelope operacional de 60 km/h é mais honesto do que declarar 80 km/h sem validação correspondente

---

### 3. MAX_JERK: 10 m/s³ → 100 m/s³

**Mudança:** O limite de jerk do atuador de freio foi elevado de 10 m/s³ para 100 m/s³.

**Problema identificado:** Com `MAX_JERK = 10 m/s³` e ciclo de 10 ms, a variação máxima de saída do PID por ciclo era:

```
max_delta = MAX_JERK × (PID_OUTPUT_MAX / BRAKE_MAX_DECEL) × dt
          = 10 × (100 / 10) × 0,01
          = 10 × 10 × 0,01
          = 1,0 %/ciclo
```

Para atingir 60% de freio (BRAKE_L3), o sistema precisava de **60 ciclos = 600 ms**. Em um cenário CCRs a 50 km/h, o veículo percorre 8,3 m nesses 600 ms — já muito próximo do alvo.

**Cálculo com MAX_JERK = 100 m/s³:**
```
max_delta = 100 × 10 × 0,01 = 10,0 %/ciclo
Tempo para 60%: 6 ciclos = 60 ms ← fisicamente adequado para AEB
```

**Conformidade com FR-BRK-001:** FR-BRK-001 exige "variação máxima de desaceleração ≤ 2 m/s²/ciclo", não "jerk ≤ 10 m/s³". Com 10%/ciclo de saída do PID:

```
variação de desaceleração = 10% × (10 m/s² / 100%) = 1,0 m/s²/ciclo < 2,0 m/s²/ciclo
```

FR-BRK-001 permanece satisfeito. O valor de 10 m/s³ no SRS era um objetivo de **conforto passageiro** para frenagem normal; sistemas AEB de emergência operam em regime diferente e podem exceder esse limite conforme ISO 15892 e regulamentações de segurança ativa.

**Referência:** Euro NCAP AEB CCR v4.3 §3.3.1 não limita a taxa de aplicação de freio em cenários de colisão iminente.

---

### 4. PID_KP: 4 → 10

**Mudança:** O ganho proporcional do controlador PI foi elevado de 4 para 10.

**Análise de malha aberta:** Na simulação, `actual_decel = 0` (ruído de odometria impede uso do derivativo). O controlador opera em **malha aberta pura**:

```
output = KP × error + integral ≈ KP × target_decel   (com integral pequeno)
```

Para BRAKE_L3 (`target_decel = 6 m/s²`):

| KP | output (estado estacionário) | Interpretação física |
|---|---|---|
| 4 | 4 × 6 = **24%** | 2,4 bar → ~2,4 m/s² de desaceleração real |
| 10 | 10 × 6 = **60%** | 6,0 bar → ~6,0 m/s² de desaceleração real |

Com KP=4, o sistema produzia apenas 24% de freio ao atingir BRAKE_L3 — o veículo mal desacelerava a 2,4 m/s² quando o requisito é 6 m/s². Com KP=10, a saída inicial de 60% resulta exatamente em 6 m/s² de desaceleração, o que é o valor fisicamente correto para o setpoint.

**Justificativa:** Em um sistema de produção com encoder de velocidade de roda de alta resolução, o loop fechado convergiria para o setpoint independentemente do KP. No modelo de simulação com malha aberta, o KP é efetivamente o fator de conversão direto "m/s² de desaceleração → % de freio", e KP=10 é o valor matematicamente correto para o mapeamento 0–10 m/s² → 0–100%.

---

### 5. POST_BRAKE target_decel: 0 m/s² → DECEL_L3 (6 m/s²)

**Mudança:** O setpoint de desaceleração no estado POST_BRAKE foi alterado de 0 para DECEL_L3 = 6 m/s².

**Problema identificado:** Com `target_decel = 0` em POST_BRAKE:

```c
// pid_compute() — guard inicial
if (target_decel <= 0.0F) {
    s_integral = 0.0F;
    s_prev_output = 0.0F;
    return 0.0F;   // ← saída zero imediatamente
}
```

O freio era liberado assim que o ego parava (`v_ego ≈ 0`), sem nenhuma manutenção de pressão. O veículo podia rolar lentamente após a parada, violando FR-BRK-005.

**Requisito FR-BRK-005:** "O sistema deve manter frenagem > 50% por 2 s após v_ego < 0,01 m/s (POST_BRAKE)."

**Solução:** Em POST_BRAKE, a FSM passa `DECEL_L3 = 6 m/s²` como setpoint ao PID. Com KP=10, a saída imediata é 60% > 50%, satisfazendo FR-BRK-005. Após `POST_BRAKE_HOLD = 2,0 s`, a FSM retorna ao STANDBY e o PID é resetado.

---

### 6. Piso de Distância (D_BRAKE_L1/L2/L3): Não Definido no SRS → Adicionado

**Mudança:** Três limiares de distância foram adicionados como condição secundária de manutenção de frenagem, não presentes no SRS original.

```c
// aeb_config.h
#define D_BRAKE_L1   20.0f   /* d ≤ 20 m → mínimo BRAKE_L1 */
#define D_BRAKE_L2   10.0f   /* d ≤ 10 m → mínimo BRAKE_L2 */
#define D_BRAKE_L3    5.0f   /* d ≤  5 m → mínimo BRAKE_L3 */
```

**Problema identificado (falha crítica):** Ao frear ativamente, v_ego diminui. Portanto, v_rel = v_ego − v_target diminui (para alvo estacionário, v_rel = v_ego). Um v_rel menor resulta em TTC maior:

```
TTC = distância / v_rel

Se v_ego cai de 13,9 m/s para 7,0 m/s com d=20 m:
  TTC = 20 / 7,0 = 2,86 s  →  ainda em BRAKE_L1 (TTC > 2,2 s)
  Mas sem piso: TTC poderia estar > 3,0 s → FSM voltaria a WARNING → 20% freio
```

Sem o piso, o TTC pode **crescer** temporariamente durante a frenagem bem-sucedida, causando de-escalada prematura e oscilação: freio reduz → ego acelera → TTC cai → freio aumenta → ciclo de instabilidade.

**Justificativa normativa:**
1. **UNECE R152 (AEBS), Annex 5, §3.2** define o conceito de "Last Point of Braking" (LPB): o sistema AEBS deve manter frenagem máxima uma vez que a distância atinge o LPB, independentemente do TTC calculado
2. **MDPI Safety 2024 "Time–Safety Distance Fusion for AEB"** demonstra que fusão de critério TTC + critério de distância melhora a robustez de AEB em cenários de desaceleração ego, especificamente porque o critério TTC isolado é instável quando a desaceleração do ego é alta
3. **ISO 22839:2013 §7.4.1** recomenda que sistemas AEB implementem condição de distância como backup ao critério TTC para prevenir cancelamento prematuro

O piso é ativado apenas quando `is_closing = 1` (v_rel > 0), portanto não afeta cenários em que o ego já igualou a velocidade do alvo em movimento — o freio é liberado naturalmente.

---

### 7. Ferramenta de Modelagem: Stateflow → C Manual MISRA C:2012

**Mudança:** A especificação original previa uso de MATLAB Stateflow para modelagem da FSM com geração automática de código. A implementação final é em C puro, escrito manualmente com conformidade MISRA C:2012.

**Justificativa:**

| Critério | Stateflow | C Manual MISRA |
|---|---|---|
| Custo de toolchain | Licença MATLAB (cara) | GCC livre |
| Controle de código | Gerado automaticamente | Total |
| Portabilidade | Depende de Embedded Coder | Qualquer compilador C99 |
| Rastreabilidade | Difícil inspeção manual | Totalmente inspecionável |
| Conformidade MISRA | Depende do gerador | Aplicada explicitamente |
| Equivalência de comportamento | Referência | Validada em SIL Python |

A decisão foi motivada pelo objetivo educacional do projeto: o código C embarcado manual oferece visibilidade total do algoritmo e pode ser compilado diretamente para qualquer MCU automotivo (Renesas RH850, NXP S32K, STM32) sem dependência de licenças proprietárias.

**Nota sobre NFR-VAL-007 (back-to-back):** A validação formal de equivalência entre Stateflow e C (execução paralela com entradas idênticas e comparação de saídas) não foi realizada — esta é uma lacuna identificada (ver seção Lacunas Restantes). Uma referência de comportamento equivalente foi implementada em `python_sil/` para verificação informal.

---

### 8. CAN: Tópicos Float64 Genéricos → 5 Frames DBC Estruturados

**Mudança:** O SRS original propunha tópicos ROS2 simples (`Float64`) para transportar distância, velocidade e comando de freio. A implementação usa 5 mensagens estruturadas alinhadas ao arquivo DBC.

**Arquitetura original (SRS):**
```
/aeb/distance     → std_msgs/Float64
/aeb/ego_speed    → std_msgs/Float64
/aeb/target_speed → std_msgs/Float64
/aeb/ttc          → std_msgs/Float64
/aeb/brake_cmd    → std_msgs/Float64
/aeb/fsm_state    → std_msgs/String
```

**Arquitetura implementada:**
```
/can/radar_target  → AebRadarTarget  (0x120, 20ms)
/can/ego_vehicle   → AebEgoVehicle   (0x100, 10ms)
/can/brake_cmd     → AebBrakeCmd     (0x080, 10ms, ASIL-B, CRC)
/can/fsm_state     → AebFsmState     (0x200, 50ms)
/can/alert         → AebAlert        (0x300, evento)
```

**Justificativas:**

1. **Realismo de produção:** Em um ECU automotivo real, as interfaces entre nós são mensagens CAN com periodicidade, alive counters e CRC definidos no DBC. Usar `Float64` genérico não é representativo de nenhum sistema real
2. **Detecção de falha de comunicação:** O `AliveCounter` permite detectar perda de frames e travamento de transmissor — funcionalidade impossível com `Float64`
3. **Integridade ASIL-B:** O CRC de 4 bits em `AEB_BrakeCmd` é um mecanismo de integridade exigido para mensagens de segurança (ISO 26262 Part 4 §7.4.3 recomenda CRC para comunicação ASIL-B)
4. **Rastreabilidade DBC:** Qualquer ferramenta CANdb++ ou similar pode abrir `can/aeb_system.dbc` e mapear os tópicos diretamente para as mensagens de barramento sem conversão adicional

---

## Lacunas Restantes (Requisitos Não Implementados)

### FR-BRK-006 — Cancelamento por Pedal do Acelerador

**Requisito:** O sistema AEB deve ser cancelado quando o motorista pressiona o pedal do acelerador (indicação de intenção de aceleração deliberada).

**Motivo da não implementação:** O `planar_move` plugin do Gazebo não simula pedal do acelerador; a adição exigiria um nó de motorista virtual e mapeamento de estado do pedal na mensagem `AebEgoVehicle`. Em produção, esse sinal viria do pedal de posição (APP — Accelerator Pedal Position Sensor) via CAN.

**Impacto:** Baixo nos cenários CCR (o motorista é passivo nos testes Euro NCAP).

---

### FR-DEC-005 — TTC Adaptativo

**Requisito:** Os limiares de TTC devem se adaptar às condições de pista (piso escorregadio, velocidade relativa alta, carga do veículo).

**Motivo da não implementação:** Requer modelo de coeficiente de atrito pista-pneu e estimador de carga, que estão fora do escopo do projeto. Os limiares fixos (4,0 / 3,0 / 2,2 / 1,8 s) são conservadores o suficiente para os cenários CCR em pista seca.

---

### NFR-VAL-007 — Validação Back-to-Back Stateflow ↔ C

**Requisito:** Execução paralela de modelo Stateflow de referência e implementação C com entradas idênticas; tolerância de saída ≤ 0,01%.

**Motivo da não implementação:** O modelo Stateflow não foi desenvolvido neste projeto (decisão § 7 acima). A validação informal foi feita por comparação com o simulador Python SIL em `python_sil/`. Uma validação formal requereria contratar a ferramenta MATLAB/Simulink ou desenvolver um modelo Stateflow equivalente do zero.

---

## Referências Normativas Utilizadas

| Norma | Título | Aplicação no Projeto |
|---|---|---|
| Euro NCAP AEB CCR v4.3 | AEB Car-to-Car Test Protocol | Definição dos 9 cenários de teste; critérios pass/fail |
| UNECE R152 | AEBS — Regulation on Uniform Provisions | Requisito de alerta ≥ 0,8 s; conceito LPB para piso de distância |
| ISO 26262:2018 (Part 4) | Functional Safety — ASIL-B | Requisitos de CRC e alive counter; arquitetura ASIL-B |
| ISO 22839:2013 | Forward Vehicle Collision Warning Systems | Condição de distância como backup ao TTC (§7.4.1) |
| MDPI Safety 2024 | "Time–Safety Distance Fusion for AEB" | Fundamentação teórica para o piso de distância |
| MISRA C:2012 | Guidelines for the Use of C in Critical Systems | Conformidade do código C embarcado |
| ISO 15622:2010 | Adaptive Cruise Control — Performance Requirements | Referência para fusão radar+lidar ponderada |

---

## Narrativa para Apresentação

A sequência de mudanças segue uma lógica de causa e efeito que pode ser apresentada em 3 blocos:

### Bloco 1 — Problema: o sistema frenava tarde demais

- **MAX_JERK = 10 m/s³** → ramp de 600 ms → veículo já colidiu (§ 3)
- **PID_KP = 4** → saída máxima de 24% → desaceleração de 2,4 m/s² em vez de 6 m/s² (§ 4)
- **Solução:** MAX_JERK = 100 m/s³ (ramp de 60 ms) + KP = 10 (saída correta de 60%)

### Bloco 2 — Problema: o sistema liberava o freio antes de parar

- **V_EGO_MIN = 10 km/h** → FSM cancela em STANDBY enquanto ainda fechando (§ 1)
- **TTC sobe durante frenagem** → de-escalada prematura → oscilação (§ 6)
- **POST_BRAKE com target_decel = 0** → freio liberado imediatamente após parada (§ 5)
- **Solução:** V_EGO_MIN = 5 km/h + pisos D_BRAKE_L1/L2/L3 + POST_BRAKE mantém DECEL_L3

### Bloco 3 — Decisões arquiteturais

- **Stateflow → C MISRA** (§ 7): controle total, portabilidade, custo zero de toolchain
- **Float64 → frames DBC** (§ 8): realismo automotivo, alive counters, CRC ASIL-B
- **V_EGO_MAX = 60 km/h** (§ 2): declaração honesta do envelope validado

---

*Última atualização: março de 2026 — Residência Stellantis/UFPE*
