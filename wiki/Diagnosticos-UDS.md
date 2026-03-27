# Diagnósticos UDS (ISO 14229)

> **Página:** Diagnósticos UDS
> **Relacionado:** [Home](Home.md) | [Arquitetura do Sistema](Arquitetura-do-Sistema.md) | [Barramento CAN](Barramento-CAN.md) | [Análise de Requisitos](Analise-de-Requisitos.md)

---

## Visão Geral

O módulo `aeb_uds.c` implementa um subconjunto dos serviços de diagnóstico **ISO 14229-1 (UDS — Unified Diagnostic Services)** necessários para a ECU AEB. O modelo MIL correspondente é `AEB_UDS.slx`, gerado por `AEB_UDS_build.m`.

Requisitos atendidos: FR-UDS-001 a FR-UDS-005 (SRS v3).

---

## Serviços UDS suportados

| SID | Serviço | Sessão requerida | FR |
|-----|---------|------------------|----|
| 0x10 | DiagnosticSessionControl | Qualquer | — |
| 0x11 | ECUReset | Extended / Programming | — |
| 0x14 | ClearDiagnosticInformation | Extended | FR-UDS-003 |
| 0x19 | ReadDTCInformation | Qualquer | FR-UDS-002 |
| 0x22 | ReadDataByIdentifier | Qualquer | FR-UDS-001 |
| 0x27 | SecurityAccess (LFSR-32) | Extended / Programming | DD-UDS-SEC |
| 0x2E | WriteDataByIdentifier | Programming + desbloqueado | FR-UDS-001 |
| 0x31 | RoutineControl | Extended | FR-UDS-004 |
| 0x3E | TesterPresent | Qualquer | — |

### Sessões de diagnóstico (SID 0x10)

| Sub-função | Sessão | Timeout S3 | Serviços adicionais |
|-----------|--------|-----------|---------------------|
| 0x01 | Default | — (não expira) | ReadDID, ReadDTC |
| 0x02 | Extended | 5 s | + SecurityAccess, RoutineControl |
| 0x03 | Programming | 5 s | + WriteDataByIdentifier |

O timeout S3 é o temporizador que retorna o sistema à sessão Default se nenhum `TesterPresent` for recebido dentro de 5 s. Isso previne que a ECU fique indefinidamente em modo de diagnóstico ativo.

---

## DTCs suportados

| DTC | Código | Falha | Debounce | Módulo C |
|-----|--------|-------|---------|----------|
| C1001 | Radar timeout | Sensor não responde por > 30 ms | 3 ciclos UDS (300 ms) | `aeb_perception.c` |
| C1004 | CRC error | Erro de integridade no frame CAN | 3 ciclos UDS | `aeb_can.c` |
| C1006 | Actuator fault | Atuador de freio não responde | 3 ciclos UDS | `aeb_pid.c` |

### Ciclo de vida de um DTC (FR-UDS-002)

```
Falha detectada no ciclo C (10 ms)
      │
      ▼ (debounce_counter++)
[debounce_counter < 3]
      │
      ▼ após 3 ciclos consecutivos com falha (30 ms)
DTC CONFIRMED → dtc_count++
fault_lamp = 1 (FR-UDS-005)
      │
      ▼
[ClearDiagnosticInformation SID 0x14]
      │
      ▼
Todos os DTCs zerados, fault_lamp = 0
```

**Por que 3 ciclos?** O ciclo UDS roda a 100 ms; 3 ciclos = 300 ms de falha persistente antes de confirmar o DTC. Isso evita DTCs espúrios por ruído transitório (ex.: frame CAN perdido isolado).

---

## DIDs suportados (ReadDataByIdentifier / WriteDataByIdentifier)

### ReadDataByIdentifier — SID 0x22

| DID | Descrição | Tamanho | Sessão |
|-----|-----------|---------|--------|
| 0xF100 | TTC atual (s) | float32 (4 bytes) | Qualquer |
| 0xF101 | Estado FSM atual | uint8 | Qualquer |
| 0xF102 | Pressão de freio atual (%) | float32 (4 bytes) | Qualquer |
| 0xF200 | TTC\_WARNING calibrado (s) | float32 (4 bytes) | Programming + desbloqueado |
| 0xF201 | TTC\_BRAKE\_L1 calibrado (s) | float32 (4 bytes) | Programming + desbloqueado |
| 0xF202 | Kp do PID calibrado | float32 (4 bytes) | Programming + desbloqueado |

### Valores padrão dos parâmetros calibráveis

| DID | Parâmetro | Padrão | Mínimo | Máximo |
|-----|-----------|--------|--------|--------|
| 0xF200 | TTC\_WARNING | 4,0 s | 2,0 s | 6,0 s |
| 0xF201 | TTC\_BRAKE\_L1 | 3,0 s | 1,5 s | 5,0 s |
| 0xF202 | Kp PID | 10,0 | 1,0 | 20,0 |

---

## Security Access — SID 0x27 (LFSR-32)

O acesso de escrita (WriteDataByIdentifier, RoutineControl em modo Programming) é protegido por um mecanismo seed-key conforme ISO 14229-1 Annex H.

### Algoritmo

#### Passo 1 — RequestSeed (sub-função 0x01)

A ECU gera uma semente de 32 bits usando um **LFSR Galois** (Linear Feedback Shift Register) com polinômio 0x80200003:

```c
uint32_t lfsr32(uint32_t x)
{
    if (x & 1U)
        return (x >> 1) ^ 0x80200003U;
    else
        return x >> 1;
}
```

A semente inicial é `0xABCD1234U`. A cada requisição de seed, um avanço do LFSR é executado.

#### Passo 2 — SendKey (sub-função 0x02)

O ferramenta de diagnóstico (tester) calcula a chave a partir da semente recebida:

```
masked   = seed  XOR  0xA55A3CC3
rotated  = ROTL32(masked, 7)       ← rotação circular de 7 bits à esquerda
key      = rotated + 0x12345678   (mod 2³²)
```

Em código C:

```c
uint32_t compute_key(uint32_t seed)
{
    const uint32_t SECRET_XOR = 0xA55A3CC3U;
    const uint32_t SECRET_ADD = 0x12345678U;
    uint32_t masked  = seed ^ SECRET_XOR;
    uint32_t rotated = (masked << 7) | (masked >> 25);
    return rotated + SECRET_ADD;
}
```

#### Verificação na ECU

A ECU executa o mesmo `compute_key(seed)` e compara com a chave enviada pelo tester:

| Resultado | NRC / código | Ação |
|-----------|-------------|------|
| Chave correta | SID 0x67 (PositiveResponse) | `sec_unlocked = 1` |
| Chave incorreta | NRC 0x35 `invalidKey` | Acesso negado |
| Escrita sem desbloqueio | NRC 0x33 `securityAccessDenied` | Acesso negado |

---

## RoutineControl — SID 0x31

| Routine ID | Ação | Sessão requerida |
|-----------|------|-----------------|
| 0x0301 | Enable/Disable AEB (`aeb_enabled` toggle) | Extended |
| 0x0302 | Self-test (verifica FSM, PID, sensores) | Extended |

**FR-UDS-004:** Quando `aeb_enabled = 0`, a FSM recebe este flag em Priority 1 (sobreposição global) e transiciona para OFF no ciclo seguinte.

---

## Modelo MIL — AEB_UDS.slx

O modelo Simulink `AEB_UDS.slx` implementa todos os serviços acima em um único bloco MATLAB Function (`UDS_Handler`). Cenários de simulação disponíveis via `AEB_UDS_scenarios.m`:

| Cenário | Descrição |
|---------|-----------|
| `nominal` | Sessão extended → leituras periódicas de DID → TesterPresent |
| `dtc_lifecycle` | Injeção de falha → DTC confirmado → cleared |
| `calibration` | SecurityAccess unlock → WriteDataByIdentifier TTC/Kp |
| `selftest` | RoutineControl 0x0302 antes e depois de injetar DTC |
| `sec_denied` | Tentativa sem desbloqueio → NRC 0x33 / chave errada → NRC 0x35 |
| `session_timeout` | Timeout S3 → retorno à sessão Default |

### Executar simulação

```matlab
% No MATLAB:
AEB_UDS_scenarios('dtc_lifecycle');
out = sim('AEB_UDS');
AEB_UDS_plot(out);
```

---

## Open Items

| # | Item | Prioridade |
|---|------|-----------|
| 1 | Integrar `aeb_uds.c` ao ciclo `aeb_cycle_10ms()` na Camada 1 | Alta |
| 2 | Validar back-to-back: C vs Simulink para cenário `calibration` | Alta |
| 3 | Implementar CAN TP (ISO 15765-2) para respostas multi-frame | Média |
| 4 | Adicionar DTCs para: CAN timeout (C1002), brake actuator saturação (C1007) | Média |
| 5 | Persistência de DTCs em NVM (não-volátil) entre resets | Baixa |

---

*Última atualização: março de 2026 — Residência Stellantis/UFPE*

*Voltar para: [Home](Home.md) | [Barramento CAN](Barramento-CAN.md)*
