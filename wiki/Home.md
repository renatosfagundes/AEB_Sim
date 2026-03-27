# AEB System — Wiki Principal

> **Projeto:** Sistema de Frenagem Autônoma de Emergência (AEB)
> **Vínculo institucional:** Residência Tecnológica Stellantis / UFPE
> **Repositório:** `AEB/modeling/`

---

## O que é este projeto?

Este projeto implementa um sistema completo de **Frenagem Autônoma de Emergência** (*Autonomous Emergency Braking* — AEB) para veículos de passeio. O desenvolvimento foi realizado no âmbito da residência tecnológica em parceria entre a **Stellantis** e a **Universidade Federal de Pernambuco (UFPE)**, com o objetivo de estudar, modelar e validar um sistema AEB em conformidade com os requisitos da indústria automotiva.

O sistema é capaz de detectar obstáculos à frente do veículo, calcular o *Time-to-Collision* (TTC), escalonar níveis de alerta e frenagem, e parar o veículo de forma autônoma antes de uma colisão — tudo isso dentro de um ciclo determinístico de **10 ms**.

### Motivação e contexto

Sistemas AEB são exigidos por protocolos de segurança como o **Euro NCAP** (cenário CCR — *Car-to-Car Rear*) e regulamentações como a **UNECE Regulation No. 152 (AEBS)**. A ISO 26262 classifica este tipo de sistema como **ASIL-B**, impondo requisitos rigorosos de confiabilidade e rastreabilidade de requisitos.

Este projeto serve como plataforma educacional e de prototipagem, reproduzindo fielmente as práticas de engenharia de uma ECU automotiva de produção: código C embarcado com conformidade MISRA, máquina de estados formal, controlador PID com limitador de jerk, e validação em simulação Gazebo via ROS2.

---

## Visão geral da arquitetura em 3 camadas

O sistema é organizado em três camadas independentes, cada uma com responsabilidades bem definidas:

```mermaid
flowchart TB
    C3["**CAMADA 3** — Simulação Gazebo / ROS2\nperception_node · scenario_controller · dashboard_node\nSensores virtuais (radar + lidar), mundo físico simulado"]
    C2["**CAMADA 2** — Wrapper ROS2 C++ (aeb_controller_node)\nPublica mensagens CAN, chama aeb_cycle_10ms() a 100 Hz\nSincroniza dados de radar/ego, guarda de prontidão"]
    C1["**CAMADA 1** — Núcleo C Embarcado (MISRA C:2012)\naeb_main · aeb_perception · aeb_ttc · aeb_fsm · aeb_pid · aeb_alert\nCiclo determinístico de 10 ms, sem alocação dinâmica"]

    C3 -->|"tópicos ROS2"| C2
    C2 -->|"chamada C direta"| C1
```

A separação entre as camadas garante que o **núcleo de lógica de segurança** (Camada 1) seja completamente independente do middleware de comunicação (Camada 2) e do ambiente de simulação (Camada 3). Em um produto real, a Camada 1 seria compilada diretamente para o microcontrolador da ECU, enquanto as Camadas 2 e 3 seriam substituídas por drivers de hardware e um veículo real.

---

## Navegação rápida

| Página | Conteúdo |
|--------|----------|
| [Home](Home.md) | Esta página — visão geral do projeto |
| [Arquitetura do Sistema](Arquitetura-do-Sistema.md) | Design em 3 camadas, fluxo de dados, filosofia de projeto |
| [Módulos C Embarcado](Modulos-C-Embarcado.md) | Documentação detalhada de cada módulo C (percepção, TTC, FSM, PID, alerta, CAN, UDS) |
| [Máquina de Estados](Maquina-de-Estados.md) | Os 7 estados, regras de transição, histerese, POST_BRAKE |
| [Controlador PID](Controlador-PID.md) | Lei de controle, ganhos, limitador de jerk, diagnóstico de tuning |
| [Barramento CAN](Barramento-CAN.md) | Mensagens DBC, CAN IDs, CRC, alive counter |
| [Diagnósticos UDS](Diagnosticos-UDS.md) | Serviços ISO 14229, DTCs, Security Access LFSR-32, calibração via WriteDataByIdentifier |
| [Análise de Requisitos](Analise-de-Requisitos.md) | Rastreabilidade FR/NFR ↔ código C e modelos Simulink (SRS v3) |
| [Cenários de Teste](Cenarios-de-Teste.md) | CCRs, CCRm, CCRb — parâmetros e critérios de aceitação |

---

## Como começar

1. Consulte o arquivo [`README.md`](../README.md) na raiz do projeto para instruções completas de configuração do ambiente.
2. Instale as dependências: **ROS2 Humble**, **Gazebo Classic**, **colcon**, compilador C99 compatível com MISRA.
3. Compile o workspace: `colcon build --symlink-install`
4. Lance a simulação: `ros2 launch gazebo_sim aeb_scenario.launch.py`
5. Monitore o dashboard: `ros2 run gazebo_sim dashboard_node`

---

## Especificações técnicas principais

| Parâmetro | Valor |
|-----------|-------|
| **Estados da FSM** | 7 (OFF, STANDBY, WARNING, BRAKE_L1, BRAKE_L2, BRAKE_L3, POST_BRAKE) |
| **Período de ciclo** | 10 ms (100 Hz) |
| **Padrão de codificação** | MISRA C:2012 |
| **Nível de integridade funcional** | ISO 26262 ASIL-B |
| **Cenário de validação** | Euro NCAP CCR (*Car-to-Car Rear stationary/moving*) |
| **Middleware de comunicação** | ROS2 Humble (LTS) |
| **Simulador físico** | Gazebo Classic 11 |
| **Velocidade mínima de ativação** | 2,78 m/s (10 km/h) |
| **Velocidade máxima de ativação** | 16,67 m/s (60 km/h) |
| **Faixa de detecção** | 0,5 m a 300 m |
| **Comunicação de atuação** | CAN bus (simulado via tópicos ROS2) |

---

## Requisitos funcionais de alto nível (SRS v3)

| ID | Requisito | Status |
|----|-----------|--------|
| FR-PER-001 | Validar dados do sensor a cada ciclo de 10 ms | ✅ |
| FR-DEC-001 | TTC = d/v\_rel quando v\_rel > 0,5 m/s; saturado em 10,0 s | ✅ |
| FR-DEC-002 | d\_brake = v²/(2 × 6 m/s²) (DECEL\_L3 = 6 m/s²) | ✅ |
| FR-DEC-009 | Sistema ativo somente para v\_ego ∈ [2,78; 16,67] m/s (10–60 km/h) | ⚠️ C: 1,39 m/s → atualizar |
| FR-BRK-003 | Capacidade mínima de frenagem ≥ 5 m/s² (sistema alvo: 6 m/s²) | ✅ |
| FR-BRK-004 | Jerk longitudinal ≤ 10 m/s³ (Simulink: 1%/ciclo = 6 m/s³ ✓) | ⚠️ C: revisar |
| FR-BRK-005 | Manter frenagem por 2 s após v\_ego < 0,01 m/s (POST\_BRAKE) | ✅ |
| FR-ALR-003 | Alerta precede frenagem autônoma em ≥ 0,8 s | ✅ |
| FR-CAN-001 | Transmitir dados do ego via CAN a cada 10 ms | ✅ |
| FR-UDS-001 | ReadDataByIdentifier (SID 0x22) para TTC, FSM state, brake pressure | ✅ |
| FR-UDS-002 | Detecção e armazenamento de DTCs (C1001, C1004, C1006) com debounce 3 ciclos | ✅ |

---

## Estrutura de diretórios do repositório

```
AEB/modeling/
├── c_embedded/          # Camada 1: núcleo C (MISRA C:2012)
│   ├── include/         # aeb_config.h, aeb_types.h, headers dos módulos
│   └── src/             # aeb_main.c, aeb_perception.c, aeb_ttc.c,
│                        # aeb_fsm.c, aeb_pid.c, aeb_alert.c,
│                        # aeb_can.c, aeb_uds.c
├── gazebo_sim/          # Camada 2+3: nós ROS2 e mundos Gazebo
│   ├── src/             # aeb_controller_node.cpp, perception_node.cpp,
│   │                    # scenario_controller.cpp, dashboard_node.cpp
│   └── worlds/          # Cenários Gazebo (.world)
├── can/                 # Definições de mensagens CAN (DBC)
├── diagrams/            # Diagramas UML, FSM, fluxos de dados
├── docs/                # Documentação técnica e referências normativas
├── results/             # Logs de simulação e métricas de validação
├── python_sil/          # Software-in-the-Loop em Python para testes rápidos
├── report_plant/        # Relatório MIL — modelo da planta (AEB_Plant.slx)
├── report_controller/   # Relatório MIL — controlador (AEB_Controller.slx, Stateflow, PID)
├── report_uds/          # Relatório MIL — diagnósticos UDS (AEB_UDS.slx)
├── report_can/          # Relatório MIL — barramento CAN (AEB_CAN.slx)
├── report_perception/   # Relatório MIL — percepção e validação de sensores
└── wiki/                # Esta wiki
```

---

## Documentos de referência

| Documento | Arquivo | Descrição |
|-----------|---------|-----------|
| SRS v3 | `AEB_SRS_v3.tex/.pdf` | Especificação de requisitos atualizada (correções Excel, FR-CAN, FR-UDS) |
| Modeling Document v1 | `AEB_Modeling_Document_v1.tex/.pdf` | Rastreabilidade MIL ↔ requisitos, 23 design decisions |
| Relatório do Controlador | `report_controller/relatorio_controlador.pdf` | MIL: FSM Stateflow, PID, TTC, alertas |
| Relatório UDS | `report_uds/relatorio_uds.pdf` | MIL: AEB_UDS.slx, Security Access LFSR-32, DTCs |
| Relatório Planta | `report_plant/relatorio_planta.pdf` | MIL: AEB_Plant.slx, modelo do veículo |

---

*Última atualização: março de 2026 — Residência Stellantis/UFPE — SRS v3 aplicada*
