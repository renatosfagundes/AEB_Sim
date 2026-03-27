function AEB_UDS_scenarios(scenario)
%AEB_UDS_SCENARIOS  Configura cenário de simulação para AEB_UDS.slx
%
% Uso:
%   AEB_UDS_scenarios('nominal')       % Operação normal, leituras DID periódicas
%   AEB_UDS_scenarios('dtc_lifecycle') % Injeção de falha → DTC confirmado → cleared
%   AEB_UDS_scenarios('calibration')   % SecurityAccess unlock → WriteDataByIdentifier
%   AEB_UDS_scenarios('selftest')      % RoutineControl autoteste (0x0302)
%   AEB_UDS_scenarios('sec_denied')    % Tentativa de escrita sem desbloqueio
%   AEB_UDS_scenarios('session_timeout') % Timeout S3: sessão retorna a DEFAULT

if nargin < 1, scenario = 'nominal'; end

DT   = 0.1;
TEND = 30.0;
t    = (0:DT:TEND)';
N    = length(t);

%% ── Sinais de sistema (nominais em todos os cenários) ─────────────────────
ttc_in      = 10.0 * ones(N,1);
fsm_in      =  1.0 * ones(N,1);    % STANDBY
brake_bar   =  0.0 * ones(N,1);
sens_fault  =  zeros(N,1);
crc_err     =  zeros(N,1);
alive_err   =  zeros(N,1);
timeout_err =  zeros(N,1);
act_err     =  zeros(N,1);
ecu_temp    = 25.0 * ones(N,1);

%% ── Sequência de requisições UDS ─────────────────────────────────────────
% Formato: [t, SID, B2, B3, B4, B5, B6, B7, B8]
req_frames = [];

switch scenario

  case 'nominal'
    % Sessão extended → leituras periódicas de DID → TesterPresent
    req_frames = [
        1.0,  0x10, 0x02, 0,0,0,0,0,0;   % DiagnosticSessionControl extended
        2.0,  0x22, 0xF1, 0x00, 0,0,0,0,0; % ReadDID TTC
        3.0,  0x22, 0xF1, 0x01, 0,0,0,0,0; % ReadDID FSM state
        4.0,  0x22, 0xF1, 0x02, 0,0,0,0,0; % ReadDID brake_bar
        5.0,  0x22, 0xF1, 0x03, 0,0,0,0,0; % ReadDID fault count
        6.0,  0x3E, 0x00, 0,0,0,0,0,0;     % TesterPresent
        8.0,  0x22, 0xF1, 0x00, 0,0,0,0,0; % ReadDID TTC again
       10.0,  0x3E, 0x00, 0,0,0,0,0,0;     % TesterPresent
       12.0,  0x19, 0x02, 0xFF, 0,0,0,0,0; % ReadDTC all
       15.0,  0x10, 0x01, 0,0,0,0,0,0;     % Return to DEFAULT
    ];

  case 'dtc_lifecycle'
    % t=0-4: nominal; t=5: injeção timeout; t=8: confirma DTC; t=25: clear
    timeout_err(round(5/DT)+1 : round(25/DT)) = 1;   % C1001 radar timeout
    crc_err(round(13/DT)+1 : round(14/DT)) = 1;       % C1004 CRC error
    req_frames = [
        1.0,  0x10, 0x03, 0,0,0,0,0,0;        % Programming session
        2.0,  0x19, 0x02, 0xFF, 0,0,0,0,0;    % ReadDTC before fault
        8.0,  0x19, 0x02, 0xFF, 0,0,0,0,0;    % ReadDTC after 3 debounce cycles
       14.0,  0x19, 0x02, 0xFF, 0,0,0,0,0;    % ReadDTC with C1004 also
       25.0,  0x14, 0xFF, 0xFF, 0xFF, 0,0,0;  % ClearDiagnosticInformation
       27.0,  0x19, 0x02, 0xFF, 0,0,0,0,0;    % Confirm cleared
    ];

  case 'calibration'
    % SecurityAccess unlock → WriteDID TTC warning → verify read-back
    req_frames = build_calibration_sequence();

  case 'selftest'
    % RoutineControl 0x0302 autoteste antes e depois de injetar DTC
    timeout_err(round(10/DT)+1 : round(15/DT)) = 1;  % C1001
    req_frames = [
        1.0,  0x10, 0x02, 0,0,0,0,0,0;        % Extended session
        2.0,  0x31, 0x01, 0x03, 0x02, 0,0,0;  % Self-test (expect pass)
       12.0,  0x31, 0x01, 0x03, 0x02, 0,0,0;  % Self-test (expect fail — DTC C1001)
       16.0,  0x14, 0xFF, 0xFF, 0xFF, 0,0,0;  % ClearDTC
       18.0,  0x31, 0x01, 0x03, 0x02, 0,0,0;  % Self-test (expect pass again)
    ];

  case 'sec_denied'
    % Tentativa de WriteDID sem desbloqueio → NRC 0x33 securityAccessDenied
    % E tentativa com chave errada → NRC 0x35 invalidKey
    req_frames = [
        1.0,  0x10, 0x03, 0,0,0,0,0,0;         % Programming session
        % Tenta escrever SEM fazer SecurityAccess
        2.0,  0x2E, 0xF2, 0x00, 0x40, 0x60, 0x00, 0x00, 0; % → NRC 0x33
        % Faz RequestSeed
        4.0,  0x27, 0x01, 0,0,0,0,0,0;
        % Envia chave ERRADA
        5.0,  0x27, 0x02, 0xDE, 0xAD, 0xBE, 0xEF, 0,0;     % → NRC 0x35
        % Faz RequestSeed de novo, envia chave correta (pré-calculada)
        6.0,  0x27, 0x01, 0,0,0,0,0,0;
        % chave correta adicionada pelo código abaixo
        7.0,  0x27, 0x02, 0,0,0,0,0,0;         % placeholder
        % Agora escreve com sucesso
        8.0,  0x2E, 0xF2, 0x00, 0x40, 0x60, 0x00, 0x00, 0;
    ];
    % Calcular chave para seed inicial
    seed = uint32(hex2dec('ABCD1234'));
    seed = lfsr32(seed);  % 1ª chamada
    seed = lfsr32(seed);  % 2ª chamada (t=6s é segunda RequestSeed)
    key  = compute_key(seed);
    kb   = double(typecast(key,'uint8'));
    idx  = find(req_frames(:,1) == 7.0);
    req_frames(idx, 4:7) = kb;

  case 'session_timeout'
    % Sessão extended sem TesterPresent → timeout S3 em t=5s
    req_frames = [
        1.0,  0x10, 0x02, 0,0,0,0,0,0;         % Extended session
        2.0,  0x27, 0x01, 0,0,0,0,0,0;          % RequestSeed (ok — ainda em sessão)
        % Sem TesterPresent → S3 expira em 5s após t=1s (timeout t≈6s)
        8.0,  0x22, 0xF1, 0x00, 0,0,0,0,0;     % ReadDID após timeout → deve funcionar (SID 0x22 aceito em DEFAULT)
       10.0,  0x10, 0x02, 0,0,0,0,0,0;          % Re-enter extended
       11.0,  0x3E, 0x00, 0,0,0,0,0,0;          % TesterPresent (mantém sessão)
       16.0,  0x3E, 0x00, 0,0,0,0,0,0;
    ];

  otherwise
    warning('Cenario ''%s'' nao reconhecido. Usando ''nominal''.', scenario);
    AEB_UDS_scenarios('nominal'); return;
end

%% ── Montar vetores de sinal ──────────────────────────────────────────────────
req_bytes = zeros(N, 9);
req_valid = zeros(N, 1);
for r = 1:size(req_frames,1)
    k = round(req_frames(r,1) / DT) + 1;
    if k >= 1 && k <= N
        req_bytes(k,:) = req_frames(r,2:end);
        req_valid(k)   = 1;
    end
end

%% ── Exportar workspace ───────────────────────────────────────────────────────
assignin('base','ws_ttc',       [t, ttc_in]);
assignin('base','ws_fsm',       [t, fsm_in]);
assignin('base','ws_brake_bar', [t, brake_bar]);
assignin('base','ws_sens_f',    [t, sens_fault]);
assignin('base','ws_crc_err',   [t, crc_err]);
assignin('base','ws_alive_err', [t, alive_err]);
assignin('base','ws_timeout',   [t, timeout_err]);
assignin('base','ws_act_err',   [t, act_err]);
assignin('base','ws_ecu_temp',  [t, ecu_temp]);
assignin('base','ws_req_valid', [t, req_valid]);
for col = 1:9
    assignin('base', sprintf('ws_req_b%d',col), [t, req_bytes(:,col)]);
end

fprintf('Cenario ''%s'' configurado. Execute: out = sim(''AEB_UDS'')\n', scenario);
end

%% ── Auxiliares ───────────────────────────────────────────────────────────────
function frames = build_calibration_sequence()
    % Monta sequência: unlock → write TTC_WARN=3.5s → write TTC_L1=2.8s → read-back
    seed_init = uint32(hex2dec('ABCD1234'));
    seed = lfsr32(seed_init);
    key  = compute_key(seed);
    kb   = double(typecast(key,'uint8'));

    % 3.5f = 0x40600000 em IEEE-754 LE → bytes: 00 00 60 40
    ttc35_bytes = double(typecast(single(3.5),'uint8'));
    % 2.8f = 0x40333333 em IEEE-754 LE
    ttc28_bytes = double(typecast(single(2.8),'uint8'));
    % Kp=8.0: 0x41000000
    kp8_bytes   = double(typecast(single(8.0),'uint8'));

    frames = [
        1.0,  0x10, 0x03, 0,0,0,0,0,0;         % Programming session
        2.0,  0x22, 0xF1, 0x00, 0,0,0,0,0;     % Read TTC antes
        3.0,  0x27, 0x01, 0,0,0,0,0,0;          % RequestSeed
        % SendKey correto
        4.0,  0x27, 0x02, kb(1), kb(2), kb(3), kb(4), 0, 0;
        5.0,  0x2E, 0xF2, 0x00, ttc35_bytes(1), ttc35_bytes(2), ttc35_bytes(3), ttc35_bytes(4), 0;
        6.0,  0x2E, 0xF2, 0x01, ttc28_bytes(1), ttc28_bytes(2), ttc28_bytes(3), ttc28_bytes(4), 0;
        7.0,  0x2E, 0xF2, 0x02, kp8_bytes(1),   kp8_bytes(2),   kp8_bytes(3),   kp8_bytes(4),  0;
        8.0,  0x22, 0xF1, 0x00, 0,0,0,0,0;     % Read TTC após calibração
       10.0,  0x19, 0x02, 0xFF, 0,0,0,0,0;     % Verificar DTCs (sem falhas)
    ];
end

function y = lfsr32(x)
    POLY = uint32(hex2dec('80200003'));
    if bitand(x, uint32(1))
        y = bitxor(bitshift(x,-1), POLY);
    else
        y = bitshift(x,-1);
    end
end

function key = compute_key(seed)
    SECRET_XOR = uint32(hex2dec('A55A3CC3'));
    SECRET_ADD = uint32(hex2dec('12345678'));
    masked  = bitxor(seed, SECRET_XOR);
    rotated = bitor(bitshift(masked, 7), bitshift(masked, -(32-7)));
    key     = rotated + SECRET_ADD;
end
