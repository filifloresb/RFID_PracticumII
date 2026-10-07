% SIMULACIÓN RFID 3D CON SEGUIMIENTO DE UN TAG
% Laboratorio de robótica
%
% Lectora: Zebra FX9600
% Cable: 5D-FB
% Antenas: Hopeland
% Tag: aproximación mediante dipolo
% Propagación: ray tracing SBR
%
% Archivos necesarios:
%   LabRobotica_ASM_materiales.glb
%   HopelandA2090.m
%   Patron_Hopeland_A2090.txt
%
% IMPORTANTE:
% Esta simulación calcula potencia recibida por el tag.
% Superar su sensibilidad evalúa el enlace de ida.
% No garantiza una lectura RFID completa.
%
% Estados del seguimiento:
%   -2 = Pendiente de confirmación
%   -1 = Zona ambigua
%    0 = Sin detección
%    1 = Zona 1
%    2 = Zona 2
%    3 = Zona 3

clear;
clc;
close all;

%% ---------------- CONFIGURACIÓN GENERAL ----------------

PTX_DBM = 30;            % Potencia configurada de la lectora
INCLINACION = -45;        % Elevación de las antenas, grados

f = 915e6;               % Frecuencia, Hz

LONG_CABLE = 4;          % Longitud del cable, metros
L_CABLE = 0.202 * LONG_CABLE;

EIRP_MAX = 36;           % Techo EIRP del modelo, dBm
SENS_TAG = -22;          % Sensibilidad supuesta del tag, dBm
PERD_POL = 3;            % Pérdida adicional supuesta, dB
L_CAL = 0;               % Pérdida residual fija, dB

MAX_REFLEXIONES = 2;

ESCENA = "LabRobotica_ASM_materiales.glb";

MOSTRAR_LOBULOS = false;
TAM_LOBULO = 5;          % Escala visual; no es alcance de lectura

% Dibujar rayos en cada muestra puede hacer lenta la visualización.
MOSTRAR_RAYOS_AL_FINAL = false;

%% ---------------- CONFIGURACIÓN DEL SEGUIMIENTO ----------------

DT = 1;                  % Paso de tiempo SIMULADO, segundos

MARGEN_ZONA_DB = 3;       % Diferencia mínima entre grupos, dB
N_CONFIRMACION = 3;      % Muestras consecutivas para confirmar

% Asociación según orientación de las antenas
GRUPOS = {
    [1 3]
    [2 4 5 7]
    [6 8]
};

ETIQUETAS = [
    "Pendiente"
    "Ambigua"
    "Sin detección"
    "Zona 1"
    "Zona 2"
    "Zona 3"
];

assert(DT > 0, "DT debe ser positivo.");

assert( ...
    N_CONFIRMACION >= 1 && ...
    N_CONFIRMACION == floor(N_CONFIRMACION), ...
    "N_CONFIRMACION debe ser un entero positivo.");

assert(MARGEN_ZONA_DB >= 0, ...
    "MARGEN_ZONA_DB no puede ser negativo.");

%% ---------------- POSICIONES DE LAS ANTENAS ----------------

% Esquina de referencia sobre la superficie superior del piso
ESQUINA = [-1.526583 3.208591 -4.776402];

% Cuatro puntos de montaje, con dos antenas por punto
PUNTOS_MONTAJE = [
    ESQUINA + [1.35 2.65 2.65]
    ESQUINA + [6.40 2.65 2.65]
    ESQUINA + [1.35 6.27 2.65]
    ESQUINA + [6.40 6.27 2.65]
];

SEP_TOTAL = 0.10;

nPares = size(PUNTOS_MONTAJE, 1);
nAnt = 2 * nPares;

ANT_POS = zeros(nAnt, 3);
ANT_ANG = zeros(nAnt, 2);

for j = 1:nPares

    filas = (2*j - 1):(2*j);

    ANT_POS(filas,:) = PUNTOS_MONTAJE(j,:) + [
        0 -SEP_TOTAL/2 0
        0  SEP_TOTAL/2 0
    ];

    % Una antena hacia -Y y otra hacia +Y.
    % Supone que el lóbulo principal del modelo está hacia +X local.
    ANT_ANG(filas,:) = [
        -90 INCLINACION
         90 INCLINACION
    ];
end

assert(nAnt == 8, "Esta configuración requiere ocho antenas.");

%% ---------------- PUNTOS DE REFERENCIA DEL TAG ----------------

% Zona 1: punto de tu ensayo anterior
PUNTO_EQUIPO_1 = [2.550000 3.400000 -3.978576];
P1 = PUNTO_EQUIPO_1 + [0 0 0.010];

% Zona 2: centro entre los cuatro pares, a 1 m del piso
P2 = ESQUINA + [3.875 4.460 1.00];

% Zona 3: punto sobre el multímetro
PUNTO_EQUIPO_3 = [0.56741791 13.92596315 -3.79173534];
NORMAL_EQUIPO_3 = [0 0.05233596 0.99862953];

P3 = PUNTO_EQUIPO_3 + 0.010 * NORMAL_EQUIPO_3;

% Orientación constante del tag durante todo el recorrido
TAG_ANG = [0 0];

%% ---------------- RECORRIDO PROGRAMADO ----------------

% Tiempo:
%   0-10 s: permanece en zona 1
%  10-15 s: se mueve hacia zona 2
%  15-30 s: permanece en zona 2
%  30-35 s: se mueve hacia zona 3
%  35-45 s: permanece en zona 3
%  45-55 s: regresa hacia zona 1
%  55-65 s: permanece en zona 1
%
% ADVERTENCIA:
% Se interpolan líneas rectas entre puntos.
% Esta ruta no evita paredes ni equipos.
% Añade puntos y tiempos intermedios para recorrer los pasillos.

TIEMPOS_RUTA = [
     0
    10
    15
    30
    35
    45
    55
    65
];

PUNTOS_RUTA = [
    P1
    P1
    P2
    P2
    P3
    P3
    P1
    P1
];

assert(size(PUNTOS_RUTA,1) == numel(TIEMPOS_RUTA), ...
    "Debe existir una posición por cada tiempo de la ruta.");

assert(all(diff(TIEMPOS_RUTA) > 0), ...
    "Los tiempos deben ser estrictamente crecientes.");

tiempo = (TIEMPOS_RUTA(1):DT:TIEMPOS_RUTA(end)).';

assert(abs(tiempo(end) - TIEMPOS_RUTA(end)) < 1e-9, ...
    "La duración del recorrido debe ser múltiplo de DT.");

posiciones = interp1( ...
    TIEMPOS_RUTA, PUNTOS_RUTA, tiempo, "linear");

nMuestras = numel(tiempo);

%% ---------------- LÍMITES PROVISIONALES DE ZONA ----------------

% Estos límites son una referencia geométrica de prueba.
% No definen la zona estimada por las señales.
% Sustitúyelos por las fronteras reales del laboratorio.
%
% Zona 1: Y < Y_LIMITE_12
% Zona 2: Y_LIMITE_12 <= Y < Y_LIMITE_23
% Zona 3: Y >= Y_LIMITE_23
%
% Esta referencia solo divide por Y; no verifica paredes,
% límites exteriores ni colisiones con mobiliario.

Y_LIMITE_12 = ESQUINA(2) + 2.65;
Y_LIMITE_23 = ESQUINA(2) + 6.27;

zonaReferencia = 2 * ones(nMuestras, 1);

zonaReferencia(posiciones(:,2) < Y_LIMITE_12) = 1;
zonaReferencia(posiciones(:,2) >= Y_LIMITE_23) = 3;

%% ---------------- VERIFICACIÓN DE ARCHIVOS ----------------

assert(isfile(ESCENA), ...
    "No se encuentra la escena: %s", ESCENA);

assert(~isempty(which("HopelandA2090")), ...
    "No se encuentra HopelandA2090.m en la ruta de MATLAB.");

assert(isfile("Patron_Hopeland_A2090.txt"), ...
    "No se encuentra Patron_Hopeland_A2090.txt.");

%% ---------------- ESCENA 3D ----------------

viewer = siteviewer( ...
    SceneModel=ESCENA, ...
    ShowOrigin=true);

disp(viewer.Materials);

%% ---------------- MODELOS DE ANTENA ----------------

antenaHope = HopelandA2090();

% Aproximación de antena del tag.
% No reproduce un tag comercial sobre metal.
antenaTag = design(dipole, f);

Gtx = max( ...
    pattern(antenaHope, f, -180:180, -90:90), ...
    [], "all");

Gtag = max( ...
    pattern(antenaTag, f, -180:180, -90:90), ...
    [], "all");

% Potencia en puerto después del cable y del límite EIRP
Ptx_dBm = min(PTX_DBM - L_CABLE, EIRP_MAX - Gtx);
Ptx_W = 10^((Ptx_dBm - 30)/10);

fprintf("\n----------- CONFIGURACIÓN -----------\n");
fprintf("Frecuencia: %.0f MHz\n", f/1e6);
fprintf("Potencia configurada lectora: %.2f dBm\n", PTX_DBM);
fprintf("Pérdida del cable: %.3f dB\n", L_CABLE);
fprintf("Ganancia máxima antena: %.2f dBi\n", Gtx);
fprintf("Ganancia máxima dipolo: %.2f dBi\n", Gtag);
fprintf("Potencia en puerto: %.2f dBm\n", Ptx_dBm);
fprintf("EIRP máxima: %.2f dBm\n", Ptx_dBm + Gtx);
fprintf("Elevación: %.1f grados\n", INCLINACION);
fprintf("Sensibilidad supuesta del tag: %.1f dBm\n", SENS_TAG);
fprintf("Pérdida residual L_CAL: %.2f dB\n", L_CAL);

if PTX_DBM - L_CABLE > EIRP_MAX - Gtx
    fprintf("El techo EIRP limita la potencia efectiva.\n");
end

%% ---------------- CREAR TRANSMISORES ----------------

tx = txsite.empty;

for k = 1:nAnt

    tx(k) = txsite( ...
        Name="Hopeland #" + k, ...
        CoordinateSystem="cartesian", ...
        Antenna=antenaHope, ...
        AntennaPosition=ANT_POS(k,:).', ...
        AntennaAngle=ANT_ANG(k,:).', ...
        TransmitterFrequency=f, ...
        TransmitterPower=Ptx_W);
end

%% ---------------- CREAR TAG ----------------

rx = rxsite( ...
    Name="Tag en movimiento", ...
    CoordinateSystem="cartesian", ...
    Antenna=antenaTag, ...
    AntennaPosition=posiciones(1,:).', ...
    AntennaAngle=TAG_ANG.');

show(tx);
show(rx);

POSICIONES_ANTENAS = table( ...
    (1:nAnt).', ...
    ANT_POS(:,1), ...
    ANT_POS(:,2), ...
    ANT_POS(:,3), ...
    ANT_ANG(:,1), ...
    ANT_ANG(:,2), ...
    VariableNames={ ...
        'Antena', 'X_m', 'Y_m', 'Z_m', ...
        'Azimut_deg', 'Elevacion_deg'});

disp(POSICIONES_ANTENAS);

%% ---------------- LÓBULOS OPCIONALES ----------------

if MOSTRAR_LOBULOS

    for k = 1:nAnt
        pattern(tx(k), ...
            Size=TAM_LOBULO, ...
            Transparency=0.35);
    end
end

%% ---------------- MODELO DE PROPAGACIÓN ----------------

pm = propagationModel("raytracing", ...
    CoordinateSystem="cartesian", ...
    Method="sbr", ...
    MaxNumReflections=MAX_REFLEXIONES);

%% ---------------- FIGURA DEL SEGUIMIENTO ----------------

fig = figure( ...
    Name="Seguimiento RFID del tag", ...
    Color="w");

axRuta = subplot(2,1,1);

plot3(axRuta, ...
    posiciones(:,1), ...
    posiciones(:,2), ...
    posiciones(:,3), ...
    "--", Color=[0.6 0.6 0.6]);

hold(axRuta, "on");

plot3(axRuta, ...
    PUNTOS_RUTA(:,1), ...
    PUNTOS_RUTA(:,2), ...
    PUNTOS_RUTA(:,3), ...
    "ko", MarkerFaceColor="k");

hRecorrido = plot3(axRuta, ...
    posiciones(1,1), ...
    posiciones(1,2), ...
    posiciones(1,3), ...
    "b-", LineWidth=1.5);

hTag = plot3(axRuta, ...
    posiciones(1,1), ...
    posiciones(1,2), ...
    posiciones(1,3), ...
    "ro", MarkerSize=10, MarkerFaceColor="r");

text(axRuta, P1(1), P1(2), P1(3), "  Punto zona 1");
text(axRuta, P2(1), P2(2), P2(3), "  Punto zona 2");
text(axRuta, P3(1), P3(2), P3(3), "  Punto zona 3");

grid(axRuta, "on");
axis(axRuta, "equal");
xlabel(axRuta, "X [m]");
ylabel(axRuta, "Y [m]");
zlabel(axRuta, "Z [m]");
view(axRuta, 3);

axEstado = subplot(2,1,2);

hReferencia = stairs(axEstado, ...
    tiempo, zonaReferencia, ...
    "--", Color=[0.5 0.5 0.5], LineWidth=1);

hold(axEstado, "on");

hEstado = stairs(axEstado, ...
    tiempo(1), -2, ...
    "b", LineWidth=1.5);

grid(axEstado, "on");
xlim(axEstado, [tiempo(1) tiempo(end)]);
ylim(axEstado, [-2.5 3.5]);

yticks(axEstado, -2:3);
yticklabels(axEstado, ETIQUETAS);

xlabel(axEstado, "Tiempo simulado [s]");
title(axEstado, "Zona de referencia y estado confirmado");

legend(axEstado, [hReferencia hEstado], ...
    {"Referencia provisional", "Estado confirmado"}, ...
    Location="best");

%% ---------------- RESERVAR MEMORIA ----------------

potenciasLog = nan(nMuestras, nAnt);
puntajesLog = nan(nMuestras, 3);

separacionLog = nan(nMuestras, 1);
antenaLog = zeros(nMuestras, 1);

candidataLog = zeros(nMuestras, 1);
confirmadaLog = -2 * ones(nMuestras, 1);
ultimaZonaLog = zeros(nMuestras, 1);

% Registro separado de eventos
eventoTiempo = zeros(0,1);
eventoEvidencia = zeros(0,1);
eventoZona = zeros(0,1);
eventoDescripcion = strings(0,1);

%% ---------------- ESTADO INICIAL ----------------

candidataAnterior = NaN;
repeticiones = 0;
inicioEvidencia = NaN;

ultimaZona = 0;
estadoAnterior = NaN;

fprintf("\n========== SEGUIMIENTO ==========\n");
fprintf("Duración simulada: %.1f s\n", tiempo(end)-tiempo(1));
fprintf("Paso simulado: %.1f s\n", DT);
fprintf("Confirmación: %d muestras consecutivas\n", N_CONFIRMACION);
fprintf("Margen entre grupos: %.1f dB\n\n", MARGEN_ZONA_DB);

relojReal = tic;

%% ---------------- CICLO DE MOVIMIENTO ----------------

for i = 1:nMuestras

    % 1. Actualizar posición del tag
    rx.AntennaPosition = posiciones(i,:).';

    % 2. Calcular las ocho potencias
    p = sigstrength(rx, tx, pm) - PERD_POL - L_CAL;
    p = p(:);

    assert(numel(p) == nAnt, ...
        "Se esperaba una potencia por antena.");

    % NaN se trata como enlace no disponible.
    % -Inf representa ausencia de señal en el modelo.
    p(isnan(p)) = -Inf;

    potenciasLog(i,:) = p.';

    [mejorPotencia, mejorAntena] = max(p);

    if isfinite(mejorPotencia)
        antenaLog(i) = mejorAntena;
    end

    % 3. Puntaje de cada zona: mejor enlace de su grupo
    puntajes = zeros(3,1);

    for z = 1:3
        puntajes(z) = max(p(GRUPOS{z}));
    end

    puntajesLog(i,:) = puntajes.';

    [ordenados, orden] = sort(puntajes, "descend");

    % 4. Clasificación instantánea
    if ordenados(1) < SENS_TAG

        candidata = 0;
        separacion = NaN;

    else

        separacion = ordenados(1) - ordenados(2);

        if separacion < MARGEN_ZONA_DB
            candidata = -1;
        else
            candidata = orden(1);
        end
    end

    candidataLog(i) = candidata;
    separacionLog(i) = separacion;

    % 5. Persistencia de la candidata
    if candidata ~= candidataAnterior

        candidataAnterior = candidata;
        repeticiones = 1;
        inicioEvidencia = tiempo(i);

    else

        repeticiones = repeticiones + 1;
    end

    % 6. Confirmar o registrar incertidumbre
    if candidata <= 0

        % Incertidumbre se registra inmediatamente.
        estadoActual = candidata;

    elseif repeticiones >= N_CONFIRMACION

        estadoActual = candidata;

    else

        % No se rellena con la última zona conocida.
        estadoActual = -2;
    end

    confirmadaLog(i) = estadoActual;

    % 7. Registrar eventos de cambio de estado
    if estadoActual ~= estadoAnterior

        descripcion = ETIQUETAS(estadoActual + 3);

        if estadoActual > 0

            if ultimaZona == 0

                descripcion = ...
                    "Entrada confirmada en zona " + estadoActual;

            elseif estadoActual ~= ultimaZona

                descripcion = ...
                    "Cambio de última zona conocida: " + ...
                    ultimaZona + " -> " + estadoActual;

            else

                descripcion = ...
                    "Reconfirmación de zona " + estadoActual;
            end

            evidenciaEvento = inicioEvidencia;

        else

            evidenciaEvento = NaN;
        end

        eventoTiempo(end+1,1) = tiempo(i);
        eventoEvidencia(end+1,1) = evidenciaEvento;
        eventoZona(end+1,1) = estadoActual;
        eventoDescripcion(end+1,1) = descripcion;

        fprintf("[t=%5.1f s] %s", tiempo(i), descripcion);

        if estadoActual > 0
            fprintf(" | Primera evidencia: %.1f s", inicioEvidencia);
        end

        fprintf("\n");

        estadoAnterior = estadoActual;
    end

    if estadoActual > 0
        ultimaZona = estadoActual;
    end

    ultimaZonaLog(i) = ultimaZona;

    % Registro breve de cada muestra
    fprintf( ...
        "  t=%5.1f s | Ref=%d | Candidata=%s | " + ...
        "Estado=%s | Antena=%d | Diferencia=%.2f dB\n", ...
        tiempo(i), ...
        zonaReferencia(i), ...
        ETIQUETAS(candidata + 3), ...
        ETIQUETAS(estadoActual + 3), ...
        antenaLog(i), ...
        separacion);

    % 8. Actualizar marcador en Site Viewer
    show(rx);

    % 9. Actualizar figura independiente
    if isgraphics(fig)

        set(hTag, ...
            XData=posiciones(i,1), ...
            YData=posiciones(i,2), ...
            ZData=posiciones(i,3));

        set(hRecorrido, ...
            XData=posiciones(1:i,1), ...
            YData=posiciones(1:i,2), ...
            ZData=posiciones(1:i,3));

        set(hEstado, ...
            XData=tiempo(1:i), ...
            YData=confirmadaLog(1:i));

        title(axRuta, sprintf( ...
            "t=%.1f s | %s | Última zona conocida: %d", ...
            tiempo(i), ...
            ETIQUETAS(estadoActual + 3), ...
            ultimaZona));
    end

    drawnow;
end

tiempoReal = toc(relojReal);

%% ---------------- REGISTRO POR MUESTRA ----------------

LOG = table( ...
    tiempo, ...
    posiciones(:,1), ...
    posiciones(:,2), ...
    posiciones(:,3), ...
    zonaReferencia, ...
    candidataLog, ...
    confirmadaLog, ...
    ultimaZonaLog, ...
    antenaLog, ...
    separacionLog, ...
    VariableNames={ ...
        'Tiempo_s', ...
        'X_m', ...
        'Y_m', ...
        'Z_m', ...
        'ZonaReferenciaProvisional', ...
        'ZonaCandidata', ...
        'EstadoConfirmado', ...
        'UltimaZonaConocida', ...
        'AntenaMasFuerte', ...
        'Separacion_dB'});

LOG.EtiquetaEstado = ETIQUETAS(confirmadaLog + 3);

for k = 1:nAnt
    nombre = sprintf("Prx_Antena%d_dBm", k);
    LOG.(nombre) = potenciasLog(:,k);
end

for z = 1:3
    nombre = sprintf("PuntajeZona%d_dBm", z);
    LOG.(nombre) = puntajesLog(:,z);
end

%% ---------------- TABLA DE EVENTOS ----------------

EVENTOS = table( ...
    eventoTiempo, ...
    eventoEvidencia, ...
    eventoZona, ...
    eventoDescripcion, ...
    VariableNames={ ...
        'TiempoConfirmacion_s', ...
        'PrimeraEvidencia_s', ...
        'CodigoEstado', ...
        'Descripcion'});

%% ---------------- INTERVALOS DE PERMANENCIA ----------------

% Cada estado se conserva hasta la siguiente muestra.
% La última muestra no añade tiempo después del final.
%
% La entrada se mide desde la confirmación, no se retrocede
% al instante de primera evidencia.
%
% Pendiente, ambigua y sin detección se contabilizan
% por separado de las zonas confirmadas.

inicios = [1; find(diff(confirmadaLog) ~= 0) + 1];

siguientes = [
    inicios(2:end)
    nMuestras + 1
];

entrada = tiempo(inicios);
salida = zeros(size(entrada));

for j = 1:numel(inicios)

    if siguientes(j) <= nMuestras
        salida(j) = tiempo(siguientes(j));
    else
        salida(j) = tiempo(end);
    end
end

duracion = salida - entrada;
estadoIntervalo = confirmadaLog(inicios);

INTERVALOS = table( ...
    ETIQUETAS(estadoIntervalo + 3), ...
    estadoIntervalo, ...
    entrada, ...
    salida, ...
    duracion, ...
    VariableNames={ ...
        'Estado', ...
        'CodigoEstado', ...
        'Entrada_s', ...
        'Salida_s', ...
        'Duracion_s'});

INTERVALOS = INTERVALOS(INTERVALOS.Duracion_s > 0, :);

%% ---------------- RESUMEN POR ESTADO ----------------

codigos = (-2:3).';
duracionesTotales = zeros(numel(codigos),1);

for j = 1:numel(codigos)

    seleccion = INTERVALOS.CodigoEstado == codigos(j);

    duracionesTotales(j) = sum( ...
        INTERVALOS.Duracion_s(seleccion));
end

RESUMEN = table( ...
    ETIQUETAS, ...
    codigos, ...
    duracionesTotales, ...
    VariableNames={ ...
        'Estado', 'CodigoEstado', 'TiempoTotal_s'});

fprintf("\n========== EVENTOS ==========\n");
disp(EVENTOS);

fprintf("\n========== INTERVALOS ==========\n");
disp(INTERVALOS);

fprintf("\n========== RESUMEN ==========\n");
disp(RESUMEN);

%% ---------------- EVALUACIÓN PROVISIONAL ----------------

% Ponderación por duración:
% la muestra final tiene peso cero.
pesos = [diff(tiempo); 0];
duracionTotal = sum(pesos);

aciertos = confirmadaLog == zonaReferencia;
clasificados = confirmadaLog > 0;

porcentajeAcierto = ...
    100 * sum(pesos(aciertos)) / duracionTotal;

porcentajeClasificado = ...
    100 * sum(pesos(clasificados)) / duracionTotal;

if any(clasificados & pesos > 0)

    aciertoClasificado = ...
        100 * sum(pesos(aciertos)) / ...
        sum(pesos(clasificados));

else

    aciertoClasificado = NaN;
end

fprintf("\n========== EVALUACIÓN ==========\n");
fprintf("Acierto sobre tiempo total: %.1f %%\n", ...
    porcentajeAcierto);

fprintf("Tiempo con zona confirmada: %.1f %%\n", ...
    porcentajeClasificado);

fprintf("Acierto durante tiempo clasificado: %.1f %%\n", ...
    aciertoClasificado);

fprintf("Tiempo SIMULADO: %.1f s\n", duracionTotal);
fprintf("Tiempo REAL de cálculo: %.1f s\n", tiempoReal);

fprintf( ...
    "La referencia usa límites geométricos provisionales.\n");

%% ---------------- RESULTADO DE LA ÚLTIMA POSICIÓN ----------------

ultimaPotencia = potenciasLog(end,:).';

RESULTADOS_FINALES = table( ...
    (1:nAnt).', ...
    ultimaPotencia, ...
    ultimaPotencia - SENS_TAG, ...
    ultimaPotencia >= SENS_TAG, ...
    VariableNames={ ...
        'Antena', ...
        'Prx_dBm', ...
        'Margen_dB', ...
        'SuperaUmbral'});

fprintf("\n========== ÚLTIMA POSICIÓN ==========\n");
fprintf("Tag XYZ: [%.6f %.6f %.6f] m\n", posiciones(end,:));
disp(RESULTADOS_FINALES);

%% ---------------- RAYOS OPCIONALES AL FINAL ----------------

if MOSTRAR_RAYOS_AL_FINAL

    rayos = raytrace(tx, rx, pm);

    for k = 1:numel(rayos)

        if ~isempty(rayos{k})
            plot(rayos{k});
        end
    end
end

%% ---------------- GUARDAR RESULTADOS ----------------

% Carpeta distinta por ejecución para conservar pruebas anteriores.
carpetaSalida = tempname(pwd);
mkdir(carpetaSalida);

writetable(LOG, ...
    fullfile(carpetaSalida, "log_movimiento_tag.csv"));

writetable(EVENTOS, ...
    fullfile(carpetaSalida, "eventos_tag.csv"));

writetable(INTERVALOS, ...
    fullfile(carpetaSalida, "intervalos_tag.csv"));

writetable(RESUMEN, ...
    fullfile(carpetaSalida, "resumen_permanencias.csv"));

save(fullfile(carpetaSalida, "seguimiento_tag.mat"), ...
    "LOG", ...
    "EVENTOS", ...
    "INTERVALOS", ...
    "RESUMEN", ...
    "TIEMPOS_RUTA", ...
    "PUNTOS_RUTA", ...
    "ANT_POS", ...
    "ANT_ANG", ...
    "GRUPOS", ...
    "DT", ...
    "N_CONFIRMACION", ...
    "MARGEN_ZONA_DB", ...
    "Y_LIMITE_12", ...
    "Y_LIMITE_23", ...
    "PTX_DBM", ...
    "Ptx_dBm", ...
    "f", ...
    "SENS_TAG", ...
    "PERD_POL", ...
    "L_CAL", ...
    "MAX_REFLEXIONES");

fprintf("\n========== ARCHIVOS GUARDADOS ==========\n");
fprintf("Carpeta:\n%s\n", carpetaSalida);
fprintf("\nSimulación terminada.\n");