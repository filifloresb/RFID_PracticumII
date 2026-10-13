% SIMULACIÓN RFID 3D - LAB ROBÓTICA
clear; 
clc; 
close all;

%% ---------------- CONFIGURACIÓN ----------------

% AJUSTES ANTENAS
PTX_DBM      = 30;      % Potencia configurada de la lectora, dBm
INCLINACION  = -45;     % Inclinación de la antena
TAM_LOBULO   = 5;       % Escala de lóbulos
MOSTRAR_LOBULOS = false;
MOSTRAR_RAYOS   = true;


f = 915e6;             % Frecuencia de trabajo, Hz (Específicada en la NOM)

LONG_CABLE = 4;        % Longitud del cable por antena, m
L_CABLE = 0.202 * LONG_CABLE; %Pérdida del cable

EIRP_MAX = 36;         % Potencia radiada máxima
SENS_TAG = -22;        % Sensibilidad tag silverline blade II
PERD_POL = 3;          % Pérdida adicional por polarización, dB

% Pérdida de calibración.
L_CAL = 0;

MAX_REFLEXIONES = 2;

ESCENA = "LabRobotica_ASM_materiales.glb";

%% ---------------- POSICIÓN DE ANTENAS ----------------

% Coordenada de la esquina del laboratorio contra el origen del diseño
ESQUINA = [-1.526583 3.208591 -4.776402];

% Coordenadas de montaje del par de antenas
PUNTOS_MONTAJE = [
    ESQUINA + [1.35 2.65 2.65]
    ESQUINA + [6.40 2.65 2.65]
    ESQUINA + [1.35 6.27 2.65]
    ESQUINA + [6.40 6.27 2.65]
];

SEP_TOTAL = 0.10;      % Separación entre pares de antenas, m

nPares = size(PUNTOS_MONTAJE, 1);
nAnt = 2 * nPares;

ANT_POS = zeros(nAnt, 3);
ANT_ANG = zeros(nAnt, 2);

%Crea vector de posición de todas las antenas
for j = 1:nPares

    filas = (2*j-1):(2*j);

    % Cada antena queda a 5 cm del centro de montaje
    ANT_POS(filas,:) = PUNTOS_MONTAJE(j,:) + [
        0 -SEP_TOTAL/2 0
        0  SEP_TOTAL/2 0
    ];

    ANT_ANG(filas,:) = [
        -90 INCLINACION
         90 INCLINACION
    ];
end

%% ---------------- UBICACIÓN DEL TAG ----------------
ZONA = 1; 

if ZONA == 1
    TAG_POS = [2.550000 3.400000 -3.968576];

elseif ZONA == 2
    TAG_POS = ESQUINA + [3.875 4.460 1.00];

elseif ZONA == 3
    % Zona 3: tag sobre el multímetro, como estaba antes.
    PUNTO_EQUIPO = [0.56741791 13.92596315 -3.79173534];
    NORMAL_EQUIPO = [0 0.05233596 0.99862953];
    SEPARACION_TAG = 0.010; % m

    TAG_POS = PUNTO_EQUIPO + SEPARACION_TAG*NORMAL_EQUIPO;

end

TAG_ANG = [0 0];

fprintf("\nSimulación del tag en ZONA %d\n", ZONA);
fprintf("Tag XYZ: [%.6f %.6f %.6f] m\n", TAG_POS);



%% ---------------- ANTENAS Y TAG ----------------

antenaHopelandA0920 = HopelandA2090();

% Aproximación de la antena del tag.
% No reproduce el comportamiento de un tag comercial sobre metal.
antenaTag = design(dipole, f);

Gtx = max(pattern(antenaHopelandA0920, f, -180:180, -90:90), [], "all");

Gtag = max(pattern(antenaTag, f, -180:180, -90:90), [], "all");

% Potencia en el puerto de antena:
% descuenta el cable y aplica el techo EIRP
Ptx_dBm = min(PTX_DBM - L_CABLE, EIRP_MAX - Gtx);

Ptx_W = 10^((Ptx_dBm - 30)/10);

fprintf("\n----------- CONFIGURACIÓN -----------\n");
fprintf("Frecuencia: %.0f MHz\n", f/1e6);
fprintf("Potencia configurada lectora: %.2f dBm\n", PTX_DBM);
fprintf("Pérdida del cable: %.3f dB\n", L_CABLE);
fprintf("Ganancia máxima antena: %.2f dBi\n", Gtx);
fprintf("Ganancia máxima dipolo tag: %.2f dBi\n", Gtag);
fprintf("Potencia en puerto de antena: %.2f dBm\n", Ptx_dBm);
fprintf("EIRP máxima del escenario: %.2f dBm\n", Ptx_dBm + Gtx);
fprintf("Elevación configurada: %.1f grados\n", INCLINACION);
fprintf("Pérdida residual fija L_CAL: %.2f dB\n", L_CAL);

if PTX_DBM - L_CABLE > EIRP_MAX - Gtx
    fprintf("El techo EIRP está limitando la potencia efectiva.\n");
end

% Crear ocho sitios transmisores
tx = txsite.empty;

for k = 1:nAnt
    tx(k) = txsite( ...
        Name="Hopeland A0920 #" + k, ...
        CoordinateSystem="cartesian", ...
        Antenna=antenaHopelandA0920, ...
        AntennaPosition=ANT_POS(k,:).', ...
        AntennaAngle=ANT_ANG(k,:).', ...
        TransmitterFrequency=f, ...
        TransmitterPower=Ptx_W);
end

% Crear sitio receptor sobre el equipo
rx = rxsite( ...
    Name="Tag sobre multímetro HP34401A", ...
    CoordinateSystem="cartesian", ...
    Antenna=antenaTag, ...
    AntennaPosition=TAG_POS.', ...
    AntennaAngle=TAG_ANG.');

show(tx);
show(rx);

% Tabla para comprobar posiciones y orientaciones
POSICIONES = table( ...
    (1:nAnt).', ...
    ANT_POS(:,1), ...
    ANT_POS(:,2), ...
    ANT_POS(:,3), ...
    ANT_ANG(:,1), ...
    ANT_ANG(:,2), ...
    VariableNames={ ...
        'Antena', 'X_m', 'Y_m', 'Z_m', ...
        'Azimut_deg', 'Elevacion_deg'});

disp(POSICIONES);

fprintf("Tag XYZ: [%.6f %.6f %.6f] m\n", TAG_POS);

%% ---------------- LÓBULOS DE RADIACIÓN ----------------

% Representación de orientación y ganancia.
% TAM_LOBULO solo cambia la escala del dibujo.
% Estas superficies no son fronteras de detección.

if MOSTRAR_LOBULOS
    for k = 1:nAnt
        pattern(tx(k), ...
            Size=TAM_LOBULO, ...
            Transparency=0.35);
    end
end

%% ---------------- RAY TRACING ----------------

% Se utilizan los materiales reconocidos de la escena
pm = propagationModel("raytracing", ...
    CoordinateSystem="cartesian", ...
    Method="sbr", ...
    MaxNumReflections=MAX_REFLEXIONES);

rayos = raytrace(tx, rx, pm);

if MOSTRAR_RAYOS
    for k = 1:nAnt
        if ~isempty(rayos{k})
            plot(rayos{k});
        end
    end
end

%% ---------------- POTENCIA RECIBIDA ----------------

% Potencia estimada por enlace, con pérdidas adicionales
Prx = sigstrength(rx, tx, pm) - PERD_POL - L_CAL;
Prx = Prx(:);

MARGEN_TAG = Prx - SENS_TAG;
ACTIVACION = Prx >= SENS_TAG;

RESULTADOS = table( ...
    (1:nAnt).', ...
    Prx, ...
    MARGEN_TAG, ...
    ACTIVACION, ...
    VariableNames={ ...
        'Antena', 'Prx_dBm', 'Margen_dB', 'SuperaUmbral'});

fprintf("\n----------- RESULTADOS -----------\n");
fprintf("Sensibilidad supuesta del tag: %.1f dBm\n", SENS_TAG);

disp(RESULTADOS);

estado = ["Bajo umbral", "Supera umbral"];

for k = 1:nAnt
    fprintf("Antena %d: %7.2f dBm | Margen %7.2f dB -> %s\n", ...
        k, Prx(k), MARGEN_TAG(k), estado(ACTIVACION(k) + 1));
end

% Superar el umbral evalúa la activación en el enlace de ida.
% No garantiza una lectura RFID completa.
% Este ensayo evalúa un punto; no demuestra cobertura de todo el salón.

%% ---------------- COMPARACIÓN DE UBICACIONES ----------------
% Ejecutar después del resto del script.
% Mantiene potencia, orientación, altura y separación de cada par.

% Centros actuales, relativos a ESQUINA
% Obtener el centro de cada par de antenas ya configurado
nAnt = size(ANT_POS,1);
assert(mod(nAnt,2)==0, "Debe haber dos antenas por par.");

nPares = nAnt/2;

CENTROS_ACTUALES = ...
    (ANT_POS(1:2:end,:) + ANT_POS(2:2:end,:))/2 - ESQUINA;

% Propuesta 1: acercar ambos lados 50 cm hacia el centro en X
CENTROS_CENTRO = CENTROS_ACTUALES;
CENTROS_CENTRO([1 3],1) = CENTROS_CENTRO([1 3],1) + 0.50;
CENTROS_CENTRO([2 4],1) = CENTROS_CENTRO([2 4],1) - 0.50;

% Propuesta 2: además, mover los pares 3 y 4 hacia el extremo del tag
CENTROS_CENTRO_TAG = CENTROS_CENTRO;
CENTROS_CENTRO_TAG([3 4],2) = ...
    CENTROS_CENTRO_TAG([3 4],2) + 0.50;

ESCENARIOS = {
    CENTROS_ACTUALES
    CENTROS_CENTRO
    CENTROS_CENTRO_TAG
};

% Puntos adicionales a 1 m sobre el piso.
% Revisar visualmente que estén en espacio libre, fuera de sólidos.
PUNTOS_PRUEBA = [
    TAG_POS
    ESQUINA + [3.875 4.460 1.00]
    ESQUINA + [0.250 4.460 1.00]
    ESQUINA + [6.450 4.460 1.00]
];

NOMBRES = [
    "Tag sobre multímetro"
    "Centro del salón"
    "Extremo X bajo"
    "Extremo X alto"
];

nPruebas = size(PUNTOS_PRUEBA,1);
MEJOR_PRX = zeros(nPruebas, numel(ESCENARIOS));

% Se crean sitios de ensayo sin dibujarlos.
% Los sitios originales tx permanecen en sus posiciones.
for e = 1:numel(ESCENARIOS)

    txPrueba = txsite.empty;

    for j = 1:nPares

        centro = ESQUINA + ESCENARIOS{e}(j,:);

        posiciones = centro + [
            0 -SEP_TOTAL/2 0
            0  SEP_TOTAL/2 0
        ];

        for a = 1:2
            k = 2*j - 2 + a;

            txPrueba(k) = txsite( ...
                Name="Ensayo antena " + k, ...
                CoordinateSystem="cartesian", ...
                Antenna=antenaHopelandA0920, ...
                AntennaPosition=posiciones(a,:).', ...
                AntennaAngle=ANT_ANG(k,:).', ...
                TransmitterFrequency=f, ...
                TransmitterPower=Ptx_W);
        end
    end

    for q = 1:nPruebas

        rxPrueba = rxsite( ...
            Name=NOMBRES(q), ...
            CoordinateSystem="cartesian", ...
            Antenna=antenaTag, ...
            AntennaPosition=PUNTOS_PRUEBA(q,:).', ...
            AntennaAngle=TAG_ANG.');

        potencias = sigstrength(rxPrueba, txPrueba, pm) ...
                    - PERD_POL - L_CAL;

        % Mejor enlace individual; no suma las ocho potencias
        MEJOR_PRX(q,e) = max(potencias(:));
    end
end

COMPARACION = table( ...
    NOMBRES, ...
    MEJOR_PRX(:,1), ...
    MEJOR_PRX(:,2), ...
    MEJOR_PRX(:,3), ...
    VariableNames={ ...
        'Punto', ...
        'Actual_dBm', ...
        'HaciaCentro_dBm', ...
        'CentroYTag_dBm'});

fprintf("\n----- COMPARACIÓN DE UBICACIONES -----\n");
fprintf("Umbral supuesto: %.1f dBm\n", SENS_TAG);
disp(COMPARACION);

MARGENES = table( ...
    NOMBRES, ...
    MEJOR_PRX(:,1)-SENS_TAG, ...
    MEJOR_PRX(:,2)-SENS_TAG, ...
    MEJOR_PRX(:,3)-SENS_TAG, ...
    VariableNames={ ...
        'Punto', ...
        'MargenActual_dB', ...
        'MargenCentro_dB', ...
        'MargenCentroYTag_dB'});

disp(MARGENES);