function antena = HopelandA2090()
    % HOPELANDA2090 Genera el Gemelo Digital 3D de la antena Hopeland A2090
    % Carga el patrón base y ensambla el objeto CustomAntennaElement
    
    % 1. Leer los datos del TXT (el archivo debe estar en tu carpeta)
    T = readtable('Patron_Hopeland_A2090.txt');
    angulos = T.Angulo_deg;
    patron_dBi = T.Directividad_dBi;

    % 2. Crear Malla esférica
    az = -180:1:180;
    el = -90:1:90;
    [AZ, EL] = meshgrid(az, el);

    % 3. Sólido de revolución (Pico apuntando a 0°)
    theta_3D = acosd(cosd(EL) .* cosd(AZ)); 
    mag3D = interp1(angulos, patron_dBi, theta_3D, 'pchip', min(patron_dBi));

    % 4. Armar y devolver el objeto de antena
    antena = phased.CustomAntennaElement( ...
        'AzimuthAngles', az, ...
        'ElevationAngles', el, ...
        'MagnitudePattern', mag3D, ...
        'PhasePattern', zeros(size(mag3D)));
end