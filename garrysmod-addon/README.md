# Femgram — Garry's Mod addon beta

Estado: **pre-beta / preparación del addon**. Este directorio documenta el paquete y el flujo de validación; todavía no contiene un Player Model compilado.

## Objetivo de la primera beta

Instalar Femgram como modelo seleccionable de jugador en Garry's Mod. La beta inicial prioriza:
- que el modelo aparezca en el menú de selección;
- que el modelo cargue sin errores de materiales;
- que el esqueleto y las animaciones de jugador funcionen;
- que no se sustituya ni sobrescriba el FBX/BLEND fuente.

## Estructura final esperada del addon

```
garrysmod/addons/femgram_player/
├── addon.json
└── models/
    └── player/
        └── femgram/
            ├── femgram.mdl
            ├── femgram.vvd
            ├── femgram.dx90.vtx
            └── femgram.phy
```

Los materiales compilados se colocarán bajo `materials/models/player/femgram/` (VMT + VTF). Los archivos de modelo indicados son salidas de compilación Source; **no** se generan renombrando el FBX.

## Requisitos antes de publicar una beta instalable

1. Revisar el rig y los pesos del modelo; el FBX optimizado de 75K es solo una prueba y no está validado para deformaciones.
2. Preparar una exportación SMD/DMX compatible con el esqueleto ValveBiped y las animaciones de jugador.
3. Crear y revisar el QC de compilación (modelo, secuencias, materiales, colisiones y rutas).
4. Compilar con herramientas Source compatibles con Garry's Mod.
5. Crear VMT/VTF a partir de las texturas PNG y verificar rutas y materiales.
6. Instalar el addon en Garry's Mod y probar selección, primera/tercera persona, animaciones, hitboxes y consola.

## Política de esta beta

- No distribuir un ZIP como si fuera instalable mientras falten los archivos compilados.
- No reemplazar el modelo fuente con `Femgram-Lite-75K.fbx` sin comparar cara, UV y deformaciones.
- Mantener separados los archivos fuente, los intermedios y el addon compilado.
- Cada artefacto de prueba debe identificarse como pre-beta hasta superar las pruebas dentro del juego.

## Instalación cuando exista el primer paquete compilado

Copiar la carpeta `femgram_player` a:

`...Steam/steamapps/common/GarrysMod/garrysmod/addons/`

Después, iniciar el juego, abrir el menú de modelo de jugador y comprobar que Femgram aparece en la lista. La ruta exacta puede variar según la instalación.
