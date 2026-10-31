# Femgram — Garry's Mod Player Model

Proyecto 3D de Femgram basado **únicamente en la última ilustración proporcionada**. La primera ilustración queda descartada.

## Diseño aprobado

- Coneja antropomórfica furry, pelaje blanco.
- Silueta más alta y estilizada; muslos gruesos y redondeados, cintura moderada y abdomen sin volumen excesivo.
- Orejas largas con interior negro; cabello blanco abundante y flequillo que cubre un ojo.
- Un ojo visible negro, nariz/hocico pequeño.
- Suéter gris holgado con cuello doblado.
- Shorts negros deportivos con dos líneas blancas laterales.
- Manos/patas furry y piernas digitígradas con pies de conejo.
- Estética anime/cartoon monocromática.

## Build desde Android

1. Entra en **Actions**.
2. Selecciona **Build Femgram Blender Model**.
3. Pulsa **Run workflow** y confirma la rama `main`.
4. Cuando finalice, abre el run y descarga **Femgram-3D-Source**.

El artefacto incluye:
- `Femgram.blend`: escena editable de Blender.
- `Femgram.fbx`: geometría y armature base para intercambio.

## Estado técnico — importante

La generación actual es un **blockout procedural inicial**, no un Player Model terminado. El armature contiene nombres ValveBiped de referencia, pero aún falta el skinning/weight painting, la revisión de proporciones y animaciones. FBX/BLEND no se pueden instalar directamente en Garry's Mod.

Para llegar a un addon real se necesita:
1. Ajustar y revisar la malla en Blender.
2. Asignar pesos de vértices al esqueleto ValveBiped y comprobar deformaciones.
3. Preparar animaciones y QC.
4. Exportar SMD/DMX y compilar con las herramientas Source/Crowbar.
5. Crear materiales VMT/VTF y probar el addon dentro de Garry's Mod.

No se afirmará que el Player Model está listo hasta que exista el modelo Source compilado y se pruebe en el juego.
