# SeekAndCapture

En esta práctica el robot debe:

1. Buscar a las personas a su alrededor.
2. Cuando detecta nua, debe ir hacia ella, incluso si ésta se mueve.
3. Cuando esté cerca, debe hacer alguna señal (sonido, mensaje,..)
4. El robot busca a otra persona diferente

* El software debe tener un subsistema de percepción que detecte a la persona en la imagen, transforme esa detección a una detección 3D y genere un frame desde "odom" para la detección. Usa los nodos del paquete perception_asr para esto.
* Debe haber un subsistema de actuación que, a partir de los frames de las percepciones de personas, elija una y genere los comandos para seguirla. Debe pararse a ~1 metro y hacer la señal.

Puntuación:

* Sistema de percepción funcionando correctamente [+1]
* El robot se aproxima a una persona [+2]
* El robot usa PIDs para seguir a la persona [+1]
* El robot se para y avisa cuando ha "cogido" a una persona [+1]
* El robot usa Behavior Trees para organizar su comportamiento [+2]
* El robot busca y sigue a otra persona cuando ha alcanzado a una [+2]
* Todo se lanza con un solo launcher. Tiene un sistema de depuración efectivo [+1]

La práctica se entrega el Martes 28/03/2023
