# Jugadas de demo por voz

## Objetivo

Estas frases se dicen por **micrófono** para demostrar la cadena completa del
proyecto en modo 7.3:

```
voz real → Whisper (ASR) → LLM (LangChain/HF) → validación legal (python-chess) → movimiento del brazo
```

Todas se han verificado contra el **estado inicial real del motor** (no una
partida estándar de 32 piezas). El proyecto arranca con un final reducido de
**6 piezas** definido en `src/chess_gazebo/config/board_layout.yaml`:

```
FEN inicial: 6k1/8/3b4/4p3/4P3/3B4/8/6K1 w - - 0 25
```

| Bando  | Piezas                                   |
|--------|------------------------------------------|
| Blancas | Rey **g1**, Alfil **d3**, Peón **e4**   |
| Negras  | Rey **g8**, Alfil **d6**, Peón **e5**   |

Mueven las **blancas**. Ninguno de los dos peones puede avanzar (se bloquean
mutuamente en e4/e5), así que la demo se basa en movimientos de **alfil y rey**,
sencillos y seguros para el robot, sin capturas.

## Nota sobre turnos

El sistema **actualiza el turno automáticamente** tras cada jugada aplicada. En
los logs verás, por ejemplo:

- tras una jugada blanca: `Move applied. Turn: black`;
- la siguiente orden debe ser **legal para negras**;
- después vuelve a **blancas**, y así sucesivamente.

Si pides una jugada del bando equivocado o ilegal, el motor la rechaza
(`Illegal move ...` / `Parse failed`) y **no mueve el brazo** ni cambia el turno.

## Jugada 1 — Demo principal validada

Comando de voz:

> "mueve el alfil blanco de d3 a c4"

- Turno: **blancas**
- UCI: `d3c4`
- SAN/log: `Bc4+` (da jaque al rey negro de g8 por la diagonal c4–g8)
- Pieza/casillas: alfil blanco, **d3 → c4**
- Resultado: el brazo mueve el alfil blanco de d3 a c4; la pieza sigue al gripper
  durante el transporte; `Move applied. Turn: black`.
- Estado: **validada end-to-end** con voz real, Whisper, LLM y movimiento del brazo.

## Jugada 2 — Respuesta de negras (salir del jaque)

Comando de voz:

> "mueve el rey negro de g8 a f8"

- Turno: **negras**
- UCI: `g8f8`
- SAN/log: `Kf8`
- Pieza/casillas: rey negro, **g8 → f8**
- Por qué es útil: tras `Bc4+` el rey negro **está en jaque**; las únicas jugadas
  legales son del rey (`Kf8`, `Kh8`, `Kh7`, `Kg7`). `Kf8` es la respuesta natural
  y demuestra que el motor exige una jugada legal del bando correcto.
- Estado: **verificada por el motor** (legal en la posición posterior a la jugada 1).

## Jugada 3 — Blancas reposicionan el alfil

Comando de voz:

> "mueve el alfil blanco de c4 a d5"

- Turno: **blancas**
- UCI: `c4d5`
- SAN/log: `Bd5`
- Pieza/casillas: alfil blanco, **c4 → d5**
- Por qué es útil: movimiento de alfil limpio y visible, sin jaque ni captura.
- Estado: **verificada por el motor**.

## Jugada 4 — Negras mueven el rey

Comando de voz:

> "mueve el rey negro de f8 a e7"

- Turno: **negras**
- UCI: `f8e7`
- SAN/log: `Ke7`
- Pieza/casillas: rey negro, **f8 → e7**
- Por qué es útil: cierra la secuencia alternando turnos; movimiento corto y seguro.
- Estado: **verificada por el motor**.

> Las cuatro frases son **parseables tanto por el LLM como por el regex de
> fallback**: nombran pieza, color y casillas origen/destino explícitas, así que
> aunque el LLM no esté disponible, el parser extrae las casillas y el motor
> valida la jugada.

## Recomendación para la demo

Secuencia recomendada (empezando siempre por la jugada validada):

1. "mueve el alfil blanco de d3 a c4"  → `d3c4` (`Bc4+`), turno → negras
2. "mueve el rey negro de g8 a f8"     → `g8f8` (`Kf8`),  turno → blancas
3. "mueve el alfil blanco de c4 a d5"  → `c4d5` (`Bd5`),  turno → negras
4. "mueve el rey negro de f8 a e7"     → `f8e7` (`Ke7`),  turno → blancas

Para una demo breve, basta con la **jugada 1** (la única validada con ejecución
real del brazo). Las jugadas 2–4 amplían la secuencia y están verificadas como
legales por el motor.

> **Si una jugada falla o el estado del tablero queda alterado**, reinicia la
> simulación con `chess-clean` y vuelve a lanzar la demo
> (`bash scripts/run_demo_73.sh`); el motor parte de nuevo del FEN inicial.

## Recordatorios

- **YOLO/percepción** queda como módulo **advisory/experimental**: nunca es la
  autoridad del tablero; el motor de ajedrez es la fuente de verdad.
- La **demo principal va sin visión** (`enable_vision:=false`).
- El **token de Hugging Face nunca se commitea**: usa `huggingface-cli login`.
