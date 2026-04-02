# Disp9
# Primera intension de uso, cualquier codigo
Control de 4 bombas que vierten liquido sobre envases de un solo uso.
Se hace sobre codigo de hace 4 anhos ayudado con IA de Perplexiti.
Se adicionan dos modulos LCD 16x2 para visualizar el proceso en cada bomba.
Se incluyen dos botones por bomba, start/stop controla ciclo normal de vertimiento segun programacion y el segundo tiene la funcion de "JOG" con su respectiva visualizacion de valor y volumen.
# Protocolo Serial – Control de Bombas

Versión firmware: **8.6**  
Formato de tramas ASCII, orientado a comandos `setX` / `getX` y eventos de estado.

---

## 1. Formato de trama

### Recepción (PC → equipo)

```text
$,ID,Orden,IDx,Valor,%
```

- `ID`: identificador del equipo (1..10 chars alfanuméricos) o `SUPER_ID`
- `Orden`: nombre del comando
- `IDx`:
  - `0` → comandos globales
  - `1..4` → número de bomba
- `Valor`: numérico o texto, según el comando

Delimitadores:
- Inicio: `$`
- Fin: `%`

### Transmisión (equipo → PC)

```text
%,ID,Orden,IDx,Valor,$
```

- `ID`: `MACHINE_ID` actual del equipo
- `Orden`: eco del comando o nombre de evento
- `IDx`: `0` o `1..4`
- `Valor`: según comando / evento

Delimitadores:
- Inicio: `%`
- Fin: `$`

---

## 2. Comandos de configuración

### 2.1 setFill

Configura los pulsos objetivo del llenado de una bomba.

- Dirección: RX/TX
- Trama RX:
  ```text
  $,ID,setFill,IDx,FillPulsos,%
  ```
- Trama TX (OK):
  ```text
  %,ID,setFill,IDx,FillPulsos,$
  ```
- Rango:
  - `IDx`: 1..4
  - `FillPulsos`: `unsigned long` (pulsos del sensor de flujo)
- Efecto:
  - Actualiza `surtidor_fill[idx]`
  - Guarda en EEPROM

### 2.2 setBill

Configura el valor monetario objetivo del llenado.

- Dirección: RX/TX
- Trama RX:
  ```text
  $,ID,setBill,IDx,ValorBill,%
  ```
- Trama TX (OK):
  ```text
  %,ID,setBill,IDx,ValorBill,$
  ```
- Rango:
  - `IDx`: 1..4
  - `ValorBill`: `unsigned long`
- Efecto:
  - Actualiza `surtidor_bill[idx]`
  - Guarda en EEPROM
  - Fuerza refresco de LCD para mostrar nueva meta

### 2.3 setTime

Configura el tiempo máximo de llenado para la bomba.

- Dirección: RX/TX
- Trama RX:
  ```text
  $,ID,setTime,IDx,TiempoMs,%
  ```
- Trama TX (OK):
  ```text
  %,ID,setTime,IDx,TiempoMs,$
  ```
- Rango:
  - `IDx`: 1..4
  - `TiempoMs`: `unsigned long` (milisegundos)
- Efecto:
  - Actualiza `surtidor_time[idx]`
  - Guarda en EEPROM

### 2.4 setCant

Configura el volumen objetivo en mL.

- Dirección: RX/TX
- Trama RX:
  ```text
  $,ID,setCant,IDx,CantMl,%
  ```
- Trama TX (OK):
  ```text
  %,ID,setCant,IDx,CantMl,$
  ```
- Rango:
  - `IDx`: 1..4
  - `CantMl`: `unsigned long` (mililitros)
- Efecto:
  - Actualiza `surtidor_cantML[idx]`
  - Guarda en EEPROM
  - Actualiza cálculo de volumen en LCD

### 2.5 setNameB

Cambia el nombre visible de la bomba.

- Dirección: RX/TX
- Trama RX:
  ```text
  $,ID,setNameB,IDx,Nombre,%
  ```
- Trama TX (OK):
  ```text
  %,ID,setNameB,IDx,1,$
  ```
- Rango:
  - `IDx`: 1..4
  - `Nombre`: texto imprimible 1..16 chars (sin `, $ %`)
- Efecto:
  - Actualiza `bombaName[idx]`
  - Guarda en EEPROM
  - Refresca LCD inmediatamente

### 2.6 setlrn

Activa/desactiva modo “aprendizaje” para el siguiente ciclo.

- Dirección: RX/TX
- Trama RX:
  ```text
  $,ID,setlrn,IDx,Flag,%
  ```
- Trama TX (OK):
  ```text
  %,ID,setlrn,IDx,Flag,$
  ```
- Rango:
  - `IDx`: 1..4
  - `Flag`: 0 (off) / 1 (on)
- Efecto:
  - Si está activo, al cerrar el siguiente ciclo la máquina:
    - Lee los pulsos reales de la bomba
    - Copia ese valor a `surtidor_fill[idx]`
    - Guarda en EEPROM
    - Emite confirmación `setlrn` con el nuevo `fill`

### 2.7 acumBill

Sobrescribe el acumulado monetario de una bomba.

- Dirección: RX/TX
- Trama RX:
  ```text
  $,ID,acumBill,IDx,ValorAcum,%
  ```
- Trama TX (OK):
  ```text
  %,ID,acumBill,IDx,ValorAcum,$
  ```

### 2.8 clrAcumBill

Limpia el acumulado monetario de una bomba.

- Dirección: RX/TX
- Trama RX:
  ```text
  $,ID,clrAcumBill,IDx,0,%
  ```
- Trama TX (OK):
  ```text
  %,ID,clrAcumBill,IDx,0,$
  ```

---

## 3. Comandos globales

### 3.1 chgID

Cambia el `MACHINE_ID` del equipo.

- Dirección: RX/TX
- Trama RX:
  ```text
  $,ID,chgID,0,NuevoID,%
  ```
- Trama TX (OK):
  ```text
  %,NuevoID,chgID,0,1,$
  ```
- Rango:
  - `IDx`: 0
  - `NuevoID`: 1..10 chars alfanuméricos
- Efecto:
  - Actualiza `MACHINE_ID`
  - Guarda en EEPROM

### 3.2 setRst

Reset por software usando el watchdog.

- Dirección: RX/TX
- Trama RX:
  ```text
  $,ID,setRst,0,1,%
  ```
- Trama TX:
  ```text
  %,ID,setRst,0,1,$
  ```
- Efecto:
  - Envía confirmación
  - Ejecuta `softwareReset()`
  - El microcontrolador se reinicia

---

## 4. Comandos de control de bomba

### 4.1 start

Activa marcha normal de una bomba (equivalente al botón START).

- Dirección: RX/TX
- Trama RX:
  ```text
  $,ID,start,IDx,1,%
  ```
- Trama TX:
  ```text
  %,ID,start,IDx,1,$
  ```
- Efecto:
  - Activa `marchaToggle[idx]` si no está en JOG
  - Si el envase está presente y la bomba está lista, inicia ciclo

### 4.2 stop

Detiene la bomba y cancela modos activos.

- Dirección: RX/TX
- Trama RX:
  ```text
  $,ID,stop,IDx,0,%
  ```
- Trama TX:
  ```text
  %,ID,stop,IDx,0,$
  ```
- Efecto:
  - Detiene la bomba
  - Anula marcha normal, JOG y learn
  - Programa tiempo de “re-arma” antes de siguiente arranque

### 4.3 jog

Controla modo JOG (manual momentáneo) por protocolo.

- Dirección: RX/TX
- Trama RX:
  ```text
  $,ID,jog,IDx,1,%
  $,ID,jog,IDx,0,%
  ```
- Trama TX:
  ```text
  %,ID,jog,IDx,1,$  // al iniciar
  %,ID,jog,IDx,0,$  // al soltar / terminar
  ```

---

## 5. Comandos de consulta

### 5.1 getFill

Devuelve el número de pulsos configurado para la bomba.

- RX:
  ```text
  $,ID,getFill,IDx,0,%
  ```
- TX:
  ```text
  %,ID,getFill,IDx,FillPulsos,$
  ```

### 5.2 getBill

Devuelve el valor monetario configurado para la bomba.

- RX:
  ```text
  $,ID,getBill,IDx,0,%
  ```
- TX:
  ```text
  %,ID,getBill,IDx,ValorBill,$
  ```

### 5.3 getTime

Devuelve el tiempo máximo configurado (ms).

- RX:
  ```text
  $,ID,getTime,IDx,0,%
  ```
- TX:
  ```text
  %,ID,getTime,IDx,TiempoMs,$
  ```

### 5.4 getCant

Devuelve el volumen objetivo en mL.

- RX:
  ```text
  $,ID,getCant,IDx,0,%
  ```
- TX:
  ```text
  %,ID,getCant,IDx,CantMl,$
  ```

### 5.5 getAcumBill

Devuelve el acumulado monetario de la bomba.

- RX:
  ```text
  $,ID,getAcumBill,IDx,0,%
  ```
- TX:
  ```text
  %,ID,getAcumBill,IDx,ValorAcum,$
  ```

### 5.6 ping / pong

Prueba de comunicación.

- RX:
  ```text
  $,ID,ping,IDx,Valor,%
  ```
- TX:
  ```text
  %,ID,pong,IDx,Valor,$
  ```

---

## 6. Eventos y telemetría

### 6.1 BOOT

Se envía al iniciar el firmware.

```text
%,ID,BOOT,0,86,$
```

El valor `86` indica versión 8.6.

### 6.2 end

Fin de ciclo de llenado.

```text
%,ID,end,IDx,ValorFinal,$
```

- `ValorFinal`: valor monetario final, **redondeado a la centena**.

### 6.3 Stat (telemetría 1 s)

Mientras la bomba está activa, se emiten periódicamente:

```text
%,ID,statBill,IDx,ValorBillActual,$
%,ID,statVol,IDx,VolumenMlActual,$
%,ID,statPct,IDx,PorcentajeActual,$
%,ID,statRun,IDx,1,$
```

---

## 7. Códigos de error (ERR)

Errores se envían como:

```text
%,ID,ERR,IDx,Codigo,$
```

Códigos:

- `1` – Trama con muy pocos campos
- `2` – Campo nulo o incompleto
- `3` – `ID` inválido
- `4` – `IDx` fuera de rango
- `9` – Comando no reconocido
- `10` – `chgID` con `IDx` ≠ 0
- `11` – Valor inválido para `chgID`
- `12` – `setRst` con `IDx` ≠ 0
- `13` – `setRst` con valor distinto de 1
- `14` – `setNameB` con `IDx` inválido
- `15` – `setNameB` con nombre inválido

---

## 8. Nota de compatibilidad

En esta versión se han **renombrado** los comandos:

- `fill`  → `setFill`
- `bill`  → `setBill`
- `time`  → `setTime`

Clientes que sigan usando los nombres antiguos deben actualizar sus tramas al nuevo formato.
