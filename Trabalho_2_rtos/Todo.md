# Temática 1 — Drone

## Contexto

O contexto desta temática implementa, em uma **ESP32 com FreeRTOS**, um autopiloto didático que reproduz o ciclo essencial de um drone:

* Fusão sensorial periódica;
* Controle de atitude;
* Navegação;
* Rotinas de segurança.

Para tornar o experimento reprodutível em laboratório, todos os estímulos externos são simulados por sensores **touch** presentes na própria ESP32, mapeados para:

* Nova amostra inercial;
* Comando de rota/telemetria;
* Fail-safe (emergência).

Com isso, investigamos como restrições temporais rígidas (**Hard Real-Time**) e brandas (**Soft Real-Time**) se comportam sob diferentes políticas de prioridade:

* **Rate Monotonic (RM)**;
* **Deadline Monotonic (DM)**;
* Critério customizado.

Também são avaliadas diferentes configurações do escalonador do FreeRTOS.

O foco está na **previsibilidade temporal**. A tarefa periódica de fusão possui período de **5 ms** e alimenta o controle de atitude com deadline curta, enquanto eventos de navegação e telemetria são assíncronos. O sistema de **fail-safe** exige baixa latência e é gerado a partir de uma **ISR**, responsável por ativar uma task dedicada.

O desempenho do sistema é medido por meio de:

* Misses de deadline;
* Jitter;
* Latência ao toque.

Os experimentos variam os seguintes parâmetros:

* Preempção;
* Time slicing;
* Frequência da CPU:

  * 80 MHz;
  * 160 MHz;
  * 240 MHz;
* Execução em modo **unicore**.

Os resultados permitem discutir os trade-offs entre:

* Robustez do controle;
* Tempo de resposta a emergências;
* Custo das preempções no tempo de execução.

---

## Introdução

O sistema de controle do drone utiliza uma tarefa periódica para manter o estado inercial por meio de uma fusão de sensores simulada, além de tarefas encadeadas responsáveis pelo controle de atitude.

Eventos de navegação, telemetria e fail-safe são gerados por toques nos pads touch da ESP32.

Todo o fluxo é instrumentado para medir latências e verificar o cumprimento de deadlines sob diferentes políticas de prioridade e configurações do FreeRTOS.

---

## Mapeamento das Entradas Touch

| Touch       | Função                                                                |
| ----------- | --------------------------------------------------------------------- |
| **Touch A** | Opcional: injeta perturbações nos sinais para gerar estresse de carga |
| **Touch B** | Comando de rota / evento de navegação                                 |
| **Touch C** | Solicitação de telemetria / flush                                     |
| **Touch D** | Fail-safe / emergência                                                |

---

## Tasks

### 1. FUS_IMU

**Classificação:** Hard RT *(analisar se deve ser considerada Hard ou Soft RT)*
**Tipo:** Periódica

#### Parâmetros temporais

* **T — Período:** 5 ms
* **D — Deadline:** 5 ms
* **C — WCET:** aproximadamente 1,0 ms

  * O WCET deverá ser medido experimentalmente.

#### Função

Executa a filtragem e o estimador de orientação, utilizando algoritmos como:

* Filtro complementar;
* Madgwick.

A tarefa produz o estado inercial atualizado do drone.

#### Comunicação

Após finalizar o processamento, sinaliza a próxima tarefa utilizando mecanismos como:

* Queue;
* Task Notification.

---

### 2. CTRL_ATT

**Classificação:** Hard RT *(analisar se deve ser considerada Hard ou Soft RT)*
**Tipo:** Encadeada pela `FUS_IMU`

#### Gatilho

É ativada por meio de:

* Task Notification; ou
* Queue enviada pela `FUS_IMU`.

#### Parâmetros temporais

* **D — Deadline:** 5 ms após a geração da amostra
* **C — WCET:** aproximadamente 0,8 ms

  * O WCET deverá ser medido experimentalmente.

#### Função

Executa:

* Controle PID;
* Cálculo da atuação;
* Atualização dos atuadores simulados.

---

### 3. NAV_PLAN

**Classificação:** Soft RT *(analisar se deve ser considerada Hard ou Soft RT)*
**Tipo:** Orientada a evento

#### Gatilho

Ativada pelo **Touch B**.

#### Parâmetros temporais

* **D — Deadline:** 20 ms
* **C — WCET:** aproximadamente 3–4 ms

  * O WCET deverá ser medido experimentalmente.

#### Função

Responsável por:

* Atualizar waypoint;
* Atualizar rota;
* Processar comandos de navegação.

Caso exista uma flag de telemetria gerada pelo **Touch C**, a tarefa também realiza o envio do estado atual do sistema.

---

### 4. FS_TASK

**Classificação:** Hard RT
**Tipo:** Orientada a evento

#### Gatilho

Ativada pelo **Touch D**, representando uma condição de emergência.

O evento deve ser detectado por uma **ISR**, que sinaliza imediatamente a execução da `FS_TASK`.

#### Parâmetros temporais

* **D — Deadline:** 10 ms a partir do toque
* **C — WCET:** aproximadamente 0,8–1,0 ms

#### Função

Executa a rotina de segurança do drone:

* Redução imediata do throttle;
* Entrada em modo hover seguro; ou
* Início de procedimento de pouso seguro.
