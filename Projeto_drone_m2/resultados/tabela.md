| Tema | Configuracao | Protocolo | MHz | Preempcao | Slicing | Politica | Todos deadlines atendidos | Repeticoes | WCRT (ms) | HWM100 (ms) | (m,k)-firm | Hard misses (%) | Soft misses (%) | Latencia Touch max (ms) | Latencia Protocolo (RTT max, ms) | RTT sem reenvio max (ms) | RTT com reenvio max (ms) | Reenvios | ESP ate PC max (ms) | PC ate task max (ms) | CPU max recebida (%) | Conclusao |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Drone | melhor | TCP | 160 | 1 | 1 | RM | False | 5 | 25.458 | 25.458 | (9,10); violacoes=0; janelas=45 | 0.1715 | 0.0 | 1.195 | 318.309 | 318.309 | N/D | 0 | N/D | N/D | 46.62 | Falha hard |
| Drone | melhor | UDP | 160 | 1 | 1 | RM | True | 5 | 10.305 | 10.305 | (9,10); violacoes=0; janelas=45 | 0.0 | 0.0 | 1.905 | 135.142 | 135.142 | N/D | 0 | N/D | N/D | 45.49 | OK hard; soft atende (9,10) |
| Drone | pior | TCP | 80 | 0 | 1 | RM | False | 5 | 44.531 | 44.531 | (9,10); violacoes=0; janelas=45 | 0.6728 | 3.3333 | 16.315 | 382.136 | 382.136 | N/D | 0 | N/D | N/D | 53.02 | Falha hard |
| Drone | pior | UDP | 80 | 0 | 1 | RM | False | 5 | 16.601 | 16.601 | (9,10); violacoes=0; janelas=45 | 0.4664 | 0.0 | 8.724 | 214.137 | 214.137 | N/D | 0 | N/D | N/D | 50.3 | Falha hard |

Latencia Protocolo = maior RTT medido entre envio do comando e recebimento do ACK no PC; inclui processamento, filas e eventuais reenvios. Nao representa atraso de mao unica.
