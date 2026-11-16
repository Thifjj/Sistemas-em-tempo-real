# Experimento C: margem de segurança sobre o HWM da etapa 1

Análise feita só com os números já medidos (`etapa1_teste.md`, `etapa2_teste.md`, `etapa3_teste.md`, `etapa4_teste.md`). Nenhuma rodada nova.

## 1. Base do cálculo

- HWM da etapa 1 (base da margem): **11266 µs**
- HWM das etapas (maior valor entre as rodadas válidas):
  - Etapa 2: 11458 µs
  - Etapa 3: 11527 µs
  - Etapa 4: 13498 µs (rodadas com watchdog desligado; 13473 a 13498)
- D atual: 13519 µs (= 11266 × 1,2)

## 2. Valores com cada margem

| Margem | Fator | Valor (µs) |
|---|---|---|
| 10% | × 1,10 | 12392,6 |
| 25% | × 1,25 | 14082,5 |
| 50% | × 1,50 | 16899,0 |

## 3. Cada margem cobre o HWM de cada etapa?

Cobre = margem ≥ HWM da etapa. Folga = margem − HWM da etapa.

| Margem | Etapa 1 (11266) | Etapa 2 (11458) | Etapa 3 (11527) | Etapa 4 (13498) |
|---|---|---|---|---|
| 10% (12393) | sim, folga 1127 | sim, folga 935 | sim, folga 866 | **não**, falta 1105 |
| 25% (14083) | sim, folga 2817 | sim, folga 2625 | sim, folga 2556 | sim, folga 585 |
| 50% (16899) | sim, folga 5633 | sim, folga 5441 | sim, folga 5372 | sim, folga 3401 |

## 4. Margem mínima que cobre a etapa 4

- 13498 / 11266 = 1,198, ou seja, **≈ 19,8%** sobre o HWM da etapa 1.
- Logo, a margem de **10% não cobre** a etapa 4, e a de **25% cobre** com folga de 585 µs.

## 5. Leitura para o relatório

- A margem de 10% (que é a que usaríamos com o D atual) **não é suficiente** para a etapa 4. Ela só funciona enquanto o sistema não tem carga adicional.
- A margem de 25% cobre todas as etapas, mas a etapa 4 fica com folga de só 585 µs, ou ~4% do valor da margem. Por isso ela é a mais exposta.
- A margem de 50% sobra demais: é segura, mas torna o D pouco útil para detectar perdas.
- A margem fixa só é adequada se a variabilidade não muda entre etapas. Aqui ela muda: o HWM sobe 2232 µs da etapa 1 para a etapa 4 (quase 20%). Isso mostra que uma margem única não acompanha a concorrência.

## 6. Ressalvas

- Os HWMs são máximos de rodadas, não o pior caso real. Uma rodada a mais pode subir o HWM e mudar essas conclusões.
- A etapa 4 foi medida com o watchdog de tarefas desligado e com a carga no core 0, que está saturado (ver `etapa4_teste.md`, seção 7).
- Esta análise não depende do CSV, porque usa só o HWM de cada etapa.
