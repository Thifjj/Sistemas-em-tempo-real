#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/*
 * Compatibilidade ESP32 (ESP-IDF) x PC
 * ------------------------------------
 * ESP_PLATFORM e definido automaticamente pelo ESP-IDF. Assim o MESMO
 * arquivo compila na ESP32 (entrada = app_main, dentro de uma task do
 * FreeRTOS) e no PC com gcc (entrada = main), o que facilita testar.
 */
#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h" // esp_get_free_heap_size()
#include "esp_timer.h"  // esp_timer_get_time()
#endif

// Maximo de simbolos diferentes que um 'char' pode ter (0..255).
// Tambem limita a profundidade da arvore (no maximo 255 niveis).
#define MAX_SIMBOLOS 256

/*
 * xcalloc
 * -------
 * calloc com verificacao. A RAM da ESP32 e pequena (~300 KB livres), entao
 * a alocacao PODE falhar. Em vez de seguir com ponteiro NULL (e travar com
 * um erro dificil de entender), avisamos e abortamos de forma clara.
 * calloc zera a memoria, o que e util para o vetor de bits.
 */
static void *xcalloc(size_t qtd, size_t tamanho)
{
    void *p = calloc(qtd, tamanho);
    if (p == NULL)
    {
        fprintf(stderr, "Erro: sem memoria\n");
        abort();
    }
    return p;
}

/*
 * contar_letras
 * -------------
 * Conta quantas vezes cada caractere aparece em "word".
 * NAO usa tamanho fixo: os vetores de saida crescem conforme
 * aparecem caracteres novos (realloc).
 *
 * Parametros:
 *   word        -> string de entrada (terminada em '\0')
 *   letters     -> (saida) endereco de um ponteiro; vai apontar para um
 *                  vetor com os caracteres DIFERENTES encontrados
 *   freq_letter -> (saida) endereco de um ponteiro; vai apontar para um
 *                  vetor de frequencias: freq_letter[k] = quantas vezes
 *                  letters[k] apareceu
 *
 * Retorno: quantidade de caracteres diferentes (tamanho dos dois vetores).
 *
 * Por que ponteiro de ponteiro (char **)?
 *   Em C os argumentos sao passados por copia. Se a funcao receber so
 *   "char *letters" e fizer realloc, so a copia muda e o main nao ve o
 *   novo endereco. Passando "&letras" (char **), a funcao altera o
 *   ponteiro original do main.
 */

typedef struct No
{
    char caractere;
    int frequencia;

    struct No *esquerda;
    struct No *direita;
} No;

int contar_letras(const char *word, char **letters, int **freq_letter)
{
    *letters = NULL; // realloc(NULL, n) se comporta como malloc(n)
    *freq_letter = NULL;

    // Palavra vazia: nao ha nada para contar
    if (word[0] == '\0')
        return 0;

    // O primeiro caractere obviamente ainda nao foi registrado,
    // entao ja o colocamos direto com frequencia 1, sem precisar procurar.
    *letters = malloc(sizeof(char));
    *freq_letter = malloc(sizeof(int));
    (*letters)[0] = word[0];
    (*freq_letter)[0] = 1;
    int qtd = 1; // quantos caracteres diferentes ja encontramos

    // Percorre o restante da palavra (comeca em i = 1, pois o 0 ja foi tratado)
    for (int i = 1; word[i] != '\0'; i++)
    {
        char c = word[i];

        // Procura "c" entre os caracteres que ja foram registrados
        int achou = -1;
        for (int j = 0; j < qtd; j++)
        {
            if ((*letters)[j] == c)
            {
                achou = j; // guarda a posicao onde ele esta
                break;
            }
        }

        if (achou >= 0)
        {
            // Caractere ja conhecido: so incrementa a contagem dele
            (*freq_letter)[achou]++;
        }
        else
        {
            // Caractere novo: aumenta os dois vetores em 1 posicao
            char *novo_l = realloc(*letters, (qtd + 1) * sizeof(char));
            int *novo_f = realloc(*freq_letter, (qtd + 1) * sizeof(int));

            *letters = novo_l;
            *freq_letter = novo_f;

            (*letters)[qtd] = c;     // registra o caractere novo
            (*freq_letter)[qtd] = 1; // primeira ocorrencia
            qtd++;                   // agora temos mais um tipo
        }
    }
    return qtd;
}

/*
 * criar_no
 * --------
 * Aloca UM no na memoria e preenche seus campos.
 * Folha: caractere = letra real, esquerda = direita = NULL.
 * No interno (criado na uniao de dois nos): caractere = '\0' (nao representa letra).
 */
No *criar_no(char caractere, int frequencia, No *esquerda, No *direita)
{
    No *no = malloc(sizeof(No));
    no->caractere = caractere;
    no->frequencia = frequencia;
    no->esquerda = esquerda;
    no->direita = direita;
    return no;
}

/*
 * achar_menor
 * -----------
 * Dentro do vetor de ponteiros "nos" (tamanho "qtd"), devolve a POSICAO
 * do no com menor frequencia.
 */
int achar_menor(No **nos, int qtd)
{
    int menor = 0;
    for (int i = 1; i < qtd; i++)
    {
        if (nos[i]->frequencia < nos[menor]->frequencia)
            menor = i;
    }
    return menor;
}

/*
 * construir_arvore
 * ----------------
 * Monta a arvore de Huffman e devolve a RAIZ.
 *
 * Ideia:
 *   1. Cria uma folha para cada letra (com sua frequencia).
 *   2. Repete ate sobrar so 1 no:
 *        - tira os DOIS nos de menor frequencia;
 *        - cria um no pai com frequencia = soma dos dois,
 *          filho esquerdo = o menor, direito = o segundo menor;
 *        - coloca o pai de volta no vetor.
 *   3. O unico no que sobrar e a raiz.
 *
 * Letras raras ficam fundas na arvore; letras comuns ficam perto da raiz.
 */
No *construir_arvore(const char *letras, const int *freq, int n)
{
    if (n == 0)
        return NULL;

    No **nos = malloc(n * sizeof(No *));

    // Passo 1: uma folha por letra
    for (int i = 0; i < n; i++)
        nos[i] = criar_no(letras[i], freq[i], NULL, NULL);

    int qtd = n; // quantos nos ativos ainda existem

    // Passo 2: junta de dois em dois ate sobrar um
    while (qtd > 1)
    {
        // Pega o menor e o remove do vetor
        int a = achar_menor(nos, qtd);
        No *menor1 = nos[a];
        nos[a] = nos[qtd - 1]; // o ultimo ocupa o buraco
        qtd--;

        // Pega o segundo menor (agora o menor do que restou) e remove
        int b = achar_menor(nos, qtd);
        No *menor2 = nos[b];
        nos[b] = nos[qtd - 1];
        qtd--;

        // Cria o pai e o devolve ao vetor (cabe: liberamos 2 posicoes)
        nos[qtd] = criar_no('\0', menor1->frequencia + menor2->frequencia,
                            menor1, menor2);
        qtd++;
    }

    No *raiz = nos[0];
    free(nos); // o vetor auxiliar nao e mais necessario (os nos continuam vivos)
    return raiz;
}

/*
 * imprimir_arvore
 * ---------------
 * Percorre a arvore (em pre-ordem) mostrando cada no, indentado pela
 * profundidade, para voce conferir o formato. Usa recursao.
 */
void imprimir_arvore(const No *no, int nivel)
{
    if (no == NULL)
        return;
    for (int i = 0; i < nivel; i++)
        printf("    ");
    if (no->esquerda == NULL && no->direita == NULL)
        printf("'%c' (%d)\n", no->caractere, no->frequencia); // folha
    else
        printf("* (%d)\n", no->frequencia); // no interno
    imprimir_arvore(no->esquerda, nivel + 1);
    imprimir_arvore(no->direita, nivel + 1);
}

/*
 * liberar_arvore
 * --------------
 * Libera todos os nos. Pos-ordem: primeiro os filhos, depois o pai
 * (se liberasse o pai antes, perderia o acesso aos filhos).
 */
void liberar_arvore(No *no)
{
    if (no == NULL)
        return;
    liberar_arvore(no->esquerda);
    liberar_arvore(no->direita);
    free(no);
}

/*
 * gerar_codigos
 * -------------
 * Percorre a arvore e descobre o codigo (sequencia de 0 e 1) de cada letra.
 * Regra: descer para a ESQUERDA = bit '0', descer para a DIREITA = bit '1'.
 * O codigo de uma letra e o caminho da raiz ate a folha dela.
 *
 *   no      -> no atual da recursao
 *   buf     -> caminho percorrido ate aqui (compartilhado entre as chamadas)
 *   prof    -> profundidade atual = quantos bits ja estao em buf
 *   tabela  -> (saida) tabela[(unsigned char)letra] = string de '0'/'1'
 *
 * Como nenhuma folha e ancestral de outra, nenhum codigo e prefixo de outro.
 * E isso que permite decodificar sem separadores entre as letras.
 */
static void gerar_codigos(const No *no, char *buf, int prof, char **tabela)
{
    if (no == NULL)
        return;

    if (no->esquerda == NULL && no->direita == NULL)
    {
        // Caso especial: texto com UM unico tipo de letra. A raiz e a propria
        // folha e o caminho seria vazio (0 bits). Damos o codigo "0".
        if (prof == 0)
        {
            buf[0] = '0';
            prof = 1;
        }
        buf[prof] = '\0'; // fecha a string
        tabela[(unsigned char)no->caractere] = xcalloc(prof + 1, 1);
        memcpy(tabela[(unsigned char)no->caractere], buf, prof + 1);
        return;
    }

    buf[prof] = '0'; // desceu para a esquerda
    gerar_codigos(no->esquerda, buf, prof + 1, tabela);
    buf[prof] = '1'; // desceu para a direita
    gerar_codigos(no->direita, buf, prof + 1, tabela);
}

/*
 * codificar
 * ---------
 * Converte o texto numa sequencia de BITS compactada em bytes.
 * Cada letra vira seu codigo de Huffman, e os bits sao colocados um apos
 * o outro, 8 por byte (do bit mais significativo para o menos).
 *
 *   texto       -> texto original
 *   tabela      -> codigos gerados por gerar_codigos
 *   total_bits  -> (saida) quantos bits sao validos. Necessario porque o
 *                  ultimo byte quase sempre tem bits de sobra (preenchimento).
 *
 * Retorna o vetor de bytes (libere com free).
 * Pressupoe texto nao vazio.
 */
static uint8_t *codificar(const char *texto, char **tabela, size_t *total_bits)
{
    // 1a passada: so soma o tamanho dos codigos para saber quantos bytes alocar
    size_t bits = 0;
    for (size_t i = 0; texto[i] != '\0'; i++)
        bits += strlen(tabela[(unsigned char)texto[i]]);

    // (bits + 7) / 8 = divisao por 8 arredondada para CIMA
    uint8_t *bytes = xcalloc((bits + 7) / 8, 1); // zerado: so ligamos os '1'

    // 2a passada: escreve os bits
    size_t pos = 0; // posicao do proximo bit a escrever
    for (size_t i = 0; texto[i] != '\0'; i++)
    {
        const char *codigo = tabela[(unsigned char)texto[i]];
        for (size_t k = 0; codigo[k] != '\0'; k++)
        {
            if (codigo[k] == '1')
            {
                // pos >> 3  = pos / 8    -> em qual byte esta o bit
                // pos & 7   = pos % 8    -> qual bit dentro do byte
                // 0x80 >> x = bit mais a esquerda deslocado x casas
                bytes[pos >> 3] |= (uint8_t)(0x80 >> (pos & 7));
            }
            pos++; // se for '0' nao precisa fazer nada: o byte ja e zero
        }
    }

    *total_bits = bits;
    return bytes;
}

/*
 * decodificar
 * -----------
 * Faz o caminho inverso: le os bits e anda pela arvore.
 *   bit 0 -> esquerda,  bit 1 -> direita.
 * Ao chegar numa folha, emite a letra e volta para a raiz.
 *
 * O tamanho do texto original e a frequencia da raiz (soma de todas as
 * folhas), entao sabemos de antemao quanto alocar.
 *
 * Retorna a string decodificada (libere com free).
 */
static char *decodificar(const No *raiz, const uint8_t *bytes, size_t total_bits)
{
    size_t n = (size_t)raiz->frequencia;
    char *saida = xcalloc(n + 1, 1); // +1 para o '\0' final

    // Caso especial: raiz folha (um unico tipo de letra, codigo "0")
    if (raiz->esquerda == NULL && raiz->direita == NULL)
    {
        memset(saida, raiz->caractere, n);
        return saida;
    }

    const No *atual = raiz;
    size_t k = 0; // quantas letras ja emitimos
    for (size_t pos = 0; pos < total_bits; pos++)
    {
        // Extrai o bit "pos": inverso do que codificar fez
        int bit = (bytes[pos >> 3] >> (7 - (pos & 7))) & 1;
        atual = bit ? atual->direita : atual->esquerda;

        if (atual->esquerda == NULL && atual->direita == NULL)
        {
            saida[k++] = atual->caractere; // chegou numa folha: achou a letra
            atual = raiz;                  // recomeca para a proxima
        }
    }
    return saida;
}

/*
 * executar_huffman
 * ----------------
 * Fluxo completo: contar -> arvore -> codigos -> codificar -> decodificar
 * -> conferir. Mostra o ganho de espaco e (na ESP32) tempo e uso de heap.
 */
static void executar_huffman(const char *texto)
{
    if (texto[0] == '\0')
    {
        printf("Texto vazio, nada a fazer.\n");
        return;
    }

#ifdef ESP_PLATFORM
    uint32_t heap_antes = esp_get_free_heap_size();
    int64_t t0 = esp_timer_get_time(); // microssegundos desde o boot
#endif

    // 1) Frequencias
    char *letras = NULL;
    int *freq = NULL;
    int n = contar_letras(texto, &letras, &freq);

    // 2) Arvore
    No *raiz = construir_arvore(letras, freq, n);

    // 3) Tabela de codigos. 256 ponteiros = 1 KB (ou 2 KB em 64 bits);
    //    posicoes sem letra ficam NULL (calloc zera).
    char **tabela = xcalloc(MAX_SIMBOLOS, sizeof(char *));
    char buf[MAX_SIMBOLOS + 1]; // caminho atual (profundidade maxima 255 + '\0')
    gerar_codigos(raiz, buf, 0, tabela);

    printf("Codigos:\n");
    for (int i = 0; i < n; i++)
        printf("  '%c' (%d x) -> %s\n", letras[i], freq[i],
               tabela[(unsigned char)letras[i]]);

    // 4) Codificar
    size_t total_bits = 0;
    uint8_t *comprimido = codificar(texto, tabela, &total_bits);
    size_t bytes_comp = (total_bits + 7) / 8;
    size_t bytes_orig = strlen(texto);

    // 5) Decodificar e conferir
    char *decodificado = decodificar(raiz, comprimido, total_bits);
    int ok = (strcmp(texto, decodificado) == 0);

#ifdef ESP_PLATFORM
    int64_t t1 = esp_timer_get_time();
#endif

    // Em Huffman a arvore (ou a tabela de frequencias) tambem precisa ser
    // guardada/enviada junto para quem for decodificar. Aqui nao contamos isso.
    printf("\nOriginal:    %u bytes (%u bits)\n", (unsigned)bytes_orig,
           (unsigned)(bytes_orig * 8));
    printf("Comprimido:  %u bytes (%u bits validos)\n", (unsigned)bytes_comp,
           (unsigned)total_bits);
    printf("Taxa:        %.1f%% do tamanho original\n",
           100.0 * (double)bytes_comp / (double)bytes_orig);
    printf("Conferencia: %s\n", ok ? "OK (texto identico)" : "ERRO");

#ifdef ESP_PLATFORM
    printf("Tempo total: %lld us\n", (long long)(t1 - t0));
#endif

    // 6) Liberar TUDO (na ESP32 vazamento = heap acabando com o tempo)
    free(decodificado);
    free(comprimido);
    for (int i = 0; i < MAX_SIMBOLOS; i++)
        free(tabela[i]); // free(NULL) e permitido e nao faz nada
    free(tabela);
    liberar_arvore(raiz);
    free(letras);
    free(freq);

#ifdef ESP_PLATFORM
    // Se heap_depois < heap_antes, algo nao foi liberado
    printf("Heap: antes=%u depois=%u\n", (unsigned)heap_antes,
           (unsigned)esp_get_free_heap_size());
#endif
}

// Texto de teste. Pode trocar por qualquer outro (de qualquer tamanho).
static const char *TEXTO = "abracadabra";

#ifdef ESP_PLATFORM

/*
 * Task do FreeRTOS que roda o Huffman.
 * A task padrao de app_main tem pouca stack; as funcoes recursivas e os
 * buffers precisam de mais folga, por isso criamos uma task propria com 8 KB.
 */
static void huffman_task(void *arg)
{
    (void)arg;
    executar_huffman(TEXTO);
    vTaskDelete(NULL); // task terminou: se remove (nunca retorne de uma task)
}

// Ponto de entrada na ESP32 (no lugar de main)
void app_main(void)
{
    xTaskCreate(huffman_task, "huffman", 8192, NULL, 5, NULL);
}

#else

// Ponto de entrada no PC (para testar com gcc)
int main(void)
{
    executar_huffman(TEXTO);
    return 0;
}

#endif
