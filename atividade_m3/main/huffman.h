/*
 * huffman.h
 * ---------
 * Codificacao de Huffman num UNICO arquivo (header-only).
 * Para usar, em qualquer .c:
 *
 *     #include "huffman.h"
 *
 * Todas as funcoes sao "static": cada arquivo que inclui este header
 * tem a sua propria copia, sem conflito de nomes no link.
 *
 * Ideia geral do algoritmo:
 *   1. Contar quantas vezes cada letra aparece no texto (frequencia).
 *   2. Criar um no para cada letra e juntar sempre os DOIS nos com menor
 *      frequencia num no "pai", ate sobrar um unico no: a RAIZ da arvore.
 *   3. Cada letra recebe um codigo: o caminho da raiz ate a letra
 *      ('0' = ir para a esquerda, '1' = ir para a direita).
 *   4. Trocar cada letra do texto pelo seu codigo (vira uma sequencia de 0 e 1).
 *   5. Para descomprimir: andar na arvore com os bits e, ao chegar numa
 *      folha, escrever a letra dela e voltar para a raiz.
 *
 * As secoes estao na ORDEM em que acontecem no algoritmo:
 *   0. Tipos e constantes     -> struct No e TAMANHO_ALFABETO
 *   1. Criar no               -> cria um no da arvore
 *   2. Contar frequencia      -> conta quantas vezes cada letra aparece
 *   3. Achar menor            -> acha o no com menor frequencia
 *   4. Construir arvore       -> junta os nos ate sobrar a raiz
 *   5. Liberar arvore         -> apaga a arvore da memoria
 *   6. Gerar codigos          -> cria o codigo binario de cada letra
 *   7. Codificar              -> texto -> bits
 *   8. Decodificar            -> bits -> texto
 *   9. Validar                -> ciclo completo e comparacao
 *  10. Gerador de texto       -> texto aleatorio com semente
 *
 * Nao ha printf na logica (so em erro de memoria, via stderr).
 */

#ifndef HUFFMAN_H
#define HUFFMAN_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ==================================================================== */
/*  0. TIPOS E CONSTANTES                                                */
/* ==================================================================== */

/* Quantos valores diferentes um char pode ter (0 a 255). */
#define TAMANHO_ALFABETO 256

/*
 * No da arvore.
 *   simbolo    -> a letra. So as FOLHAS tem letra de verdade ('\0' nos internos).
 *   frequencia -> quantas vezes a letra aparece (ou soma dos filhos, nos internos).
 *   esquerda   -> filho com bit '0'.
 *   direita    -> filho com bit '1'.
 * Folha = no sem filhos (esquerda == NULL e direita == NULL).
 */
typedef struct No
{
    char simbolo;
    int frequencia;
    struct No *esquerda;
    struct No *direita;
} No;

/* ==================================================================== */
/*  1. CRIAR NO                                                          */
/* ==================================================================== */

/* Aloca um no (malloc) e preenche os campos.
   Se faltar memoria, encerra o programa com uma mensagem. */
static No *criar_no(char simbolo, int frequencia, No *esquerda, No *direita)
{
    No *novo = (No *)malloc(sizeof(No));
    if (novo == NULL)
    {
        fprintf(stderr, "Erro: sem memoria\n");
        exit(1);
    }
    novo->simbolo = simbolo;
    novo->frequencia = frequencia;
    novo->esquerda = esquerda;
    novo->direita = direita;
    return novo;
}

/* ==================================================================== */
/*  2. CONTAR FREQUENCIA                                                 */
/* ==================================================================== */

/* Preenche freq[v] com quantas vezes o valor v aparece no texto.
   Devolve quantos valores DIFERENTES apareceram (quantas folhas a arvore vai ter). */
static int contar_frequencias(const char *texto, int freq[TAMANHO_ALFABETO])
{
    for (int i = 0; i < TAMANHO_ALFABETO; i++)
    {
        freq[i] = 0;
    }
    for (int i = 0; texto[i] != '\0'; i++)
    {
        freq[(unsigned char)texto[i]]++;
    }

    int diferentes = 0;
    for (int i = 0; i < TAMANHO_ALFABETO; i++)
    {
        if (freq[i] > 0)
        {
            diferentes++;
        }
    }
    return diferentes;
}

/* ==================================================================== */
/*  3. ACHAR MENOR                                                       */
/* ==================================================================== */

/* Devolve a POSICAO (indice) do no com menor frequencia em nos[0..qtd-1]. */
static int posicao_menor(No **nos, int qtd)
{
    int menor = 0;
    for (int i = 1; i < qtd; i++)
    {
        if (nos[i]->frequencia < nos[menor]->frequencia)
        {
            menor = i;
        }
    }
    return menor;
}

/* ==================================================================== */
/*  4. CONSTRUIR ARVORE                                                  */
/* ==================================================================== */

/* Recebe o texto e devolve a RAIZ da arvore de Huffman (NULL se vazio).
   Passos:
     a) conta as frequencias;
     b) cria uma folha para cada letra diferente;
     c) enquanto sobrar mais de um no, junta os dois de menor frequencia
        num no pai; d) o ultimo no que sobrar e a raiz. */
static No *construir_arvore(const char *texto)
{
    /* a) frequencias */
    int freq[TAMANHO_ALFABETO];
    int qtd = contar_frequencias(texto, freq);
    if (qtd == 0)
    {
        return NULL;
    }

    /* b) uma folha por letra diferente */
    No **nos = (No **)malloc(qtd * sizeof(No *));
    if (nos == NULL)
    {
        fprintf(stderr, "Erro: sem memoria\n");
        exit(1);
    }
    int k = 0;
    for (int i = 0; i < TAMANHO_ALFABETO; i++)
    {
        if (freq[i] > 0)
        {
            nos[k] = criar_no((char)i, freq[i], NULL, NULL);
            k++;
        }
    }

    /* c) junta os dois menores ate sobrar um */
    while (qtd > 1)
    {
        int a = posicao_menor(nos, qtd);
        No *m1 = nos[a];
        nos[a] = nos[qtd - 1];
        qtd--;

        int b = posicao_menor(nos, qtd);
        No *m2 = nos[b];
        nos[b] = criar_no('\0', m1->frequencia + m2->frequencia, m1, m2);
    }

    /* d) a raiz */
    No *raiz = nos[0];
    free(nos);
    return raiz;
}

/* ==================================================================== */
/*  5. LIBERAR ARVORE                                                    */
/* ==================================================================== */

/* Apaga todos os nos. Filhos primeiro, pai depois (senao perderiamos os filhos). */
static void liberar_arvore(No *no)
{
    if (no == NULL)
    {
        return;
    }
    liberar_arvore(no->esquerda);
    liberar_arvore(no->direita);
    free(no);
}

/* ==================================================================== */
/*  6. GERAR CODIGOS                                                     */
/* ==================================================================== */

/* Devolve 1 se o no e uma folha (uma letra), 0 caso contrario. */
static int eh_folha(const No *no)
{
    return no->esquerda == NULL && no->direita == NULL;
}

/* Recursao: desce pela arvore montando o caminho ('0' esquerda, '1' direita).
   Ao chegar numa folha, guarda uma copia do caminho em codigos[letra]. */
static void gerar_codigos_rec(const No *no, char *caminho, int prof, char **codigos)
{
    if (no == NULL)
    {
        return;
    }

    if (eh_folha(no))
    {
        char *copia = (char *)malloc(prof + 2); /* bits + '\0' (+1 de folga) */
        if (copia == NULL)
        {
            fprintf(stderr, "Erro: sem memoria\n");
            exit(1);
        }
        if (prof == 0)
        {
            /* so existe UMA letra: a raiz ja e a folha, usamos "0" */
            strcpy(copia, "0");
        }
        else
        {
            caminho[prof] = '\0';
            strcpy(copia, caminho);
        }
        codigos[(unsigned char)no->simbolo] = copia;
        return;
    }

    caminho[prof] = '0';
    gerar_codigos_rec(no->esquerda, caminho, prof + 1, codigos);

    caminho[prof] = '1';
    gerar_codigos_rec(no->direita, caminho, prof + 1, codigos);
}

/* Cria a tabela codigos[letra] (NULL para letras ausentes). Libere com liberar_codigos. */
static char **gerar_codigos(const No *raiz)
{
    char **codigos = (char **)calloc(TAMANHO_ALFABETO, sizeof(char *));
    if (codigos == NULL)
    {
        fprintf(stderr, "Erro: sem memoria\n");
        exit(1);
    }
    char caminho[TAMANHO_ALFABETO];
    if (raiz != NULL)
    {
        gerar_codigos_rec(raiz, caminho, 0, codigos);
    }
    return codigos;
}

/* Apaga cada codigo da tabela e depois a tabela. */
static void liberar_codigos(char **codigos)
{
    for (int i = 0; i < TAMANHO_ALFABETO; i++)
    {
        free(codigos[i]); /* free(NULL) nao faz nada */
    }
    free(codigos);
}

/* ==================================================================== */
/*  7. CODIFICAR (texto -> bits)                                         */
/* ==================================================================== */

/* Troca cada letra pelo seu codigo e junta tudo numa string de '0' e '1'.
   Libere com free(). */
static char *codificar(const char *texto, char **codigos)
{
    /* 1a passada: tamanho total em bits */
    size_t total = 0;
    for (size_t i = 0; texto[i] != '\0'; i++)
    {
        total += strlen(codigos[(unsigned char)texto[i]]);
    }

    /* 2a passada: copia os codigos um depois do outro */
    char *bits = (char *)malloc(total + 1);
    if (bits == NULL)
    {
        fprintf(stderr, "Erro: sem memoria\n");
        exit(1);
    }

    char *posicao = bits;
    for (size_t i = 0; texto[i] != '\0'; i++)
    {
        const char *codigo = codigos[(unsigned char)texto[i]];
        strcpy(posicao, codigo);
        posicao += strlen(codigo);
    }
    *posicao = '\0';
    return bits;
}

/* ==================================================================== */
/*  8. DECODIFICAR (bits -> texto)                                       */
/* ==================================================================== */

/* Le cada bit e anda na arvore ('0' esquerda, '1' direita). Ao chegar numa
   folha, guarda a letra e volta para a raiz. Libere com free(). */
static char *decodificar(const No *raiz, const char *bits)
{
    /* a frequencia da raiz e o numero de letras do texto original */
    char *saida = (char *)malloc(raiz->frequencia + 1);
    if (saida == NULL)
    {
        fprintf(stderr, "Erro: sem memoria\n");
        exit(1);
    }

    int k = 0;
    const No *atual = raiz;

    for (size_t i = 0; bits[i] != '\0'; i++)
    {
        /* caso especial: so existe uma letra, cada bit vira essa letra */
        if (eh_folha(raiz))
        {
            saida[k] = raiz->simbolo;
            k++;
            continue;
        }

        if (bits[i] == '0')
        {
            atual = atual->esquerda;
        }
        else
        {
            atual = atual->direita;
        }

        if (eh_folha(atual))
        {
            saida[k] = atual->simbolo;
            k++;
            atual = raiz;
        }
    }

    saida[k] = '\0';
    return saida;
}

/* ==================================================================== */
/*  9. VALIDAR (ciclo completo)                                          */
/* ==================================================================== */

/* Faz arvore -> codigos -> codificar -> decodificar e compara com o original.
   Devolve 1 se bateu, 0 se nao bateu ou se o texto estiver vazio. */
static int huffman_validar(const char *texto)
{
    if (texto[0] == '\0')
    {
        return 0; /* texto vazio nao serve para o experimento */
    }

    No *raiz = construir_arvore(texto);
    char **codigos = gerar_codigos(raiz);
    char *bits = codificar(texto, codigos);
    char *decodificado = decodificar(raiz, bits);

    int ok = (strcmp(texto, decodificado) == 0);

    free(decodificado);
    free(bits);
    liberar_codigos(codigos);
    liberar_arvore(raiz);
    return ok;
}

/* ==================================================================== */
/*  10. GERADOR DE TEXTO ALEATORIO (com semente)                         */
/* ==================================================================== */

/* Passo do xorshift32: muda o estado e devolve o novo valor pseudoaleatorio. */
static uint32_t xorshift32(uint32_t *estado)
{
    uint32_t x = *estado;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *estado = x;
    return x;
}

/* Preenche destino com n caracteres de 1 a 255, seguidos de '\0'.
   Mesma semente = mesmo texto. destino precisa ter n + 1 posicoes. */
static void huffman_gerar_texto(char *destino, size_t n, uint32_t semente)
{
    /* o xorshift nao funciona com estado 0, entao trocamos por 1 */
    uint32_t estado = (semente == 0) ? 1 : semente;

    for (size_t i = 0; i < n; i++)
    {
        /* 1 a 255: nunca 0, para nao encerrar a string antes do fim */
        destino[i] = (char)(1 + xorshift32(&estado) % 255);
    }
    destino[n] = '\0';
}

#endif /* HUFFMAN_H */
