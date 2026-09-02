#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pgmfiles.h"
#include "diff2d.h"
#include <time.h>
clock_t begin, end;
double time_spent;

//gcc -o fda pgmtolist.c pgmfiles.c diff2d.c main.c -lm

void main (int argc, char **argv) {
  char   row[80];
  float  **matrix;
  int i, j;
  FILE   *inimage, *outimage;
  long   imax;
  float  lambda;
  int result;
  eightBitPGMImage *PGMImage;
  
  /* ---- read image name  ---- */
  begin = clock();
  PGMImage = (eightBitPGMImage *) malloc(sizeof(eightBitPGMImage));
	
  if (!argv[1])
  {
    printf("name of input PGM image file (with extender): ");
    scanf("%s", PGMImage->fileName);
  }
  else
  {
    strcpy(PGMImage->fileName, argv[1]);
  }
  
  result = read8bitPGM(PGMImage);
  
  if(result < 0) 
  {
    printPGMFileError(result);
    exit(result);
  }
  end = clock();
  time_spent = (double)(end - begin)/CLOCKS_PER_SEC;
  printf("Tempo que levou para ler o nome da imagem foi de %f\n",time_spent);
  
  /* ---- allocate storage for matrix ---- */
  begin = clock();
  matrix = (float **) malloc (PGMImage->x * sizeof(float *));
  if (matrix == NULL)
  { 
    printf("not enough storage available\n");
    exit(1);
  } 
  for (i=0; i<PGMImage->x; i++)
  {
    matrix[i] = (float *) malloc (PGMImage->y * sizeof(float));
    if (matrix[i] == NULL)
    { 
      printf("not enough storage available\n");
      exit(1);
    }
  }
  end = clock();
  time_spent = (double)(end - begin)/CLOCKS_PER_SEC;
  printf("Tempo levou para alocar espaco para a matriz foi de %f\n",time_spent);
  /* ---- read image data into matrix ---- */
  begin = clock();
  for (i=0; i<PGMImage->x; i++)
  for (j=0; j<PGMImage->y; j++)
matrix[i][j] = (float) *(PGMImage->imageData + (i*PGMImage->y) + j); 

end = clock();
time_spent = (double)(end - begin)/CLOCKS_PER_SEC;
printf("Tempo que levou para ler da imagem para a matriz foi de %f\n",time_spent);
/* ---- process image ---- */
  begin = clock();
  printf("contrast paramter lambda (>0) : ");
  //~ gets(row);  sscanf(row, "%f", &lambda);
  scanf("%f", &lambda);
  printf("number of iterations: ");
  //~ gets(row);  sscanf(row, "%ld", &imax);
  scanf("%ld", &imax);
  for (i=1; i<=imax; i++)
  {
    printf("iteration number: %3ld \n", i);
    diff2d (0.5, lambda, PGMImage->x, PGMImage->y, matrix); 
  }
  end = clock();
  time_spent = (double)(end - begin)/CLOCKS_PER_SEC;
  printf("Tempo que levou para processar o filtro da imagem foi de %f\n",time_spent);
  /* copy the Result Image to PGM Image/File structure */
    begin = clock();
    for (i=0; i<PGMImage->x; i++)
    for (j=0; j<PGMImage->y; j++)
  *(PGMImage->imageData + i*PGMImage->y + j) = (char) matrix[i][j];

end = clock();
time_spent = (double)(end - begin)/CLOCKS_PER_SEC;
printf("Tempo que levou para copiar o resultado da imagem para a estrutura de arquivo foi de %f\n",time_spent);
  /* ---- write image ---- */
      begin = clock();
      if (!argv[2])
      {
        printf("name of output PGM image file (with extender): ");
        scanf("%s", PGMImage->fileName);
      }
  else
  {
    strcpy(PGMImage->fileName, argv[2]);
  }

  write8bitPGM(PGMImage);
  
  end = clock();
  time_spent = (double)(end - begin)/CLOCKS_PER_SEC;
  printf("Tempo que levou para escrever a imagem de volta foi de %f\n",time_spent);
  /* ---- disallocate storage ---- */
    begin = clock();
    for (i=0; i<PGMImage->x; i++)
    free(matrix[i]);
  free(matrix);
  
  free(PGMImage->imageData);
  free(PGMImage);
  end = clock();
  time_spent = (double)(end - begin)/CLOCKS_PER_SEC;
  printf("Tempo que levou para liberar o armazenamento da matriz foi de %f\n",time_spent);
}