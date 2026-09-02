#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
clock_t begin, end;
double time_spent;
void func1(){
    int x, y, z;

    y = 3;
    z = 4;
    x = y+z;
}

int func2(){
    int x, y, z;

    y = 3;
    z = 4;
    x = y+z;

    return x;
}

char func3(char param){
    char x, y, z;
    if(param = 'a'){
        y = 3;
        z = 4;
        x = y+z;
    }else{
        y = 5;
        z = 5;
        x = y+z;
    }
    if(x == 10){
        return 'a';
    }else{
        return 'b';
    }
}


void func4(float param){
    printf("\n The parameter is: %.2f", param);
}

short func5(){
    int temp_param;
    printf("\nValue to temp_param will be 15: ");
    temp_param = 15;

    return ((short)temp_param);

}

int main(){
    char param_1 = 'b';
    short param_2;
    int param_3;
    float param_4 = 50.0;
    int n = 100;
    begin = clock();
    for(int i = 0; i<n;i++){
        func1();
        param_3 = func2();
        param_1 = func3(param_1);
        func4(param_4);
        param_2 = func5();
    }
    end = clock();
    time_spent = (double)(end - begin)/CLOCKS_PER_SEC;
    printf("\nNumero de interacoes: %d\n", n);
    printf("Tempo levou foi de %f\n",time_spent);
}