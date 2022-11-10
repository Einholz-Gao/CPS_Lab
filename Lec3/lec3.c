#include <stdio.h>
#include <stdlib.h>

// int main() {
//    printf("Hello World!");
//    return 0;
// }
int main(){
    char *p;
    *p ='a';

    printf("p = %c \n",*p);
    p = malloc(10);// Memory leak. create a dynamic memory for 'p', and use free() to release.
    printf("p = %c \n",*p);


    // dynamic Memory
    // int* q = (int*) malloc (10*sizeof(int));// parameter is how many memory you want.
    // same as int x[10];  these arae two ways to allocate.
    // q[1]=1;
    // printf("p = %i \n",*q);
    free(p);//release the memory.

    // int* x;
    //
    // int x[5];  //located adress? 5
    // printf("%d \n",&x);


    /*Strings:
    //different with char
    string: use ""
    char: use ''

    BUT string equal to char[]???
    */
}
