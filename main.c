#include <stdio.h>
#define MAX(a,b) (a)>(b)?(a):(b) 
#define SQUARE(x) x*x 
#define DOUBLE(x) x+x 
#define SUM( value ) ( ( value ) + ( value ) ) 
int main() 
{ 
int x/*blah blah*/,y; 
int a[5]= {1,2,3,4,5};
int i = 0;
printf("%d",SUM(a[i++]));


}