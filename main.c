#include <stdio.h>
#include <stdlib.h>

unsigned long long nFunc(unsigned int n);

int main() {
    int n;

    printf("please enter a number: ");
    scanf("%d", &n);

    printf("the sum of the 1 to the %d: %llu\n", n, nFunc(n));

    return 0;
}

unsigned long long nFunc(unsigned int number) {
    if (number == 0) {
        return 0;
    } else {
        return (number + nFunc(number - 1));
    }
}
