#include <stdio.h>

unsigned long long sumFunc(unsigned int num);

int main() {
    int num;
    printf("please enter a number: ");
    scanf("%d", &num);

    printf("the sum of the digits the %d : %llu\n", num, sumFunc(num));

    return 0;
}

unsigned long long sumFunc(unsigned int num) {
    if (num == 0) {
        return 0;
    } else {
        return (num % 10 + sumFunc(num / 10));
    }
}
