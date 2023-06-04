#include "taniho.h"

#include <cstdio>

// githash.cpp
void getGithash(char* addr);

int main(void) {
    // 各モジュールの初期化

    // 各モジュールをタイマーイベントに登録

    // 起動音再生

    // アクティビティの開始

    char hash[14];

    getGithash(hash);

    printf("%s\n", hash);

    // while (1) {
    // }
    return 0;
}
