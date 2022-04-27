#include "githash.h"

// addrから始まるアドレスに13+\0の14Byteハッシュ値を書き込む
void getGithash(char* addr) {
    char ret[14] = GIT_HASH;
    for (int i = 0; i < 13; ++i) {
        addr[i] = ret[i];
    }
}
