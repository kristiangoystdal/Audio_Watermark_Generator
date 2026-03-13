#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define PRIM 0x11d
#define GF_SIZE 256
#define GF_MAX 255
#define GENERATOR 2
#define FCR 0


static uint8_t gf_exp[512];
static uint8_t gf_log[256];

static void gf_init(void) {
    uint16_t x = 1;
    for (int i = 0; i < GF_MAX; i++) {
        gf_exp[i] = (uint8_t)x;
        gf_log[x] = (uint8_t)i;
        x <<= 1;
        if (x & 0x100) {
            x ^= PRIM;
        }
    }

    for (int i = GF_MAX; i < 512; i++) {
        gf_exp[i] = gf_exp[i - GF_MAX];
    }

    gf_log[0] = 0;
}

static uint8_t gf_mul(uint8_t a, uint8_t b) {
    if (a == 0 || b == 0) {
        return 0;
    }
    return gf_exp[gf_log[a] + gf_log[b]];
}

static void poly_mul(const uint8_t *p, int p_len,
                     const uint8_t *q, int q_len,
                     uint8_t *out) {
    memset(out, 0, (p_len + q_len - 1) * sizeof(uint8_t));

    for (int i = 0; i < p_len; i++) {
        for (int j = 0; j < q_len; j++) {
            out[i + j] ^= gf_mul(p[i], q[j]);
        }
    }
}

void rs_generator_poly(uint8_t *gen, int nsym) {
    uint8_t g[nsym + 1];
    uint8_t temp[nsym + 1];

    memset(g, 0, (nsym + 1) * sizeof(uint8_t));
    memset(temp, 0, (nsym + 1) * sizeof(uint8_t));

    g[0] = 1;
    int g_len = 1;

    for (int i = 0; i < nsym; i++) {
        uint8_t factor[2];
        factor[0] = 1;
        factor[1] = gf_exp[(FCR + i) % GF_MAX];

        memset(temp, 0, sizeof(temp));
        poly_mul(g, g_len, factor, 2, temp);

        g_len += 1;
        memcpy(g, temp, g_len);
    }

    memcpy(gen, g, nsym + 1);
}

void rs_encode_msg(const uint8_t *msg, int msg_len,
                          uint8_t *codeword, int nsym) {
    gf_init();

    uint8_t gen[nsym + 1];
    memset(gen, 0, (nsym + 1) * sizeof(uint8_t));
    rs_generator_poly(gen, nsym);

    memcpy(codeword, msg, msg_len);
    memset(codeword + msg_len, 0, nsym);

    for (int i = 0; i < msg_len; i++) {
        uint8_t coef = codeword[i];
        if (coef != 0) {
            for (int j = 1; j < nsym + 1; j++) {
                codeword[i + j] ^= gf_mul(gen[j], coef);
            }
        }
    }

    memcpy(codeword, msg, msg_len);
}

// int main(void) {
//     gf_init();

//     const uint8_t msg[] = { 'H', 'E', 'L', 'L', 'O' };
//     const int msg_len = (int)sizeof(msg);
//     uint8_t codeword[msg_len + NSYM];

//     rs_encode_msg(msg, msg_len, codeword, NSYM);

//     printf("Message: ");
//     for (int i = 0; i < msg_len; i++) {
//         printf("%c", msg[i]);
//     }
//     printf("\n");

//     printf("Codeword hex: ");
//     for (int i = 0; i < msg_len + NSYM; i++) {
//         printf("%02X", codeword[i]);
//         if (i < msg_len + NSYM - 1) {
//             printf(" ");
//         }
//     }
//     printf("\n");

//     return 0;
// }