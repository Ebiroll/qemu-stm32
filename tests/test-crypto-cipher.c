/*
 * QEMU Crypto cipher algorithms
 *
 * Copyright (c) 2015 Red Hat, Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "qemu/osdep.h"

#include "crypto/init.h"
#include "crypto/cipher.h"
#include "qapi/error.h"

typedef struct QCryptoCipherTestData QCryptoCipherTestData;
struct QCryptoCipherTestData {
    const char *path;
    QCryptoCipherAlgorithm alg;
    QCryptoCipherMode mode;
    const char *key;
    const char *plaintext;
    const char *ciphertext;
    const char *iv;
};

/* AES test data comes from appendix F of:
 *
 * http://csrc.nist.gov/publications/nistpubs/800-38a/sp800-38a.pdf
 */
static QCryptoCipherTestData test_data[] = {
    {
        /* NIST F.1.1 ECB-AES128.Encrypt */
        .path = "/crypto/cipher/aes-ecb-128",
        .alg = QCRYPTO_CIPHER_ALG_AES_128,
        .mode = QCRYPTO_CIPHER_MODE_ECB,
        .key = "2b7e151628aed2a6abf7158809cf4f3c",
        .plaintext =
            "6bc1bee22e409f96e93d7e117393172a"
            "ae2d8a571e03ac9c9eb76fac45af8e51"
            "30c81c46a35ce411e5fbc1191a0a52ef"
            "f69f2445df4f9b17ad2b417be66c3710",
        .ciphertext =
            "3ad77bb40d7a3660a89ecaf32466ef97"
            "f5d3d58503b9699de785895a96fdbaaf"
            "43b1cd7f598ece23881b00e3ed030688"
            "7b0c785e27e8ad3f8223207104725dd4"
    },
    {
        /* NIST F.1.3 ECB-AES192.Encrypt */
        .path = "/crypto/cipher/aes-ecb-192",
        .alg = QCRYPTO_CIPHER_ALG_AES_192,
        .mode = QCRYPTO_CIPHER_MODE_ECB,
        .key = "8e73b0f7da0e6452c810f32b809079e562f8ead2522c6b7b",
        .plaintext  =
            "6bc1bee22e409f96e93d7e117393172a"
            "ae2d8a571e03ac9c9eb76fac45af8e51"
            "30c81c46a35ce411e5fbc1191a0a52ef"
            "f69f2445df4f9b17ad2b417be66c3710",
        .ciphertext =
            "bd334f1d6e45f25ff712a214571fa5cc"
            "974104846d0ad3ad7734ecb3ecee4eef"
            "ef7afd2270e2e60adce0ba2face6444e"
            "9a4b41ba738d6c72fb16691603c18e0e"
    },
    {
        /* NIST F.1.5 ECB-AES256.Encrypt */
        .path = "/crypto/cipher/aes-ecb-256",
        .alg = QCRYPTO_CIPHER_ALG_AES_256,
        .mode = QCRYPTO_CIPHER_MODE_ECB,
        .key =
            "603deb1015ca71be2b73aef0857d7781"
            "1f352c073b6108d72d9810a30914dff4",
        .plaintext  =
            "6bc1bee22e409f96e93d7e117393172a"
            "ae2d8a571e03ac9c9eb76fac45af8e51"
            "30c81c46a35ce411e5fbc1191a0a52ef"
            "f69f2445df4f9b17ad2b417be66c3710",
        .ciphertext =
            "f3eed1bdb5d2a03c064b5a7e3db181f8"
            "591ccb10d410ed26dc5ba74a31362870"
            "b6ed21b99ca6f4f9f153e7b1beafed1d"
            "23304b7a39f9f3ff067d8d8f9e24ecc7",
    },
    {
        /* NIST F.2.1 CBC-AES128.Encrypt */
        .path = "/crypto/cipher/aes-cbc-128",
        .alg = QCRYPTO_CIPHER_ALG_AES_128,
        .mode = QCRYPTO_CIPHER_MODE_CBC,
        .key = "2b7e151628aed2a6abf7158809cf4f3c",
        .iv = "000102030405060708090a0b0c0d0e0f",
        .plaintext  =
            "6bc1bee22e409f96e93d7e117393172a"
            "ae2d8a571e03ac9c9eb76fac45af8e51"
            "30c81c46a35ce411e5fbc1191a0a52ef"
            "f69f2445df4f9b17ad2b417be66c3710",
        .ciphertext =
            "7649abac8119b246cee98e9b12e9197d"
            "5086cb9b507219ee95db113a917678b2"
            "73bed6b8e3c1743b7116e69e22229516"
            "3ff1caa1681fac09120eca307586e1a7",
    },
    {
        /* NIST F.2.3 CBC-AES128.Encrypt */
        .path = "/crypto/cipher/aes-cbc-192",
        .alg = QCRYPTO_CIPHER_ALG_AES_192,
        .mode = QCRYPTO_CIPHER_MODE_CBC,
        .key = "8e73b0f7da0e6452c810f32b809079e562f8ead2522c6b7b",
        .iv = "000102030405060708090a0b0c0d0e0f",
        .plaintext  =
            "6bc1bee22e409f96e93d7e117393172a"
            "ae2d8a571e03ac9c9eb76fac45af8e51"
            "30c81c46a35ce411e5fbc1191a0a52ef"
            "f69f2445df4f9b17ad2b417be66c3710",
        .ciphertext =
            "4f021db243bc633d7178183a9fa071e8"
            "b4d9ada9ad7dedf4e5e738763f69145a"
            "571b242012fb7ae07fa9baac3df102e0"
            "08b0e27988598881d920a9e64f5615cd",
    },
    {
        /* NIST F.2.5 CBC-AES128.Encrypt */
        .path = "/crypto/cipher/aes-cbc-256",
        .alg = QCRYPTO_CIPHER_ALG_AES_256,
        .mode = QCRYPTO_CIPHER_MODE_CBC,
        .key =
            "603deb1015ca71be2b73aef0857d7781"
            "1f352c073b6108d72d9810a30914dff4",
        .iv = "000102030405060708090a0b0c0d0e0f",
        .plaintext  =
            "6bc1bee22e409f96e93d7e117393172a"
            "ae2d8a571e03ac9c9eb76fac45af8e51"
            "30c81c46a35ce411e5fbc1191a0a52ef"
            "f69f2445df4f9b17ad2b417be66c3710",
        .ciphertext =
            "f58c4c04d6e5f1ba779eabfb5f7bfbd6"
            "9cfc4e967edb808d679f777bc6702c7d"
            "39f23369a9d9bacfa530e26304231461"
            "b2eb05e2c39be9fcda6c19078c6a9d1b",
    },
    {
        .path = "/crypto/cipher/des-rfb-ecb-56",
        .alg = QCRYPTO_CIPHER_ALG_DES_RFB,
        .mode = QCRYPTO_CIPHER_MODE_ECB,
        .key = "0123456789abcdef",
        .plaintext =
            "6bc1bee22e409f96e93d7e117393172a"
            "ae2d8a571e03ac9c9eb76fac45af8e51"
            "30c81c46a35ce411e5fbc1191a0a52ef"
            "f69f2445df4f9b17ad2b417be66c3710",
        .ciphertext =
            "8f346aaf64eaf24040720d80648c52e7"
            "aefc616be53ab1a3d301e69d91e01838"
            "ffd29f1bb5596ad94ea2d8e6196b7f09"
            "30d8ed0bf2773af36dd82a6280c20926",
    },
    {
        /* RFC 2144, Appendix B.1 */
        .path = "/crypto/cipher/cast5-128",
        .alg = QCRYPTO_CIPHER_ALG_CAST5_128,
        .mode = QCRYPTO_CIPHER_MODE_ECB,
        .key = "0123456712345678234567893456789A",
        .plaintext = "0123456789abcdef",
        .ciphertext = "238b4fe5847e44b2",
    },
    {
        /* libgcrypt serpent.c */
        .path = "/crypto/cipher/serpent-128",
        .alg = QCRYPTO_CIPHER_ALG_SERPENT_128,
        .mode = QCRYPTO_CIPHER_MODE_ECB,
        .key = "00000000000000000000000000000000",
        .plaintext = "d29d576fcea3a3a7ed9099f29273d78e",
        .ciphertext = "b2288b968ae8b08648d1ce9606fd992d",
    },
    {
        /* libgcrypt serpent.c */
        .path = "/crypto/cipher/serpent-192",
        .alg = QCRYPTO_CIPHER_ALG_SERPENT_192,
        .mode = QCRYPTO_CIPHER_MODE_ECB,
        .key = "00000000000000000000000000000000"
               "0000000000000000",
        .plaintext = "d29d576fceaba3a7ed9899f2927bd78e",
        .ciphertext = "130e353e1037c22405e8faefb2c3c3e9",
    },
    {
        /* libgcrypt serpent.c */
        .path = "/crypto/cipher/serpent-256a",
        .alg = QCRYPTO_CIPHER_ALG_SERPENT_256,
        .mode = QCRYPTO_CIPHER_MODE_ECB,
        .key = "00000000000000000000000000000000"
               "00000000000000000000000000000000",
        .plaintext = "d095576fcea3e3a7ed98d9f29073d78e",
        .ciphertext = "b90ee5862de69168f2bdd5125b45472b",
    },
    {
        /* libgcrypt serpent.c */
        .path = "/crypto/cipher/serpent-256b",
        .alg = QCRYPTO_CIPHER_ALG_SERPENT_256,
        .mode = QCRYPTO_CIPHER_MODE_ECB,
        .key = "00000000000000000000000000000000"
               "00000000000000000000000000000000",
        .plaintext = "00000000010000000200000003000000",
        .ciphertext = "2061a42782bd52ec691ec383b03ba77c",
    },
    {
        /* Twofish paper "Known Answer Test" */
        .path = "/crypto/cipher/twofish-128",
        .alg = QCRYPTO_CIPHER_ALG_TWOFISH_128,
        .mode = QCRYPTO_CIPHER_MODE_ECB,
        .key = "d491db16e7b1c39e86cb086b789f5419",
        .plaintext = "019f9809de1711858faac3a3ba20fbc3",
        .ciphertext = "6363977de839486297e661c6c9d668eb",
    },
    {
        /* Twofish paper "Known Answer Test", I=3 */
        .path = "/crypto/cipher/twofish-192",
        .alg = QCRYPTO_CIPHER_ALG_TWOFISH_192,
        .mode = QCRYPTO_CIPHER_MODE_ECB,
        .key = "88b2b2706b105e36b446bb6d731a1e88"
               "efa71f788965bd44",
        .plaintext = "39da69d6ba4997d585b6dc073ca341b2",
        .ciphertext = "182b02d81497ea45f9daacdc29193a65",
    },
    {
        /* Twofish paper "Known Answer Test", I=4 */
        .path = "/crypto/cipher/twofish-256",
        .alg = QCRYPTO_CIPHER_ALG_TWOFISH_256,
        .mode = QCRYPTO_CIPHER_MODE_ECB,
        .key = "d43bb7556ea32e46f2a282b7d45b4e0d"
               "57ff739d4dc92c1bd7fc01700cc8216f",
        .plaintext = "90afe91bb288544f2c32dc239b2635e6",
        .ciphertext = "6cb4561c40bf0a9705931cb6d408e7fa",
    },
    {
        /* #1 32 byte key, 32 byte PTX */
        .path = "/crypto/cipher/aes-xts-128-1",
        .alg = QCRYPTO_CIPHER_ALG_AES_128,
        .mode = QCRYPTO_CIPHER_MODE_XTS,
        .key =
            "00000000000000000000000000000000"
            "00000000000000000000000000000000",
        .iv =
            "00000000000000000000000000000000",
        .plaintext =
            "00000000000000000000000000000000"
            "00000000000000000000000000000000",
        .ciphertext =
            "917cf69ebd68b2ec9b9fe9a3eadda692"
            "cd43d2f59598ed858c02c2652fbf922e",
    },
    {
        /* #2, 32 byte key, 32 byte PTX */
        .path = "/crypto/cipher/aes-xts-128-2",
        .alg = QCRYPTO_CIPHER_ALG_AES_128,
        .mode = QCRYPTO_CIPHER_MODE_XTS,
        .key =
            "11111111111111111111111111111111"
            "22222222222222222222222222222222",
        .iv =
            "33333333330000000000000000000000",
        .plaintext =
            "44444444444444444444444444444444"
            "44444444444444444444444444444444",
        .ciphertext =
            "c454185e6a16936e39334038acef838b"
            "fb186fff7480adc4289382ecd6d394f0",
    },
    {
        /* #5 from xts.7, 32 byte key, 32 byte PTX */
        .path = "/crypto/cipher/aes-xts-128-3",
        .alg = QCRYPTO_CIPHER_ALG_AES_128,
        .mode = QCRYPTO_CIPHER_MODE_XTS,
        .key =
            "fffefdfcfbfaf9f8f7f6f5f4f3f2f1f0"
            "bfbebdbcbbbab9b8b7b6b5b4b3b2b1b0",
        .iv =
            "9a785634120000000000000000000000",
        .plaintext =
            "44444444444444444444444444444444"
            "44444444444444444444444444444444",
        .ciphertext =
            "b01f86f8edc1863706fa8a4253e34f28"
            "af319de38334870f4dd1f94cbe9832f1",
    },
    {
        /* #4, 32 byte key, 512 byte PTX  */
        .path = "/crypto/cipher/aes-xts-128-4",
        .alg = QCRYPTO_CIPHER_ALG_AES_128,
        .mode = QCRYPTO_CIPHER_MODE_XTS,
        .key =
            "27182818284590452353602874713526"
            "31415926535897932384626433832795",
        .iv =
            "00000000000000000000000000000000",
        .plaintext =
            "000102030405060708090a0b0c0d0e0f"
            "101112131415161718191a1b1c1d1e1f"
            "202122232425262728292a2b2c2d2e2f"
            "303132333435363738393a3b3c3d3e3f"
            "404142434445464748494a4b4c4d4e4f"
            "505152535455565758595a5b5c5d5e5f"
            "606162636465666768696a6b6c6d6e6f"
            "707172737475767778797a7b7c7d7e7f"
            "808182838485868788898a8b8c8d8e8f"
            "909192939495969798999a9b9c9d9e9f"
            "a0a1a2a3a4a5a6a7a8a9aaabacadaeaf"
            "b0b1b2b3b4b5b6b7b8b9babbbcbdbebf"
            "c0c1c2c3c4c5c6c7c8c9cacbcccdcecf"
            "d0d1d2d3d4d5d6d7d8d9dadbdcdddedf"
            "e0e1e2e3e4e5e6e7e8e9eaebecedeeef"
            "f0f1f2f3f4f5f6f7f8f9fafbfcfdfeff"
            "000102030405060708090a0b0c0d0e0f"
            "101112131415161718191a1b1c1d1e1f"
            "202122232425262728292a2b2c2d2e2f"
            "303132333435363738393a3b3c3d3e3f"
            "404142434445464748494a4b4c4d4e4f"
            "505152535455565758595a5b5c5d5e5f"
            "606162636465666768696a6b6c6d6e6f"
            "707172737475767778797a7b7c7d7e7f"
            "808182838485868788898a8b8c8d8e8f"
            "909192939495969798999a9b9c9d9e9f"
            "a0a1a2a3a4a5a6a7a8a9aaabacadaeaf"
            "b0b1b2b3b4b5b6b7b8b9babbbcbdbebf"
            "c0c1c2c3c4c5c6c7c8c9cacbcccdcecf"
            "d0d1d2d3d4d5d6d7d8d9dadbdcdddedf"
            "e0e1e2e3e4e5e6e7e8e9eaebecedeeef"
            "f0f1f2f3f4f5f6f7f8f9fafbfcfdfeff",
        .ciphertext =
            "27a7479befa1d476489f308cd4cfa6e2"
            "a96e4bbe3208ff25287dd3819616e89c"
            "c78cf7f5e543445f8333d8fa7f560000"
            "05279fa5d8b5e4ad40e736ddb4d35412"
            "328063fd2aab53e5ea1e0a9f332500a5"
            "df9487d07a5c92cc512c8866c7e860ce"
            "93fdf166a24912b422976146ae20ce84"
            "6bb7dc9ba94a767aaef20c0d61ad0265"
            "5ea92dc4c4e41a8952c651d33174be51"
            "a10c421110e6d81588ede82103a252d8"
            "a750e8768defffed9122810aaeb99f91"
            "72af82b604dc4b8e51bcb08235a6f434"
            "1332e4ca60482a4ba1a03b3e65008fc5"
            "da76b70bf1690db4eae29c5f1badd03c"
            "5ccf2a55d705ddcd86d449511ceb7ec3"
            "0bf12b1fa35b913f9f747a8afd1b130e"
            "94bff94effd01a91735ca1726acd0b19"
            "7c4e5b03393697e126826fb6bbde8ecc"
            "1e08298516e2c9ed03ff3c1b7860f6de"
            "76d4cecd94c8119855ef5297ca67e9f3"
            "e7ff72b1e99785ca0a7e7720c5b36dc6"
            "d72cac9574c8cbbc2f801e23e56fd344"
            "b07f22154beba0f08ce8891e643ed995"
            "c94d9a69c9f1b5f499027a78572aeebd"
            "74d20cc39881c213ee770b1010e4bea7"
            "18846977ae119f7a023ab58cca0ad752"
            "afe656bb3c17256a9f6e9bf19fdd5a38"
            "fc82bbe872c5539edb609ef4f79c203e"
            "bb140f2e583cb2ad15b4aa5b655016a8"
            "449277dbd477ef2c8d6c017db738b18d"
            "eb4a427d1923ce3ff262735779a418f2"
            "0a282df920147beabe421ee5319d0568",
    },
    {
        /* Bad config - cast5-128 has 8 byte block size
         * which is incompatible with XTS
         */
        .path = "/crypto/cipher/cast5-xts-128",
        .alg = QCRYPTO_CIPHER_ALG_CAST5_128,
        .mode = QCRYPTO_CIPHER_MODE_XTS,
        .key =
            "27182818284590452353602874713526"
            "31415926535897932384626433832795",
    }
};


static inline int unhex(char c)
{
    if (c >= 'a' && c <= 'f') {
        return 10 + (c - 'a');
    }
    if (c >= 'A' && c <= 'F') {
        return 10 + (c - 'A');
    }
    return c - '0';
}

static inline char hex(int i)
{
    if (i < 10) {
        return '0' + i;
    }
    return 'a' + (i - 10);
}

static size_t unhex_string(const char *hexstr,
                           uint8_t **data)
{
    size_t len;
    size_t i;

    if (!hexstr) {
        *data = NULL;
        return 0;
    }

    len = strlen(hexstr);
    *data = g_new0(uint8_t, len / 2);

    for (i = 0; i < len; i += 2) {
        (*data)[i/2] = (unhex(hexstr[i]) << 4) | unhex(hexstr[i+1]);
    }
    return len / 2;
}

static char *hex_string(const uint8_t *bytes,
                        size_t len)
{
    char *hexstr = g_new0(char, len * 2 + 1);
    size_t i;

    for (i = 0; i < len; i++) {
        hexstr[i*2] = hex((bytes[i] >> 4) & 0xf);
        hexstr[i*2+1] = hex(bytes[i] & 0xf);
    }
    hexstr[len*2] = '\0';

    return hexstr;
}

static void test_cipher(const void *opaque)
{
    const QCryptoCipherTestData *data = opaque;

    QCryptoCipher *cipher;
    uint8_t *key, *iv = NULL, *ciphertext = NULL,
        *plaintext = NULL, *outtext = NULL;
    size_t nkey, niv = 0, nciphertext = 0, nplaintext = 0;
    char *outtexthex = NULL;
    size_t ivsize, keysize, blocksize;
    Error *err = NULL;

    nkey = unhex_string(data->key, &key);
    if (data->iv) {
        niv = unhex_string(data->iv, &iv);
    }
    if (data->ciphertext) {
        nciphertext = unhex_string(data->ciphertext, &ciphertext);
    }
    if (data->plaintext) {
        nplaintext = unhex_string(data->plaintext, &plaintext);
    }

    g_assert(nciphertext == nplaintext);

    outtext = g_new0(uint8_t, nciphertext);

    cipher = qcrypto_cipher_new(
        data->alg, data->mode,
        key, nkey,
        &err);
    if (data->plaintext) {
        g_assert(err == NULL);
        g_assert(cipher != NULL);
    } else {
        error_free_or_abort(&err);
        g_assert(cipher == NULL);
        goto cleanup;
    }

    keysize = qcrypto_cipher_get_key_len(data->alg);
    blocksize = qcrypto_cipher_get_block_len(data->alg);
    ivsize = qcrypto_cipher_get_iv_len(data->alg, data->mode);

    if (data->mode == QCRYPTO_CIPHER_MODE_XTS) {
        g_assert_cmpint(keysize * 2, ==, nkey);
    } else {
        g_assert_cmpint(keysize, ==, nkey);
    }
    g_assert_cmpint(ivsize, ==, niv);
    if (niv) {
        g_assert_cmpint(blocksize, ==, niv);
    }

    if (iv) {
        g_assert(qcrypto_cipher_setiv(cipher,
                                      iv, niv,
                                      &error_abort) == 0);
    }
    g_assert(qcrypto_cipher_encrypt(cipher,
                                    plaintext,
                                    outtext,
                                    nplaintext,
                                    &error_abort) == 0);

    outtexthex = hex_string(outtext, nciphertext);

    g_assert_cmpstr(outtexthex, ==, data->ciphertext);

    g_free(outtexthex);

    if (iv) {
        g_assert(qcrypto_cipher_setiv(cipher,
                                      iv, niv,
                                      &error_abort) == 0);
    }
    g_assert(qcrypto_cipher_decrypt(cipher,
                                    ciphertext,
                                    outtext,
                                    nplaintext,
                                    &error_abort) == 0);

    outtexthex = hex_string(outtext, nplaintext);

    g_assert_cmpstr(outtexthex, ==, data->plaintext);

 cleanup:
    g_free(outtext);
    g_free(outtexthex);
    g_free(key);
    g_free(iv);
    g_free(ciphertext);
    g_free(plaintext);
    qcrypto_cipher_free(cipher);
}


static void test_cipher_null_iv(void)
{
    QCryptoCipher *cipher;
    uint8_t key[32] = { 0 };
    uint8_t plaintext[32] = { 0 };
    uint8_t ciphertext[32] = { 0 };

    cipher = qcrypto_cipher_new(
        QCRYPTO_CIPHER_ALG_AES_256,
        QCRYPTO_CIPHER_MODE_CBC,
        key, sizeof(key),
        &error_abort);
    g_assert(cipher != NULL);

    /* Don't call qcrypto_cipher_setiv */

    qcrypto_cipher_encrypt(cipher,
                           plaintext,
                           ciphertext,
                           sizeof(plaintext),
                           &error_abort);

    qcrypto_cipher_free(cipher);
}

static void test_cipher_short_plaintext(void)
{
    Error *err = NULL;
    QCryptoCipher *cipher;
    uint8_t key[32] = { 0 };
    uint8_t plaintext1[20] = { 0 };
    uint8_t ciphertext1[20] = { 0 };
    uint8_t plaintext2[40] = { 0 };
    uint8_t ciphertext2[40] = { 0 };
    int ret;

    cipher = qcrypto_cipher_new(
        QCRYPTO_CIPHER_ALG_AES_256,
        QCRYPTO_CIPHER_MODE_CBC,
        key, sizeof(key),
        &error_abort);
    g_assert(cipher != NULL);

    /* Should report an error as plaintext is shorter
     * than block size
     */
    ret = qcrypto_cipher_encrypt(cipher,
                                 plaintext1,
                                 ciphertext1,
                                 sizeof(plaintext1),
                                 &err);
    g_assert(ret == -1);
    g_assert(err != NULL);

    error_free(err);
    err = NULL;

    /* Should report an error as plaintext is larger than
     * block size, but not a multiple of block size
     */
    ret = qcrypto_cipher_encrypt(cipher,
                                 plaintext2,
                                 ciphertext2,
                                 sizeof(plaintext2),
                                 &err);
    g_assert(ret == -1);
    g_assert(err != NULL);

    error_free(err);
    qcrypto_cipher_free(cipher);
}

int main(int argc, char **argv)
{
    size_t i;

    g_test_init(&argc, &argv, NULL);

    g_assert(qcrypto_init(NULL) == 0);

    for (i = 0; i < G_N_ELEMENTS(test_data); i++) {
        if (qcrypto_cipher_supports(test_data[i].alg, test_data[i].mode)) {
            g_test_add_data_func(test_data[i].path, &test_data[i], test_cipher);
        }
    }

    g_test_add_func("/crypto/cipher/null-iv",
                    test_cipher_null_iv);

    g_test_add_func("/crypto/cipher/short-plaintext",
                    test_cipher_short_plaintext);

    return g_test_run();
}
