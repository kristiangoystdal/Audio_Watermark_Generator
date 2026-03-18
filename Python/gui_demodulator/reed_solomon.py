NSYM = 20
PRIM = 0x11D
FCR = 0
GENERATOR = 2
C_EXP = 8

def build_codec():
    """Create an RSCodec instance using the project defaults."""
    from reedsolo import RSCodec

    return RSCodec(
        nsym=NSYM,
        nsize=255,
        fcr=FCR,
        prim=PRIM,
        generator=GENERATOR,
        c_exp=C_EXP,
    )


def build_codec_with_nsym(nsym):
    """Create an RSCodec instance with caller-specified parity bytes."""
    from reedsolo import RSCodec

    return RSCodec(
        nsym=nsym,
        nsize=255,
        fcr=FCR,
        prim=PRIM,
        generator=GENERATOR,
        c_exp=C_EXP,
    )


def decode_codeword(codeword, codec=None):
    """Decode one RS codeword and return only the corrected payload bytes."""
    rsc = codec if codec is not None else build_codec()
    result = rsc.decode(codeword)

    # reedsolo versions may return either bytes or a tuple
    if isinstance(result, tuple):
        return result[0]
    return result


if __name__ == "__main__":
    # Same codeword produced by the C program
    codeword = bytes.fromhex("48 45 4C 4C 4F A4 4F 66 0E 94 44 59 48")
    rsc = build_codec()

    print("Original codeword:", codeword.hex(" "))

    # Try with a couple of corrupted bytes
    corrupted = bytearray(codeword)
    corrupted[2] ^= 0x55
    corrupted[8] ^= 0x99

    print("Corrupted codeword:", bytes(corrupted).hex(" "))

    decoded_msg = decode_codeword(bytes(corrupted), codec=rsc)
    print("Decoded message:", decoded_msg)
    print("Decoded text:", decoded_msg.decode("ascii"))
