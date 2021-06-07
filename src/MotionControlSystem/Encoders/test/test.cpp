#include "../PinMaskDefines.h"
#include <assert.h>

#ifdef TEST

int main() {
    assert (pin_mask(A5) == 0x0040);
    assert (pin_mask(D2) == 0x1000);
    assert (pin_mask(D3) == 0x0001);
    assert (pin_mask(A4) == 0x0020);
    assert (get_shift(A5, FORWARD) == 6);
    assert (get_shift(D2, BACKWARD) == 11);
    assert (get_shift(D3, FORWARD) == 0);
    assert (get_shift(A4, BACKWARD) == 4);
    return 0;
}

#endif