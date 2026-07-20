#include <cstdint>

#include "gattserver.h"

static_assert(sizeof(gatt_chr_flags_t) == sizeof(uint32_t));
static_assert(GATT_CHR_F_READ_ENC == 0x00000200u);
static_assert(GATT_CHR_F_WRITE_ENC == 0x00001000u);
static_assert((GATT_CHR_PROP_WRITE | GATT_CHR_F_WRITE_ENC) == 0x00001008u);

int main()
{
    return 0;
}
