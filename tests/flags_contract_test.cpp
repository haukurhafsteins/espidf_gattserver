#include <cstdint>
#include <type_traits>

#include "gattserver.h"

static_assert(sizeof(gatt_chr_flags_t) == sizeof(uint32_t));
static_assert(GATT_CHR_F_READ_ENC == 0x00000200u);
static_assert(GATT_CHR_F_WRITE_ENC == 0x00001000u);
static_assert((GATT_CHR_PROP_WRITE | GATT_CHR_F_WRITE_ENC) == 0x00001008u);

static_assert(GATT_WRITE_OK == 0x00);
static_assert(GATT_WRITE_ERR_UNLIKELY == 0x0e);
static_assert(GATT_WRITE_ERR_INSUFFICIENT_RESOURCES == 0x11);

static gatt_write_status_t statusWrite(
    gatt_param_handle_t, void *, size_t)
{
    return GATT_WRITE_OK;
}

static_assert(std::is_same_v<decltype(&statusWrite), gatt_write_status_cb_t>);
static_assert(std::is_same_v<
              decltype(&gattserver_register_write_status_cb),
              esp_err_t (*)(gatt_param_handle_t, gatt_write_status_cb_t)>);

int main()
{
    return 0;
}
