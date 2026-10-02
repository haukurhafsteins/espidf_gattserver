import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class ZephyrBackendContractTest(unittest.TestCase):
    def test_long_write_capacity_is_configured_and_enforced(self) -> None:
        kconfig = (ROOT / "zephyr" / "Kconfig").read_text(encoding="utf-8")
        source = (ROOT / "backends" / "zephyr" / "gattserver_zephyr.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("default 4 if GATTSERVER", kconfig)
        self.assertIn("CONFIG_BT_ATT_PREPARE_COUNT >= 4", source)
        self.assertIn("BT_GATT_WRITE_FLAG_PREPARE", source)
        self.assertIn("BT_GATT_WRITE_FLAG_EXECUTE", source)
        self.assertIn("k_work_submit(&param->write_callback_work)", source)

    def test_value_storage_uses_a_configured_static_arena(self) -> None:
        kconfig = (ROOT / "zephyr" / "Kconfig").read_text(encoding="utf-8")
        source = (ROOT / "backends" / "zephyr" / "gattserver_zephyr.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("config GATTSERVER_VALUE_ARENA_BYTES", kconfig)
        self.assertIn("CONFIG_GATTSERVER_VALUE_ARENA_BYTES", source)
        self.assertIn("ValueArena", source)
        self.assertNotIn("k_malloc", source)
        self.assertNotIn("k_free", source)

    def test_null_initial_value_uses_zero_initialized_arena_storage(self) -> None:
        source = (ROOT / "backends" / "zephyr" / "gattserver_zephyr.cpp").read_text(
            encoding="utf-8"
        )

        self.assertNotIn(
            "(initial_value == nullptr && value_size != 0)", source
        )
        self.assertIn("if (initial_value != nullptr)", source)

    def test_complete_name_is_present_in_advertising_and_scan_response(self) -> None:
        source = (ROOT / "backends" / "zephyr" / "gattserver_zephyr.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("advertising_data[advertising_count++] = name_data", source)
        self.assertIn("const bt_data scan_response[] = {name_data}", source)

    def test_reliable_notify_retries_only_transient_errors_and_reports_each_attempt(self) -> None:
        header = (ROOT / "include" / "gattserver.h").read_text(encoding="utf-8")
        source = (ROOT / "backends" / "zephyr" / "gattserver_zephyr.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("gattserver_notify_reliable", header)
        self.assertIn("gattserver_register_notify_attempt_cb", header)
        self.assertIn("notify_retry::transmit", source)
        self.assertIn("g_notify_attempt_cb(handle, result)", source)

    def test_serialized_notify_caps_zephyr_bulk_in_flight_without_changing_esp(self) -> None:
        header = (ROOT / "include" / "gattserver.h").read_text(encoding="utf-8")
        zephyr = (ROOT / "backends" / "zephyr" / "gattserver_zephyr.cpp").read_text(
            encoding="utf-8"
        )
        esp = (ROOT / "gattserver.cpp").read_text(encoding="utf-8")

        self.assertIn("gattserver_notify_serialized", header)
        self.assertIn("bt_gatt_notify_cb", zephyr)
        self.assertIn("g_serialized_notify_complete", zephyr)
        self.assertIn("SERIALIZED_NOTIFY_TIMEOUT_MS", zephyr)
        self.assertIn("-ETIMEDOUT", zephyr)
        self.assertRegex(
            zephyr,
            r"k_sem_take\(\s*&g_serialized_notify_complete",
        )
        self.assertIn("k_sem_give(&g_serialized_notify_complete", zephyr)
        self.assertIn("serialized_notify_finished", zephyr)
        self.assertRegex(
            esp,
            r"gattserver_notify_serialized\([^}]+return gattserver_notify\(",
        )

    def test_link_info_updates_are_exposed_through_neutral_callback(self) -> None:
        header = (ROOT / "include" / "gattserver.h").read_text(encoding="utf-8")
        source = (ROOT / "backends" / "zephyr" / "gattserver_zephyr.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("gatt_link_info_t", header)
        self.assertIn("gattserver_register_link_info_cb", header)
        self.assertIn("g_link_info_cb(&linkInfo)", source)


class ZephyrEncryptionContractTest(unittest.TestCase):
    def test_declared_encryption_is_enforced_not_deferred(self) -> None:
        builder = (ROOT / "backends" / "zephyr" / "gatt_table_builder.hpp").read_text(
            encoding="utf-8"
        )
        source = (ROOT / "backends" / "zephyr" / "gattserver_zephyr.cpp").read_text(
            encoding="utf-8"
        )

        self.assertNotIn("encryption_deferred", builder)
        self.assertNotIn("encryption_deferred", source)
        self.assertNotIn("Encrypted GATT permissions deferred", source)
        self.assertIn(
            "gattserver::zephyr::kPermissionReadEncrypt ==\n              BT_GATT_PERM_READ_ENCRYPT",
            source,
        )
        self.assertIn(
            "gattserver::zephyr::kPermissionWriteEncrypt ==\n              BT_GATT_PERM_WRITE_ENCRYPT",
            source,
        )
        # The computed mask, including the encrypt bits, is what Zephyr checks.
        self.assertIn(".perm = layout.permissions,", source)
        # Callbacks still follow the base bits, so encrypted values keep them.
        self.assertIn("(layout.permissions & BT_GATT_PERM_READ) != 0", source)
        self.assertIn("(layout.permissions & BT_GATT_PERM_WRITE) != 0", source)


if __name__ == "__main__":
    unittest.main()
