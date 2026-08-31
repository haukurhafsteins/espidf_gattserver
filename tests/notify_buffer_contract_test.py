#!/usr/bin/env python3

from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


class NotifyBufferContractTest(unittest.TestCase):
    def test_public_api_is_stack_neutral(self) -> None:
        header = (ROOT / "include" / "gattserver.h").read_text(encoding="utf-8")

        self.assertIn(
            "int gattserver_get_available_notify_buffers(void);", header
        )
        self.assertNotIn("os_msys", header)

    def test_esp_backend_preserves_the_nimble_buffer_count(self) -> None:
        source = (ROOT / "gattserver.cpp").read_text(encoding="utf-8")
        implementation = re.compile(
            r"int gattserver_get_available_notify_buffers\(void\)\s*"
            r"\{\s*return os_msys_num_free\(\);\s*\}"
        )

        self.assertIn('#include "os/os_mbuf.h"', source)
        self.assertRegex(source, implementation)

    def test_zephyr_backend_fails_closed_without_a_pool_metric(self) -> None:
        source = (
            ROOT / "backends" / "zephyr" / "gattserver_zephyr.cpp"
        ).read_text(encoding="utf-8")
        implementation = re.compile(
            r"int gattserver_get_available_notify_buffers\(void\)\s*"
            r"\{\s*return 0;\s*\}"
        )

        self.assertRegex(source, implementation)


if __name__ == "__main__":
    unittest.main()
