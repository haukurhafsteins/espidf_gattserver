


/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

 #include "esp_log.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "gattserver.h"


 /**
  * Utility function to log an array of bytes.
  */
 void
 print_bytes(const uint8_t *bytes, int len)
 {
     int i;
 
     for (i = 0; i < len; i++) {
         MODLOG_DFLT(INFO, "%s0x%02x", i != 0 ? ":" : "", bytes[i]);
     }
 }
 
 void print_addr(const void *addr)
 {
     const uint8_t *u8p;
 
     u8p = (const uint8_t *) addr;
     MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x",
                 u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
 }
 
 char *
 addr_str(const void *addr)
 {
     static char buf[6 * 2 + 5 + 1];
     const uint8_t *u8p;
 
     u8p = (const uint8_t *) addr;
     sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x",
             u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
 
     return buf;
 }
 
 void
 print_mbuf(const struct os_mbuf *om)
 {
     int colon, i;
 
     colon = 0;
     while (om != NULL) {
         if (colon) {
             MODLOG_DFLT(DEBUG, ":");
         } else {
             colon = 1;
         }
         for (i = 0; i < om->om_len; i++) {
             MODLOG_DFLT(DEBUG, "%s0x%02x", i != 0 ? ":" : "", om->om_data[i]);
         }
         om = SLIST_NEXT(om, om_next);
     }
 }

 /**
 * Logs information about a connection to the console.
 */
void bleprph_print_conn_desc(struct ble_gap_conn_desc *desc)
{
    MODLOG_DFLT(INFO, "handle=%d our_ota_addr_type=%d our_ota_addr=",
                desc->conn_handle, desc->our_ota_addr.type);
    print_addr(desc->our_ota_addr.val);
    MODLOG_DFLT(INFO, " our_id_addr_type=%d our_id_addr=",
                desc->our_id_addr.type);
    print_addr(desc->our_id_addr.val);
    MODLOG_DFLT(INFO, " peer_ota_addr_type=%d peer_ota_addr=",
                desc->peer_ota_addr.type);
    print_addr(desc->peer_ota_addr.val);
    MODLOG_DFLT(INFO, " peer_id_addr_type=%d peer_id_addr=",
                desc->peer_id_addr.type);
    print_addr(desc->peer_id_addr.val);
    MODLOG_DFLT(INFO, " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
                "encrypted=%d authenticated=%d bonded=%d\n",
                desc->conn_itvl, desc->conn_latency,
                desc->supervision_timeout,
                desc->sec_state.encrypted,
                desc->sec_state.authenticated,
                desc->sec_state.bonded);
}