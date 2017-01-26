#ifdef __cplusplus
extern "C" {
#endif

void cdc_acm_tx();

void cdc_acm_rx();

void init_cdc_acm();

void cdcacm_setup(void *dummy1, void *dummy2, void *dummy3);

void baudrate_reset(void *dummy1, void *dummy2, void *dummy3);

void usb_serial(void *dummy1, void *dummy2, void *dummy3);

#ifdef __cplusplus
}
#endif
