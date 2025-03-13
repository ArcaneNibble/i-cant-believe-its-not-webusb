/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

// Pico
#include "pico/stdlib.h"

// For memcpy
#include <string.h>

// Include descriptor struct definitions
#include "usb_common.h"
// USB register definitions from pico-sdk
#include "hardware/regs/usb.h"
// USB hardware struct definitions from pico-sdk
#include "hardware/structs/usb.h"
// For interrupt enable and numbers
#include "hardware/irq.h"
// For resetting the USB controller
#include "hardware/resets.h"

// Device descriptors
#include "dev_lowlevel.h"

#define usb_hw_set ((usb_hw_t *)hw_set_alias_untyped(usb_hw))
#define usb_hw_clear ((usb_hw_t *)hw_clear_alias_untyped(usb_hw))

// Function prototypes for our device specific endpoint handlers defined
// later on
void ep0_in_handler(uint8_t *buf, uint16_t len);
void ep0_out_handler(uint8_t *buf, uint16_t len);
void ep1_out_handler(uint8_t *buf, uint16_t len);
void ep2_in_handler(uint8_t *buf, uint16_t len);

// Global device address
static bool should_set_address = false;
static uint8_t dev_addr = 0;
static volatile bool configured = false;

// Global data buffer for EP0
static uint8_t ep0_buf[64];

// Struct defining the device configuration
static struct usb_device_configuration dev_config = {
        .device_descriptor = &device_descriptor,
        .interface_descriptor = &interface_descriptor,
        .hid_descriptor = &hid_descriptor,
        .config_descriptor = &config_descriptor,
        .lang_descriptor = lang_descriptor,
        .descriptor_strings = descriptor_strings,
        .endpoints = {
                {
                        .descriptor = &ep0_out,
                        .handler = &ep0_out_handler,
                        .endpoint_control = NULL, // NA for EP0
                        .buffer_control = &usb_dpram->ep_buf_ctrl[0].out,
                        // EP0 in and out share a data buffer
                        .data_buffer = &usb_dpram->ep0_buf_a[0],
                },
                {
                        .descriptor = &ep0_in,
                        .handler = &ep0_in_handler,
                        .endpoint_control = NULL, // NA for EP0,
                        .buffer_control = &usb_dpram->ep_buf_ctrl[0].in,
                        // EP0 in and out share a data buffer
                        .data_buffer = &usb_dpram->ep0_buf_a[0],
                },
                {
                        .descriptor = &ep1_out,
                        .handler = &ep1_out_handler,
                        // EP1 starts at offset 0 for endpoint control
                        .endpoint_control = &usb_dpram->ep_ctrl[0].out,
                        .buffer_control = &usb_dpram->ep_buf_ctrl[1].out,
                        // First free EPX buffer
                        .data_buffer = &usb_dpram->epx_data[0 * 64],
                },
                {
                        .descriptor = &ep2_in,
                        .handler = &ep2_in_handler,
                        .endpoint_control = &usb_dpram->ep_ctrl[1].in,
                        .buffer_control = &usb_dpram->ep_buf_ctrl[2].in,
                        // Second free EPX buffer
                        .data_buffer = &usb_dpram->epx_data[1 * 64],
                }
        }
};

/**
 * @brief Given an endpoint address, return the usb_endpoint_configuration of that endpoint. Returns NULL
 * if an endpoint of that address is not found.
 *
 * @param addr
 * @return struct usb_endpoint_configuration*
 */
struct usb_endpoint_configuration *usb_get_endpoint_configuration(uint8_t addr) {
    struct usb_endpoint_configuration *endpoints = dev_config.endpoints;
    for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
        if (endpoints[i].descriptor && (endpoints[i].descriptor->bEndpointAddress == addr)) {
            return &endpoints[i];
        }
    }
    return NULL;
}

/**
 * @brief Given a C string, fill the EP0 data buf with a USB string descriptor for that string.
 *
 * @param C string you would like to send to the USB host
 * @return the length of the string descriptor in EP0 buf
 */
uint8_t usb_prepare_string_descriptor(const unsigned char *str) {
    // 2 for bLength + bDescriptorType + strlen * 2 because string is unicode. i.e. other byte will be 0
    uint8_t bLength = 2 + (strlen((const char *)str) * 2);
    static const uint8_t bDescriptorType = 0x03;

    volatile uint8_t *buf = &ep0_buf[0];
    *buf++ = bLength;
    *buf++ = bDescriptorType;

    uint8_t c;

    do {
        c = *str++;
        *buf++ = c;
        *buf++ = 0;
    } while (c != '\0');

    return bLength;
}

/**
 * @brief Take a buffer pointer located in the USB RAM and return as an offset of the RAM.
 *
 * @param buf
 * @return uint32_t
 */
static inline uint32_t usb_buffer_offset(volatile uint8_t *buf) {
    return (uint32_t) buf ^ (uint32_t) usb_dpram;
}

/**
 * @brief Set up the endpoint control register for an endpoint (if applicable. Not valid for EP0).
 *
 * @param ep
 */
void usb_setup_endpoint(const struct usb_endpoint_configuration *ep) {
    printf("Set up endpoint 0x%x with buffer address 0x%p\n", ep->descriptor->bEndpointAddress, ep->data_buffer);

    // EP0 doesn't have one so return if that is the case
    if (!ep->endpoint_control) {
        return;
    }

    // Get the data buffer as an offset of the USB controller's DPRAM
    uint32_t dpram_offset = usb_buffer_offset(ep->data_buffer);
    uint32_t reg = EP_CTRL_ENABLE_BITS
                   | EP_CTRL_INTERRUPT_PER_BUFFER
                   | (ep->descriptor->bmAttributes << EP_CTRL_BUFFER_TYPE_LSB)
                   | dpram_offset;
    *ep->endpoint_control = reg;
}

/**
 * @brief Set up the endpoint control register for each endpoint.
 *
 */
void usb_setup_endpoints() {
    const struct usb_endpoint_configuration *endpoints = dev_config.endpoints;
    for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
        if (endpoints[i].descriptor && endpoints[i].handler) {
            usb_setup_endpoint(&endpoints[i]);
        }
    }
}

/**
 * @brief Set up the USB controller in device mode, clearing any previous state.
 *
 */
void usb_device_init() {
    // Reset usb controller
    reset_unreset_block_num_wait_blocking(RESET_USBCTRL);

    // Clear any previous state in dpram just in case
    memset(usb_dpram, 0, sizeof(*usb_dpram)); // <1>

    // Enable USB interrupt at processor
    irq_set_enabled(USBCTRL_IRQ, true);

    // Mux the controller to the onboard usb phy
    usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;

    // Force VBUS detect so the device thinks it is plugged into a host
    usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;

    // Enable the USB controller in device mode.
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;

    // Enable an interrupt per EP0 transaction
    usb_hw->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS; // <2>

    // Enable interrupts for when a buffer is done, when the bus is reset,
    // and when a setup packet is received
    usb_hw->inte = USB_INTS_BUFF_STATUS_BITS |
                   USB_INTS_BUS_RESET_BITS |
                   USB_INTS_SETUP_REQ_BITS;

    // Set up endpoints (endpoint control registers)
    // described by device configuration
    usb_setup_endpoints();

    // Present full speed device by enabling pull up on DP
    usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
}

/**
 * @brief Given an endpoint configuration, returns true if the endpoint
 * is transmitting data to the host (i.e. is an IN endpoint)
 *
 * @param ep, the endpoint configuration
 * @return true
 * @return false
 */
static inline bool ep_is_tx(struct usb_endpoint_configuration *ep) {
    return ep->descriptor->bEndpointAddress & USB_DIR_IN;
}

/**
 * @brief Starts a transfer on a given endpoint.
 *
 * @param ep, the endpoint configuration.
 * @param buf, the data buffer to send. Only applicable if the endpoint is TX
 * @param len, the length of the data in buf (this example limits max len to one packet - 64 bytes)
 */
void usb_start_transfer(struct usb_endpoint_configuration *ep, uint8_t *buf, uint16_t len) {
    // We are asserting that the length is <= 64 bytes for simplicity of the example.
    // For multi packet transfers see the tinyusb port.
    assert(len <= 64);

    printf("Start transfer of len %d on ep addr 0x%x\n", len, ep->descriptor->bEndpointAddress);

    // Prepare buffer control register value
    uint32_t val = len | USB_BUF_CTRL_AVAIL;

    if (ep_is_tx(ep)) {
        // Need to copy the data from the user buffer to the usb memory
        memcpy((void *) ep->data_buffer, (void *) buf, len);
        // Mark as full
        val |= USB_BUF_CTRL_FULL;
    }

    // Set pid and flip for next transfer
    val |= ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
    ep->next_pid ^= 1u;

    *ep->buffer_control = val;
}

/**
 * @brief Send device descriptor to host
 *
 */
void usb_handle_device_descriptor(volatile struct usb_setup_packet *pkt) {
    const struct usb_device_descriptor *d = dev_config.device_descriptor;
    // EP0 in
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_IN_ADDR);
    // Always respond with pid 1
    ep->next_pid = 1;
    usb_start_transfer(ep, (uint8_t *) d, MIN(sizeof(struct usb_device_descriptor), pkt->wLength));
}

/**
 * @brief Send the configuration descriptor (and potentially the configuration and endpoint descriptors) to the host.
 *
 * @param pkt, the setup packet received from the host.
 */
void usb_handle_config_descriptor(volatile struct usb_setup_packet *pkt) {
    uint8_t *buf = &ep0_buf[0];

    // First request will want just the config descriptor
    const struct usb_configuration_descriptor *d = dev_config.config_descriptor;
    memcpy((void *) buf, d, sizeof(struct usb_configuration_descriptor));
    buf += sizeof(struct usb_configuration_descriptor);

    // If we more than just the config descriptor copy it all
    if (pkt->wLength >= d->wTotalLength) {
        memcpy((void *) buf, dev_config.interface_descriptor, sizeof(struct usb_interface_descriptor));
        buf += sizeof(struct usb_interface_descriptor);
        memcpy((void *) buf, dev_config.hid_descriptor, sizeof(struct usb_hid_descriptor));
        buf += sizeof(struct usb_hid_descriptor);
        const struct usb_endpoint_configuration *ep = dev_config.endpoints;

        // Copy all the endpoint descriptors starting from EP1
        for (uint i = 2; i < USB_NUM_ENDPOINTS; i++) {
            if (ep[i].descriptor) {
                memcpy((void *) buf, ep[i].descriptor, sizeof(struct usb_endpoint_descriptor));
                buf += sizeof(struct usb_endpoint_descriptor);
            }
        }

    }

    // Send data
    // Get len by working out end of buffer subtract start of buffer
    uint32_t len = (uint32_t) buf - (uint32_t) &ep0_buf[0];
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), &ep0_buf[0], MIN(len, pkt->wLength));
}

void usb_handle_hid_descriptor(volatile struct usb_setup_packet *pkt) {
    uint8_t *buf = &ep0_buf[0];

    const struct usb_hid_descriptor *d = dev_config.hid_descriptor;
    memcpy((void *) buf, d, sizeof(struct usb_hid_descriptor));
    buf += sizeof(struct usb_hid_descriptor);
    uint32_t len = (uint32_t) buf - (uint32_t) &ep0_buf[0];
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), &ep0_buf[0], MIN(len, pkt->wLength));
}

void usb_handle_report_descriptor(volatile struct usb_setup_packet *pkt) {
    uint8_t *buf = &ep0_buf[0];
    memcpy((void *) buf, report_descriptor, sizeof(report_descriptor));
    uint32_t len = sizeof(report_descriptor);
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), &ep0_buf[0], MIN(len, pkt->wLength));
}

/**
 * @brief Handle a BUS RESET from the host by setting the device address back to 0.
 *
 */
void usb_bus_reset(void) {
    // Set address back to 0
    dev_addr = 0;
    should_set_address = false;
    usb_hw->dev_addr_ctrl = 0;
    configured = false;
}

/**
 * @brief Send the requested string descriptor to the host.
 *
 * @param pkt, the setup packet from the host.
 */
void usb_handle_string_descriptor(volatile struct usb_setup_packet *pkt) {
    uint8_t i = pkt->wValue & 0xff;
    uint8_t len = 0;

    if (i == 0) {
        len = 4;
        memcpy(&ep0_buf[0], dev_config.lang_descriptor, len);
    } else {
        // Prepare fills in ep0_buf
        len = usb_prepare_string_descriptor(dev_config.descriptor_strings[i - 1]);
    }

    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), &ep0_buf[0], MIN(len, pkt->wLength));
}

/**
 * @brief Sends a zero length status packet back to the host.
 */
void usb_acknowledge_out_request(void) {
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
}

/**
 * @brief Handles a SET_ADDR request from the host. The actual setting of the device address in
 * hardware is done in ep0_in_handler. This is because we have to acknowledge the request first
 * as a device with address zero.
 *
 * @param pkt, the setup packet from the host.
 */
void usb_set_device_address(volatile struct usb_setup_packet *pkt) {
    // Set address is a bit of a strange case because we have to send a 0 length status packet first with
    // address 0
    dev_addr = (pkt->wValue & 0xff);
    printf("Set address %d\r\n", dev_addr);
    // Will set address in the callback phase
    should_set_address = true;
    usb_acknowledge_out_request();
}

/**
 * @brief Handles a SET_CONFIGRUATION request from the host. Assumes one configuration so simply
 * sends a zero length status packet back to the host.
 *
 * @param pkt, the setup packet from the host.
 */
void usb_set_device_configuration(__unused volatile struct usb_setup_packet *pkt) {
    // Only one configuration so just acknowledge the request
    printf("Device Enumerated\r\n");
    usb_acknowledge_out_request();
    configured = true;
}

/**
 * @brief Respond to a setup packet from the host.
 *
 */
void usb_handle_setup_packet(void) {
    volatile struct usb_setup_packet *pkt = (volatile struct usb_setup_packet *) &usb_dpram->setup_packet;
    uint8_t req_direction = pkt->bmRequestType;
    uint8_t req = pkt->bRequest;

    // Reset PID to 1 for EP0 IN
    usb_get_endpoint_configuration(EP0_IN_ADDR)->next_pid = 1u;

    if (req_direction == USB_DIR_OUT) {
        if (req == USB_REQUEST_SET_ADDRESS) {
            usb_set_device_address(pkt);
        } else if (req == USB_REQUEST_SET_CONFIGURATION) {
            usb_set_device_configuration(pkt);
        } else {
            usb_acknowledge_out_request();
            printf("Other OUT request (0x%x)\r\n", pkt->bRequest);
        }
    } else if (req_direction == USB_DIR_IN) {
        if (req == USB_REQUEST_GET_DESCRIPTOR) {
            uint16_t descriptor_type = pkt->wValue >> 8;

            switch (descriptor_type) {
                case USB_DT_DEVICE:
                    usb_handle_device_descriptor(pkt);
                    printf("GET DEVICE DESCRIPTOR\r\n");
                    break;

                case USB_DT_CONFIG:
                    usb_handle_config_descriptor(pkt);
                    printf("GET CONFIG DESCRIPTOR\r\n");
                    break;

                case USB_DT_STRING:
                    usb_handle_string_descriptor(pkt);
                    printf("GET STRING DESCRIPTOR\r\n");
                    break;

                default:
                    printf("Unhandled GET_DESCRIPTOR type 0x%x\r\n", descriptor_type);
            }
        } else {
            printf("Other IN request (0x%x)\r\n", pkt->bRequest);
        }
    } else if (req_direction == 0x81) {
        if (req == USB_REQUEST_GET_DESCRIPTOR) {
            uint16_t descriptor_type = pkt->wValue >> 8;

            switch (descriptor_type) {
                case USB_DT_HID:
                    usb_handle_hid_descriptor(pkt);
                    printf("GET HID DESCRIPTOR\r\n");
                    break;

                case USB_DT_HID_REPORT:
                    usb_handle_report_descriptor(pkt);
                    printf("GET HID REPORT DESCRIPTOR\r\n");
                    break;

                default:
                    printf("Unhandled GET_DESCRIPTOR type 0x%x\r\n", descriptor_type);
            }
        } else {
            printf("Other IN request (0x%x)\r\n", pkt->bRequest);
        }
    }
}

/**
 * @brief Notify an endpoint that a transfer has completed.
 *
 * @param ep, the endpoint to notify.
 */
static void usb_handle_ep_buff_done(struct usb_endpoint_configuration *ep) {
    uint32_t buffer_control = *ep->buffer_control;
    // Get the transfer length for this endpoint
    uint16_t len = buffer_control & USB_BUF_CTRL_LEN_MASK;

    // Call that endpoints buffer done handler
    ep->handler((uint8_t *) ep->data_buffer, len);
}

/**
 * @brief Find the endpoint configuration for a specified endpoint number and
 * direction and notify it that a transfer has completed.
 *
 * @param ep_num
 * @param in
 */
static void usb_handle_buff_done(uint ep_num, bool in) {
    uint8_t ep_addr = ep_num | (in ? USB_DIR_IN : 0);
    printf("EP %d (in = %d) done\n", ep_num, in);
    for (uint i = 0; i < USB_NUM_ENDPOINTS; i++) {
        struct usb_endpoint_configuration *ep = &dev_config.endpoints[i];
        if (ep->descriptor && ep->handler) {
            if (ep->descriptor->bEndpointAddress == ep_addr) {
                usb_handle_ep_buff_done(ep);
                return;
            }
        }
    }
}

/**
 * @brief Handle a "buffer status" irq. This means that one or more
 * buffers have been sent / received. Notify each endpoint where this
 * is the case.
 */
static void usb_handle_buff_status() {
    uint32_t buffers = usb_hw->buf_status;
    uint32_t remaining_buffers = buffers;

    uint bit = 1u;
    for (uint i = 0; remaining_buffers && i < USB_NUM_ENDPOINTS * 2; i++) {
        if (remaining_buffers & bit) {
            // clear this in advance
            usb_hw_clear->buf_status = bit;
            // IN transfer for even i, OUT transfer for odd i
            usb_handle_buff_done(i >> 1u, !(i & 1u));
            remaining_buffers &= ~bit;
        }
        bit <<= 1u;
    }
}

/**
 * @brief USB interrupt handler
 *
 */
#ifdef __cplusplus
extern "C" {
#endif
/// \tag::isr_setup_packet[]
void isr_usbctrl(void) {
    // USB interrupt handler
    uint32_t status = usb_hw->ints;
    uint32_t handled = 0;

    // Setup packet received
    if (status & USB_INTS_SETUP_REQ_BITS) {
        handled |= USB_INTS_SETUP_REQ_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
        usb_handle_setup_packet();
    }
/// \end::isr_setup_packet[]

    // Buffer status, one or more buffers have completed
    if (status & USB_INTS_BUFF_STATUS_BITS) {
        handled |= USB_INTS_BUFF_STATUS_BITS;
        usb_handle_buff_status();
    }

    // Bus is reset
    if (status & USB_INTS_BUS_RESET_BITS) {
        printf("BUS RESET\n");
        handled |= USB_INTS_BUS_RESET_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
        usb_bus_reset();
    }

    if (status ^ handled) {
        panic("Unhandled IRQ 0x%x\n", (uint) (status ^ handled));
    }
}
#ifdef __cplusplus
}
#endif

/**
 * @brief EP0 in transfer complete. Either finish the SET_ADDRESS process, or receive a zero
 * length status packet from the host.
 *
 * @param buf the data that was sent
 * @param len the length that was sent
 */
void ep0_in_handler(__unused uint8_t *buf, __unused uint16_t len) {
    if (should_set_address) {
        // Set actual device address in hardware
        usb_hw->dev_addr_ctrl = dev_addr;
        should_set_address = false;
    } else {
        // Receive a zero length status packet from the host on EP0 OUT
        struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_OUT_ADDR);
        usb_start_transfer(ep, NULL, 0);
    }
}

void ep0_out_handler(__unused uint8_t *buf, __unused uint16_t len) {
}


//////// U2F HAX STUFF

uint8_t cmd_buf[7609];
size_t cmd_pos;
size_t cmd_sz;
int cmd_seq;
int cmd_in_progress;

uint8_t res_buf[7609];
size_t res_pos;
size_t res_sz;
int res_seq;

uint8_t usb_buf[64];

uint32_t cid_in_progress;
uint32_t cid_next;

void do_u2f_cmd() {
    struct usb_endpoint_configuration *ep2 = usb_get_endpoint_configuration(EP2_IN_ADDR);

    if (cmd_in_progress == 0x81) {
        // CTAPHID_PING
        memcpy(res_buf, cmd_buf, cmd_sz);

        res_sz = cmd_sz;
        res_seq = 0;
        res_pos = MIN(64 - 7, res_sz);
    }

    if (cmd_in_progress == 0x83) {
        // CTAPHID_MSG
        uint16_t cla_ins = (cmd_buf[0] << 8) | cmd_buf[1];

        if (cla_ins == 0x0003) {
            // U2F_VERSION
            res_buf[0] = 'U';
            res_buf[1] = '2';
            res_buf[2] = 'F';
            res_buf[3] = '_';
            res_buf[4] = 'V';
            res_buf[5] = '2';
            res_buf[6] = 0x90;
            res_buf[7] = 0x00;
            res_sz = res_pos = 8;
        } else if (cla_ins == 0x0002) {
            // U2F_AUTHENTICATE

            if (cmd_buf[2] == 0x07) {
                // check-only
                if (cmd_buf[7 + 65] == 0xfe && cmd_buf[7 + 66] == 0xed && cmd_buf[7 + 67] == 0xfa && cmd_buf[7 + 68] == 0xce) {
                    // ok
                    res_buf[0] = 0x69;
                    res_buf[1] = 0x85;
                    res_sz = res_pos = 2;
                } else {
                    // not ok
                    res_buf[0] = 0x69;
                    res_buf[1] = 0x84;
                    res_sz = res_pos = 2;
                }
            } else {
                if (!(cmd_buf[7 + 65] == 0xfe && cmd_buf[7 + 66] == 0xed && cmd_buf[7 + 67] == 0xfa && cmd_buf[7 + 68] == 0xce)) {
                    // not ok
                    res_buf[0] = 0x69;
                    res_buf[1] = 0x84;
                    res_sz = res_pos = 2;
                } else {
                    // control LED with extra byte
                    gpio_put(PICO_DEFAULT_LED_PIN, cmd_buf[7 + 73]);

                    memset(res_buf, 0, sizeof(res_buf));
                    res_buf[0] = 0x01;  // user presence
                    res_buf[1] = 0xde;  // counter
                    res_buf[2] = 0xad;
                    res_buf[3] = 0xbe;
                    res_buf[4] = 0xef;

                    res_buf[5] = 0x30;      // ASN.1 sequence
                    res_buf[6] = 0x44;

                    res_buf[7] = 0x02;      // ASN.1 integer
                    res_buf[8] = 0x20;

                    res_buf[9] = 0x80;     // make it not all zero
                    // copy bytes [13-16] to signature, except inverted
                    res_buf[10] = cmd_buf[7 + 69] ^ 0xff;
                    res_buf[11] = cmd_buf[7 + 70] ^ 0xff;
                    res_buf[12] = cmd_buf[7 + 71] ^ 0xff;
                    res_buf[13] = cmd_buf[7 + 72] ^ 0xff;

                    res_buf[41] = 0x02;     // ASN.1 integer
                    res_buf[42] = 0x20;

                    res_buf[43] = 0x80;     // make it not all zero

                    res_buf[75] = 0x90;
                    res_buf[76] = 0x00;
                    res_sz = 77;
                    res_seq = 0;
                    res_pos = MIN(64 - 7, res_sz);
                }
            }
        } else {
            // unsupported
            res_buf[0] = 0x6d;
            res_buf[1] = 0x00;
            res_sz = res_pos = 2;
        }
    }

    // common setup for response
    memset(usb_buf, 0, sizeof(usb_buf));
    memcpy(usb_buf, &cid_in_progress, 4);
    usb_buf[4] = cmd_in_progress;
    usb_buf[5] = res_sz >> 8;
    usb_buf[6] = res_sz;
    memcpy(&usb_buf[7], res_buf, res_pos);
    usb_start_transfer(ep2, usb_buf, 64);

    if (res_pos == res_sz) {
        cmd_in_progress = 0;
        cid_in_progress = 0;
    }
}

void ep1_out_handler(uint8_t *buf, uint16_t len) {
    printf("RX %d bytes from host\n", len);

    struct usb_endpoint_configuration *ep2 = usb_get_endpoint_configuration(EP2_IN_ADDR);

    uint32_t this_cid;
    memcpy(&this_cid, buf, 4);

    if (buf[4] & 0x80) {
        // init packet
        uint16_t bcnt = (buf[5] << 8) | buf[6];

        if (this_cid == 0) goto out;

        switch (buf[4]) {
            case 0x86:
                // CTAPHID_INIT
                if (bcnt != 8) {
                    // invalid len
                    memset(usb_buf, 0, sizeof(usb_buf));
                    memcpy(usb_buf, buf, 4);
                    usb_buf[4] = 0xbf;
                    usb_buf[5] = 0;
                    usb_buf[6] = 1;
                    usb_buf[7] = 3;
                    usb_start_transfer(ep2, usb_buf, 64);
                    break;
                }

                // allocate a new channel id
                uint32_t new_cid = this_cid;
                if (new_cid = 0xffffffff) {
                    ++cid_next;
                    if (!cid_next) ++cid_next;
                    new_cid = cid_next;
                }

                memset(usb_buf, 0, sizeof(usb_buf));
                memcpy(usb_buf, buf, 4);
                usb_buf[4] = 0x86;
                usb_buf[5] = 0;
                usb_buf[6] = 17;
                memcpy(&usb_buf[7], &buf[7], 8);
                memcpy(&usb_buf[15], &new_cid, 4);
                usb_buf[19] = 2;
                usb_buf[20] = 0;
                usb_buf[21] = 0;
                usb_buf[22] = 0;
                usb_buf[23] = 0;
                usb_start_transfer(ep2, usb_buf, 64);
                break;

            case 0x81:
            case 0x83:
                // CTAPHID_MSG or CTAPHID_PING
                if (cid_in_progress) {
                    // busy
                    memset(usb_buf, 0, sizeof(usb_buf));
                    memcpy(usb_buf, buf, 4);
                    usb_buf[4] = 0xbf;
                    usb_buf[5] = 0;
                    usb_buf[6] = 1;
                    usb_buf[7] = 6;
                    usb_start_transfer(ep2, usb_buf, 64);
                    break;
                }

                cid_in_progress = this_cid;
                cmd_in_progress = buf[4];
                cmd_sz = bcnt;
                cmd_pos = MIN(64 - 7, bcnt);
                cmd_seq = 0;
                memcpy(cmd_buf, &buf[7], cmd_pos);

                if (cmd_pos == cmd_sz)
                    do_u2f_cmd();
                break;

            default:
                // invalid command
                memset(usb_buf, 0, sizeof(usb_buf));
                memcpy(usb_buf, buf, 4);
                usb_buf[4] = 0xbf;
                usb_buf[5] = 0;
                usb_buf[6] = 1;
                usb_buf[7] = 1;
                usb_start_transfer(ep2, usb_buf, 64);
                break;
        }
    } else {
        // continuation
        if (cmd_in_progress && cid_in_progress == this_cid) {
            if (buf[4] == cmd_seq) {
                // valid seq
                uint32_t chunk_sz = MIN(64 - 5, cmd_sz - cmd_pos);
                memcpy(&cmd_buf[cmd_pos], &buf[5], chunk_sz);
                cmd_pos += chunk_sz;
                cmd_seq++;

                if (cmd_pos == cmd_sz)
                    do_u2f_cmd();
            } else {
                // invalid seq
                memset(usb_buf, 0, sizeof(usb_buf));
                memcpy(usb_buf, buf, 4);
                usb_buf[4] = 0xbf;
                usb_buf[5] = 0;
                usb_buf[6] = 1;
                usb_buf[7] = 4;
                usb_start_transfer(ep2, usb_buf, 64);
            }
        }
    }

out:
    // Get ready to rx again from host
    usb_start_transfer(usb_get_endpoint_configuration(EP1_OUT_ADDR), NULL, 64);
}

void ep2_in_handler(__unused uint8_t *buf, uint16_t len) {
    printf("Sent %d bytes to host\n", len);

    struct usb_endpoint_configuration *ep2 = usb_get_endpoint_configuration(EP2_IN_ADDR);

    if (res_pos != res_sz) {
        uint32_t chunk_sz = MIN(64 - 5, res_sz - res_pos);
        memset(usb_buf, 0, sizeof(usb_buf));
        memcpy(usb_buf, &cid_in_progress, 4);
        usb_buf[4] = res_seq++;
        memcpy(&usb_buf[5], &res_buf[res_pos], chunk_sz);
        usb_start_transfer(ep2, usb_buf, 64);

        res_pos += chunk_sz;
        if (res_pos == res_sz) {
            cmd_in_progress = 0;
            cid_in_progress = 0;
        }
    }
}

int main(void) {
    stdio_init_all();
    printf("USB Device Low-Level hardware example\n");
    usb_device_init();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // Wait until configured
    while (!configured) {
        tight_loop_contents();
    }

    // Get ready to rx from host
    usb_start_transfer(usb_get_endpoint_configuration(EP1_OUT_ADDR), NULL, 64);

    // Everything is interrupt driven so just loop here
    while (1) {
        tight_loop_contents();
    }
}
