
#include "interface_usb.h"
#include "ascii_protocol.hpp"

#include <MotorControl/utils.hpp>

#include <fibre/protocol.hpp>
#include <usbd_cdc.h>
#include <usbd_cdc_if.h>
#include <usb_device.h>
#include <cmsis_os.h>
#include <freertos_vars.h>

#include <odrive_main.h>

osThreadId usb_thread;
const uint32_t stack_size_usb_thread = 4096; // Bytes
USBStats_t usb_stats_;

class USBSender : public PacketSink {
public:
    USBSender(uint8_t endpoint_pair, const osSemaphoreId& sem_usb_tx)
            : endpoint_pair_(endpoint_pair), sem_usb_tx_(sem_usb_tx) {}

    int process_packet(const uint8_t* buffer, size_t length) {
        // cannot send partial packets
        if (length > USB_TX_DATA_SIZE)
            return -1;
        // wait for USB interface to become ready
        if (osSemaphoreWait(sem_usb_tx_, PROTOCOL_SERVER_TIMEOUT_MS) != osOK) {
            // If the host resets the device it might be that the TX-complete handler is never called
            // and the sem_usb_tx_ semaphore is never released. To handle this we just override the
            // TX buffer if this wait times out. The implication is that the channel is no longer lossless.
            // TODO: handle endpoint reset properly
            usb_stats_.tx_overrun_cnt++;
        }
        // transmit packet
        uint8_t status = CDC_Transmit_FS(
                const_cast<uint8_t*>(buffer) /* casting this const away is safe because...
                well... it's not actually. Stupid STM. */, length, endpoint_pair_);
        if (status != USBD_OK) {
            osSemaphoreRelease(sem_usb_tx_);
            return -1;
        }
        usb_stats_.tx_cnt++;
        return 0;
    }
private:
    uint8_t endpoint_pair_;
    const osSemaphoreId& sem_usb_tx_;
};

// Note we could have independent semaphores here to allow concurrent transmission
USBSender usb_packet_output_cdc(CDC_OUT_EP, sem_usb_tx);
USBSender usb_packet_output_native(ODRIVE_OUT_EP, sem_usb_tx);

class TreatPacketSinkAsStreamSink : public StreamSink {
public:
    TreatPacketSinkAsStreamSink(PacketSink& output) : output_(output) {}
    int process_bytes(const uint8_t* buffer, size_t length, size_t* processed_bytes) {
        // Loop to ensure all bytes get sent
        while (length) {
            size_t chunk = length < USB_TX_DATA_SIZE ? length : USB_TX_DATA_SIZE;
            if (output_.process_packet(buffer, chunk) != 0)
                return -1;
            buffer += chunk;
            length -= chunk;
            if (processed_bytes)
                *processed_bytes += chunk;
        }
        return 0;
    }
    size_t get_free_space() { return SIZE_MAX; }
private:
    PacketSink& output_;
} usb_stream_output(usb_packet_output_cdc);

// This is used by the printf feature. Hence the above statics, and below seemingly random ptr (it's externed)
// TODO: less spaghetti code
StreamSink* usb_stream_output_ptr = &usb_stream_output;

#if defined(USB_PROTOCOL_NATIVE)
BidirectionalPacketBasedChannel usb_channel(usb_packet_output_native);
#elif defined(USB_PROTOCOL_NATIVE_STREAM_BASED)
StreamBasedPacketSink usb_packetized_output(usb_stream_output);
BidirectionalPacketBasedChannel usb_channel(usb_packetized_output);
StreamToPacketSegmenter usb_native_stream_input(usb_channel);
#endif

struct USBInterface {
    uint8_t* rx_buf = nullptr;
    uint32_t rx_len = 0;
    bool data_pending = false;
    uint8_t out_ep;
    uint8_t in_ep;
    USBSender& usb_sender;
};

// Note: statics make this less modular.
// Note: we use a single rx semaphore and loop over data_pending to allow a single pump loop thread
static USBInterface CDC_interface = {
    .rx_buf = nullptr,
    .rx_len = 0,
    .data_pending = false,
    .out_ep = CDC_OUT_EP,
    .in_ep = CDC_IN_EP,
    .usb_sender = usb_packet_output_cdc,
};
static USBInterface ODrive_interface = {
    .rx_buf = nullptr,
    .rx_len = 0,
    .data_pending = false,
    .out_ep = ODRIVE_OUT_EP,
    .in_ep = ODRIVE_IN_EP,
    .usb_sender = usb_packet_output_native,
};

static void usb_server_thread(void * ctx) {
    (void) ctx;
    
    for (;;) {
        // const uint32_t usb_check_timeout = 1; // ms
        osStatus sem_stat = osSemaphoreWait(sem_usb_rx, osWaitForever);
        if (sem_stat == osOK) {
            usb_stats_.rx_cnt++;

            // CDC Interface
            if (CDC_interface.data_pending) {
                CDC_interface.data_pending = false;
                if (odrv.config_.enable_ascii_protocol_on_usb) {
                    ASCII_protocol_parse_stream(CDC_interface.rx_buf,
                            CDC_interface.rx_len, usb_stream_output);
                } else {
#if defined(USB_PROTOCOL_NATIVE)
                    usb_channel.process_packet(CDC_interface.rx_buf, CDC_interface.rx_len);
#elif defined(USB_PROTOCOL_NATIVE_STREAM_BASED)
                    usb_native_stream_input.process_bytes(
                            CDC_interface.rx_buf, CDC_interface.rx_len, nullptr);
#endif
                }
                USBD_CDC_ReceivePacket(&hUsbDeviceFS, CDC_interface.out_ep);  // Allow next packet
            }

            // Native Interface
            if (ODrive_interface.data_pending) {
                ODrive_interface.data_pending = false;
#if defined(USB_PROTOCOL_NATIVE)
                /*int result = */usb_channel.process_packet(ODrive_interface.rx_buf, ODrive_interface.rx_len);
                //if (result != 0)
                //    delay_us(500000);
#elif defined(USB_PROTOCOL_NATIVE_STREAM_BASED)
                usb_native_stream_input.process_bytes(
                        ODrive_interface.rx_buf, ODrive_interface.rx_len, nullptr);
#endif
                USBD_CDC_ReceivePacket(&hUsbDeviceFS, ODrive_interface.out_ep);  // Allow next packet
            }
        }
    }
}

// Called from CDC_Receive_FS callback function, this allows the communication
// thread to handle the incoming data
void usb_rx_process_packet(uint8_t *buf, uint32_t len, uint8_t endpoint_pair) {
    USBInterface* usb_iface;
    if (endpoint_pair == CDC_interface.out_ep) {
        usb_iface = &CDC_interface;
    } else if (endpoint_pair == ODrive_interface.out_ep) {
        usb_iface = &ODrive_interface;
    } else {
        return;
    }

    // We don't allow the next USB packet until the previous one has been processed completely.
    // Therefore it's safe to write to these vars directly since we know previous processing is complete.
    usb_iface->rx_buf = buf;
    usb_iface->rx_len = len;
    usb_iface->data_pending = true;
    osSemaphoreRelease(sem_usb_rx);
}

void start_usb_server() {
    // Start USB communication thread
    osThreadDef(usb_server_thread_def, usb_server_thread, osPriorityNormal, 0, stack_size_usb_thread / sizeof(StackType_t));
    usb_thread = osThreadCreate(osThread(usb_server_thread_def), NULL);
}
