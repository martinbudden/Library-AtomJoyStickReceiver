#include <HardwareSerial.h>
#include <esp_wifi.h>

#include <ESPNOW_Transceiver.h>

//#define USE_INSTRUMENTATION
#if defined(USE_INSTRUMENTATION)
#include <freertos/FreeRTOS.h>
#endif

// see https://github.com/espressif/esp-idf/blob/v5.3.1/components/esp_wifi/include/esp_now.h

constexpr uint8_t ESPNOW_Transceiver::broadcastMacAddress[ESP_NOW_ETH_ALEN] {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/*!
Pointer to the transceiver used by the callback functions.
*/
ESPNOW_Transceiver* ESPNOW_Transceiver::transceiver;

/*!
Callback when data is sent.
*/
void ESPNOW_Transceiver::onDataSent(const uint8_t* macAddress, esp_now_send_status_t status)
{
    // status can be ESP_NOW_SEND_SUCCESS or ESP_NOW_SEND_FAIL
    transceiver->setSendStatus(status);
}

/*!
Callback when data is received.

This runs in the high priority WiFi task, and so should not perform any lengthy operations.

Parameter `len` is `int` rather than `size_t` to match `esp_now_recv_cb_t` callback signature.
*/
void ESPNOW_Transceiver::onDataReceived(const uint8_t* macAddress, const uint8_t* data, int len)
{
    if (!transceiver->isPrimaryPeerMacAddressSet()) {
        // If data is received when the primary peer MAC address is not yet set, it means we are in the binding process
        // So if check this data is not a broadcast packet and is not from the secondary peer
        if (!transceiver->macAddressIsBroadCastMacAddress(macAddress) && !transceiver->macAddressIsSecondaryPeerMacAddress(macAddress)) {
            transceiver->setPrimaryPeerMacAddress(macAddress);
        }
    }
    transceiver->copyReceivedDataToBuffer(macAddress, data, len);
}

ESPNOW_Transceiver::ESPNOW_Transceiver(const uint8_t* myMacAddress)
{
    transceiver = this;
    memcpy(_myMacAddress, myMacAddress, ESP_NOW_ETH_ALEN);
}

esp_err_t ESPNOW_Transceiver::init(uint8_t channel)
{
    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        Serial.printf("esp_now_init failed: 0x%X (0x%X)\r\n", err, err - ESP_ERR_ESPNOW_BASE);
        return err;
    }
    err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        Serial.printf("esp_wifi_set_channel failed: 0x%X (0x%X)\r\n", err, err - ESP_ERR_ESPNOW_BASE);
        return err;
    }
    err = addBroadcastPeer(channel);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_now_register_recv_cb(ESPNOW_Transceiver::onDataReceived);
    if (err != ESP_OK) {
        Serial.printf("esp_now_register_recv_cb failed: 0x%X (0x%X)\r\n", err, err - ESP_ERR_ESPNOW_BASE);
        return err;
    }
    err = esp_now_register_send_cb(ESPNOW_Transceiver::onDataSent);
    if (err != ESP_OK) {
        Serial.printf("esp_now_register_send_cb failed: %X\r\n", err);
        return err;
    }
    return ESP_OK;
}

esp_err_t ESPNOW_Transceiver::init(received_data_t& received_data, uint8_t channel, const uint8_t* primaryMacAddress)
{
    //Serial.printf("ESPNOW_Transceiver::init received data: %x, %d\r\n", received_data.bufferPtr, received_data.bufferSize);
    const esp_err_t err = init(channel);
    if (err != ESP_OK) {
        return err;
    }

    received_data.len = 0;
    _peerData[PRIMARY_PEER].receivedDataPtr = &received_data;
    _peerData[PRIMARY_PEER].peer_info.channel = channel;
    _peerData[PRIMARY_PEER].peer_info.encrypt = false;
    _peerCount = 2; // since we have already added the broadcast peer

    // Set the primary MAC address now, if it is provided
    // Otherwise it will be set from `onDataReceived` as part of the binding process
    if (primaryMacAddress != nullptr) {
        return setPrimaryPeerMacAddress(primaryMacAddress);
    }

    return ESP_OK;
}

esp_err_t ESPNOW_Transceiver::addBroadcastPeer(int channel)
{
    // set receivedDataPtr to nullptr so no broadcast data is copied
    _peerData[BROADCAST_PEER].receivedDataPtr = nullptr;
    memcpy(_peerData[BROADCAST_PEER].peer_info.peer_addr, broadcastMacAddress, ESP_NOW_ETH_ALEN);
    _peerData[BROADCAST_PEER].peer_info.channel = channel;
    _peerData[BROADCAST_PEER].peer_info.encrypt = false;

    const esp_err_t err = esp_now_add_peer(&_peerData[BROADCAST_PEER].peer_info);
    if (err != ESP_OK) {
        Serial.printf("addBroadcastPeer esp_now_add_peer failed: 0x%X (0x%X)\r\n", err, err - ESP_ERR_ESPNOW_BASE);
        return err;
    }

    // don't inadvertently erase an already set primary or secondary peer
    if (_peerCount < 1) {
        _peerCount = 1;
    }
    return err;
}

esp_err_t ESPNOW_Transceiver::addSecondaryPeer(received_data_t& received_data, const uint8_t* macAddress)
{
    received_data.len = 0;
    _peerData[PEER_2].receivedDataPtr = &received_data;
    _peerData[PEER_2].peer_info.channel = _peerData[PRIMARY_PEER].peer_info.channel;
    _peerData[PEER_2].peer_info.encrypt = false;
    if (macAddress != nullptr) {
        memcpy(_peerData[PEER_2].peer_info.peer_addr, macAddress, ESP_NOW_ETH_ALEN);
    }

    const esp_err_t err = esp_now_add_peer(&_peerData[PEER_2].peer_info);
    if (err != ESP_OK) {
        Serial.printf("addSecondaryPeer esp_now_add_peer failed: 0x%X (0x%X)\r\n", err, err - ESP_ERR_ESPNOW_BASE);
        return err;
    }

    _peerCount = 3; // includes the primary peer and the broadcast peer
    return ESP_OK;
}

bool ESPNOW_Transceiver::isPrimaryPeerMacAddressSet() const
{
     return _isPrimaryPeerMacAddressSet;
}

bool ESPNOW_Transceiver::macAddressIsBroadCastMacAddress(const uint8_t* macAddress) const
{
    if (_peerCount > 0) {
        // check this is not the broadcast MAC address
        if (memcmp(macAddress, broadcastMacAddress, ESP_NOW_ETH_ALEN) == 0) {
            return true;
        }
    }
    return false;
}

bool ESPNOW_Transceiver::macAddressIsSecondaryPeerMacAddress(const uint8_t* macAddress) const
{
    if (_peerCount > 2) {
        // the secondary peer has been set, so compare the MAC address with that of the secondary peer.
        if (memcmp(macAddress, _peerData[SECONDARY_PEER].peer_info.peer_addr, ESP_NOW_ETH_ALEN) == 0) {
            return true;
        }
    }
    return false;
}

esp_err_t ESPNOW_Transceiver::setPrimaryPeerMacAddress(const uint8_t* macAddress)
{
    memcpy(_peerData[PRIMARY_PEER].peer_info.peer_addr, macAddress, ESP_NOW_ETH_ALEN);

    if (_isPrimaryPeerMacAddressSet != 0) {
        return ESP_OK;
    }

    _isPrimaryPeerMacAddressSet = static_cast<int>(true);

    const esp_err_t err = esp_now_add_peer(&_peerData[PRIMARY_PEER].peer_info);
    if (err != ESP_OK) {
        Serial.printf("setTransmitMacAddress esp_now_add_peer failed: 0x%X (0x%X)\r\n", err, err - ESP_ERR_ESPNOW_BASE);
    }
    return err;
}

bool ESPNOW_Transceiver::copyReceivedDataToBuffer(const uint8_t* macAddress, const uint8_t* data, size_t len) // NOLINT(readability-make-member-function-const) false positive
{
    esp_now_peer_info_t peerInfo;
    const esp_err_t err = esp_now_get_peer(macAddress, &peerInfo);
    if (err == ESP_ERR_ESPNOW_NOT_FOUND) {
        // ignore any data sent from someone who is not a peer
        return false;
    }
    if (err != ESP_OK) {
        Serial.printf("copyReceivedDataToBuffer esp_now_get_peer failed: 0x%X (0x%X)\r\n", err, err - ESP_ERR_ESPNOW_BASE);
        return false;
    }

    for (int ii = PRIMARY_PEER; ii < _peerCount; ++ii) {
        const auto& peerData = _peerData[ii];
        if (memcmp(macAddress, peerData.peer_info.peer_addr, ESP_NOW_ETH_ALEN) == 0) {
            if (ii == PRIMARY_PEER) {
                ++_receivedPacketCount; // only count packets being sent to the primary peer
                const TickType_t tickCount = xTaskGetTickCount();
                _tickCountDelta = tickCount - _tickCountPrevious;
                _tickCountPrevious = tickCount;
            }
            // copy the received data into the _peerData buffer
            const int copyLength = std::min(len, peerData.receivedDataPtr->bufferSize); // so don't overwrite buffer
            memcpy(peerData.receivedDataPtr->bufferPtr, data, copyLength);
            // and set the _peerData length
            peerData.receivedDataPtr->len = copyLength;
            return true;
        }
    }
    return false;
}

esp_err_t ESPNOW_Transceiver::sendData(const uint8_t* data, size_t len) const
{
    //const uint8_t* ma = _transmitMacAddress;
    //Serial.printf("sendData MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", ma[0], ma[1], ma[2], ma[3], ma[4], ma[5]);
    //Serial.printf("sendData len:%d\r\n", len);
    assert(len < ESP_NOW_MAX_DATA_LEN); // 250
    if (!isPrimaryPeerMacAddressSet() || data == nullptr || len==0) {
        return ESP_FAIL;
    }
    const esp_err_t err = esp_now_send(_peerData[PRIMARY_PEER].peer_info.peer_addr, data, len);
    //if (err != ESP_OK) { Serial.printf("sendData err:0x%X (0x%X)\r\n", err, err - ESP_ERR_ESPNOW_BASE); }
    return err;
}

esp_err_t ESPNOW_Transceiver::sendDataSecondary(const uint8_t* data, size_t len) const
{
    //const uint8_t* ma = _peerData[SECONDARY_PEER].peer_info.peer_addr;
    //Serial.printf("sendDataSecondary MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", ma[0], ma[1], ma[2], ma[3], ma[4], ma[5]);
    //Serial.printf("sendDataSecondary len:%d\r\n", len);
    assert(len < ESP_NOW_MAX_DATA_LEN); // 250
    if (_peerCount < SECONDARY_PEER + 1 || data == nullptr || len==0) {
        return ESP_FAIL;
    }
    const esp_err_t err = esp_now_send(_peerData[SECONDARY_PEER].peer_info.peer_addr, data, len);
    //if (err != ESP_OK) { Serial.printf("sendDataSecondary err:0x%X (0x%x)\r\n", err, err - ESP_ERR_ESPNOW_BASE); }
    return err;
}
