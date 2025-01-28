#include <AtomJoyStickReceiver.h>
#include <HardwareSerial.h>


AtomJoyStickReceiver::AtomJoyStickReceiver(const uint8_t* myMacAddress) :
    _transceiver(myMacAddress),
    _received_data(_packet, sizeof(_packet))
    {}

esp_err_t AtomJoyStickReceiver::init(uint8_t channel, const uint8_t* transmitMacAddress)
{
    return  _transceiver.init(_received_data, channel, transmitMacAddress);
}

esp_err_t AtomJoyStickReceiver::broadcastMyMacAddressForBinding(int broadcastCount, int broadcastDelayMs) const
{
    // peer command as used by the StampFlyController, see: https://github.com/m5stack/Atom-JoyStick/blob/main/examples/StampFlyController/src/main.cpp#L117
    static const uint8_t peerCommand[4] { 0xaa, 0x55, 0x16, 0x88 };
    uint8_t data[16];
    static_assert(sizeof(data) > sizeof(peerCommand) + ESP_NOW_ETH_ALEN + 2);

    data[0] = _transceiver.getBroadcastChannel();
    memcpy(&data[1], _transceiver.myMacAddress(), ESP_NOW_ETH_ALEN);
    memcpy(&data[1 + ESP_NOW_ETH_ALEN], peerCommand, sizeof(peerCommand));

    for (int ii = 0; ii < broadcastCount; ++ii) {
        if (esp_err_t err = _transceiver.broadcastData(data, sizeof(data)) != ESP_OK) {
            Serial.printf("broadcastMyMacAddressForBinding failed: %X\r\n", err);
            return err;
        }
        delay(broadcastDelayMs); // Arduino delay() function has units of milliseconds
    }
    return ESP_OK;
}

/*!
Convert the 4 bytes of a floating point number to a fixed point integer in Q4dot12 format, ie in range [-2048,2047].
*/
int32_t AtomJoyStickReceiver::ubyte4float_to_Q4dot12(uint8_t f[4])
{
    union bi_t { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        uint8_t b[4];
        uint32_t i;
    };
    const bi_t n = { .b = { f[0], f[1], f[2], f[3] } };

    const uint8_t  sign     = (n.i >> 31) & 0x1; // 0x1000 0000
    const uint8_t  exponent = (n.i >> 23) & 0xFF; // 0x7F80 0000
    if (exponent == 0) {
        return 0;
    }

    const uint32_t mantissa = (n.i & 0x7FFFFF) | 0x800000; // 0x007F FFFF, or in implicit bit

    const int32_t i = mantissa >> ((22-11) - (exponent - 0x80)); // -Wshift-count-overflow
    return sign ? -i : i;
}

/*!
Check the packet if `checkPacket` set. If the packet is valid then unpack it into the member data and set the packet to empty.

Returns true if a valid packet received, false otherwise.
*/
bool AtomJoyStickReceiver::unpackPacket(checkPacket_t checkPacket)
{
    // see https://github.com/M5Fly-kanazawa/AtomJoy2024June/blob/main/src/main.cpp#L560 for packet format
    if (isPacketEmpty()) {
        return false;
    }

    uint8_t checksum = 0;
    for (int ii = 0; ii < PACKET_SIZE - 1; ++ii) {
        checksum += _packet[ii];
    }
    if ((checkPacket == CHECK_PACKET) && checksum != _packet[PACKET_SIZE - 1]) {
        //Serial.printf("checksum:%d, packet[24]:%d, packet[0]:%d, len:%d\r\n", checksum, _packet[24], _packet[0], receivedDataLen());
        setPacketEmpty();
        return false;
    }

    const uint8_t* macAddress = myMacAddress();
    if (checkPacket == DONT_CHECK_PACKET) {
        Serial.printf("packet: %02X:%02X:%02X\r\n", _packet[0], _packet[1], _packet[2]);
        //Serial.printf("peer:   %02X:%02X:%02X\r\n", _primaryPeerInfo.peer_addr[3], _primaryPeerInfo.peer_addr[4], _primaryPeerInfo.peer_addr[5]);
        //Serial.printf("my:     %02X:%02X:%02X\r\n", macAddress[3], macAddress[4], macAddress[5]);
    }
    if ((checkPacket == CHECK_PACKET) && (_packet[0] != macAddress[3] || _packet[1] != macAddress[4] || _packet[2] != macAddress[5])) {
        //Serial.printf("packet: %02X:%02X:%02X\r\n", _packet[0], _packet[1], _packet[2]);
        //Serial.printf("my:     %02X:%02X:%02X\r\n", macAddress[3], macAddress[4], macAddress[5]);
        setPacketEmpty();
        return false;
    }

    _controls[YAW].rawQ4dot12 = ubyte4float_to_Q4dot12(&_packet[3]); // cppcheck-suppress invalidPointerCast
    _controls[THROTTLE].rawQ4dot12 = ubyte4float_to_Q4dot12(&_packet[7]); // cppcheck-suppress invalidPointerCast
    _controls[ROLL].rawQ4dot12 = ubyte4float_to_Q4dot12(&_packet[11]); // cppcheck-suppress invalidPointerCast
    _controls[PITCH].rawQ4dot12 = -ubyte4float_to_Q4dot12(&_packet[15]); // cppcheck-suppress invalidPointerCast

    _armButton = _packet[19];
    _flipButton = _packet[20];
    _mode = _packet[21];  // _mode: stable or sport
    _altMode = _packet[22];
    _proactiveFlag = _packet[23];

    setPacketEmpty();
    return true;
}

void AtomJoyStickReceiver::setCurrentReadingsToBias()
{
    _biasIsSet = static_cast<int>(true);
    for (auto& control : _controls) {
        control.biasQ4dot12 = control.rawQ4dot12;
    }
}

int32_t AtomJoyStickReceiver::normalizedControl(const Control& control, bool raw)
{
    if (raw) {
        return control.rawQ4dot12;
    }
    const int32_t ret = control.rawQ4dot12 - control.biasQ4dot12;

    if (ret < -control.deadZoneQ4dot12) {
        return -(-control.deadZoneQ4dot12 - ret);//      / (control.bias - control.deadZone/2 - min);
    }
    if (ret > control.deadZoneQ4dot12) {
        return (ret - control.deadZoneQ4dot12);//    / (max - control.bias - control.deadZone/2);
    }
    return 0.0F;
}

void AtomJoyStickReceiver::resetControls()
{
    _biasIsSet = static_cast<int>(false);
    for (auto& control : _controls) {
        control.biasQ4dot12 = 0;
        control.deadZoneQ4dot12 = 0;
    }
}

void AtomJoyStickReceiver::setDeadZones(int32_t deadZoneQ4dot12)
{
    for (auto& control : _controls) {
        control.deadZoneQ4dot12 = deadZoneQ4dot12;
    }
}

int32_t AtomJoyStickReceiver::getThrottleQ4dot12() const
{
    return normalizedControl(_controls[THROTTLE], !isBiasSet());
}

int32_t AtomJoyStickReceiver::getRollQ4dot12() const
{
    return normalizedControl(_controls[ROLL], !isBiasSet());
}

int32_t AtomJoyStickReceiver::getPitchQ4dot12() const
{
    return normalizedControl(_controls[PITCH], !isBiasSet());
}

int32_t AtomJoyStickReceiver::getYawQ4dot12() const
{
    return normalizedControl(_controls[YAW], !isBiasSet());
}
