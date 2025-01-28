# pragma once

#include <ESPNOW_Transceiver.h>


/*!
Receiver compatible with the M5Stack Atom JoyStick.
*/
class AtomJoyStickReceiver {
public:
    explicit AtomJoyStickReceiver(const uint8_t* myMacAddress);
    esp_err_t init(uint8_t channel, const uint8_t* transmitMacAddress);
public:
    enum { DEFAULT_BROADCAST_COUNT = 20, DEFAULT_BROADCAST_DELAY_MS = 50 };
public:
    enum { MODE_STABLE = 0, MODE_SPORT = 1 };
    enum { ALT_MODE_AUTO = 4, ALT_MODE_MANUAL = 5};
private:
    enum { PACKET_SIZE = 25 };
    enum { THROTTLE = 0, ROLL = 1, PITCH = 2, YAW = 3, CONTROL_COUNT = 4 };
public:
    inline ESPNOW_Transceiver& getTransceiver() { return _transceiver; }
    inline esp_err_t sendData(const uint8_t* data, size_t len) const {return _transceiver.sendData(data, len);}
    inline bool isPrimaryPeerMacAddressSet() const { return _transceiver.isPrimaryPeerMacAddressSet(); }
    inline const uint8_t* getPrimaryPeerMacAddress() const { return _transceiver.getPrimaryPeerMacAddress(); }
    inline bool isPacketEmpty() const { return _received_data.len == 0 ? true : false;  }
    inline void setPacketEmpty() { _received_data.len = 0; }
    inline const uint8_t* myMacAddress() const {return _transceiver.myMacAddress();}
    esp_err_t broadcastMyMacAddressForBinding(int broadcastCount=DEFAULT_BROADCAST_COUNT, int broadcastDelayMs=DEFAULT_BROADCAST_DELAY_MS) const;
public:
    enum checkPacket_t { CHECK_PACKET, DONT_CHECK_PACKET };
    static int32_t ubyte4float_to_Q4dot12(uint8_t b[4]);
    bool unpackPacket(checkPacket_t checkPacket);
    inline bool unpackPacket() { return unpackPacket(CHECK_PACKET); }
    void resetControls();
    void setDeadZones(int32_t deadZone);
    void setCurrentReadingsToBias();
    inline bool isBiasSet() const { return _biasIsSet; }
    inline int32_t getThrottleQ4dot12Raw() const { return _controls[THROTTLE].rawQ4dot12; }
    inline int32_t getRollQ4dot12Raw() const { return _controls[ROLL].rawQ4dot12; }
    inline int32_t getPitchQ4dot12Raw() const { return _controls[PITCH].rawQ4dot12; }
    inline int32_t getYawQ4dot12Raw() const { return _controls[YAW].rawQ4dot12; }
    int32_t getThrottleQ4dot12 () const;
    int32_t getRollQ4dot12 () const;
    int32_t getPitchQ4dot12() const;
    int32_t getYawQ4dot12() const;
    inline uint8_t getMode() const { return _mode; }
    inline uint8_t getAltMode() const { return _altMode; }
    inline uint8_t getArmButton() const { return _armButton; }
    inline uint8_t getFlipButton() const { return _flipButton; }
    inline uint8_t getProactiveFlag() const { return _proactiveFlag; }
private:
    struct Control {
        int32_t rawQ4dot12 {0};
        int32_t biasQ4dot12 {0};
        int32_t deadZoneQ4dot12 {16}; // last 4 bits of number
    };
    static int32_t normalizedControl(const Control& control, bool raw);
private:
    ESPNOW_Transceiver _transceiver;
    ESPNOW_Transceiver::received_data_t _received_data;
    uint8_t _packet[PACKET_SIZE] {};
    uint8_t _filler[28 - PACKET_SIZE] {};
    Control _controls[CONTROL_COUNT] {};
    int _biasIsSet {false}; //NOTE: if `bool` type is used here then `getMode()` sometimes returns incorrect value
    uint8_t _mode {0};
    uint8_t _altMode {0};
    uint8_t _armButton {0};
    uint8_t _flipButton {0};
    uint8_t _proactiveFlag {0};
};
