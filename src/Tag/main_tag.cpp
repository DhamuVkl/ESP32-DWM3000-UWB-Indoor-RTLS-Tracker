#include <SPI.h>
#include <WiFi.h>
#include <WiFiClient.h>

// WiFi Configuration
const char *ssid = "Sxxxxxxxx";
const char *password = "cxxxxxxxx";
const char *host = "192.168.xxx.xx"; // Your PC's IP address
const int port = 7007;             // Choose a port number
WiFiClient client;
bool wifiConnected = false;

// SPI Setup
#define RST_PIN 27
#define CHIP_SELECT_PIN 4

// Scalable Anchor Configuration
#define NUM_ANCHORS 3 // Change this to scale the system
#define TAG_ID 10
#define FIRST_ANCHOR_ID 1 // Starting ID for anchors (1, 2, 3, ...)

// Ranging Configuration
#define FILTER_SIZE 30 // For median filter
#define MIN_DISTANCE 0
#define MAX_DISTANCE 1000.0

// UWB Configuration
#define LEN_RX_CAL_CONF 4
#define LEN_TX_FCTRL_CONF 6
#define LEN_AON_DIG_CFG_CONF 3
#define PMSC_STATE_IDLE 0x3
#define FCS_LEN 2
#define STDRD_SYS_CONFIG 0x188
#define DTUNE0_CONFIG 0x0F
#define SYS_STATUS_FRAME_RX_SUCC 0x2000
#define SYS_STATUS_RX_ERR 0x4279000
#define SYS_STATUS_FRAME_TX_SUCC 0x80
#define PREAMBLE_32 4
#define PREAMBLE_64 8
#define PREAMBLE_128 5
#define PREAMBLE_256 9
#define PREAMBLE_512 11
#define PREAMBLE_1024 2
#define PREAMBLE_2048 10
#define PREAMBLE_4096 3
#define PREAMBLE_1536 6
#define CHANNEL_5 0x0
#define CHANNEL_9 0x1
#define PAC4 0x03
#define PAC8 0x00
#define PAC16 0x01
#define PAC32 0x02
#define DATARATE_6_8MB 0x1
#define DATARATE_850KB 0x0
#define PHR_MODE_STANDARD 0x0
#define PHR_MODE_LONG 0x1
#define PHR_RATE_6_8MB 0x1
#define PHR_RATE_850KB 0x0
#define SPIRDY_MASK 0x80
#define RCINIT_MASK 0x100
#define BIAS_CTRL_BIAS_MASK 0x1F
#define GEN_CFG_AES_LOW_REG 0x00
#define GEN_CFG_AES_HIGH_REG 0x01
#define STS_CFG_REG 0x2
#define RX_TUNE_REG 0x3
#define EXT_SYNC_REG 0x4
#define GPIO_CTRL_REG 0x5
#define DRX_REG 0x6
#define RF_CONF_REG 0x7
#define RF_CAL_REG 0x8
#define FS_CTRL_REG 0x9
#define AON_REG 0xA
#define OTP_IF_REG 0xB
#define CIA_REG1 0xC
#define CIA_REG2 0xD
#define CIA_REG3 0xE
#define DIG_DIAG_REG 0xF
#define PMSC_REG 0x11
#define RX_BUFFER_0_REG 0x12
#define RX_BUFFER_1_REG 0x13
#define TX_BUFFER_REG 0x14
#define ACC_MEM_REG 0x15
#define SCRATCH_RAM_REG 0x16
#define AES_RAM_REG 0x17
#define SET_1_2_REG 0x18
#define INDIRECT_PTR_A_REG 0x1D
#define INDIRECT_PTR_B_REG 0x1E
#define IN_PTR_CFG_REG 0x1F
#define TRANSMIT_DELAY 0x3B9ACA00
#define TRANSMIT_DIFF 0x1FF
#define NS_UNIT 4.0064102564102564    // ns
#define PS_UNIT 15.6500400641025641   // ps
#define SPEED_OF_LIGHT 0.029979245800 // in centimetres per picosecond
#define CLOCK_OFFSET_CHAN_5_CONSTANT -0.5731e-3f
#define CLOCK_OFFSET_CHAN_9_CONSTANT -0.1252e-3f
#define NO_OFFSET 0x0
#define DEBUG_OUTPUT 0
static int ANTENNA_DELAY = 16350;
int led_status = 0;
int destination = 0x0;
int sender = 0x0;

// Initial Radio Configuration
int config[] = {
    CHANNEL_5,         // Channel
    PREAMBLE_128,      // Preamble Length
    9,                 // Preamble Code (Same for RX and TX!)
    PAC8,              // PAC
    DATARATE_6_8MB,    // Datarate
    PHR_MODE_STANDARD, // PHR Mode
    PHR_RATE_850KB     // PHR Rate
};

// Global variables
static int rx_status;
static int tx_status;
static int current_anchor_index = 0; // Index into anchors array
static int curr_stage = 0;

// Anchor data structure
struct AnchorData
{
    int anchor_id; // Anchor ID

    // Timing measurements
    int t_roundA = 0;
    int t_replyA = 0;
    long long rx = 0;
    long long tx = 0;
    int clock_offset = 0;

    // Distance measurements
    float distance = 0;
    float distance_history[FILTER_SIZE] = {0};
    int history_index = 0;
    float filtered_distance = 0;

    // Signal quality metrics
    float signal_strength = 0;    // RSSI in dBm
    float fp_signal_strength = 0; // First Path RSSI in dBm
};

// Dynamic array of anchor data
AnchorData anchors[NUM_ANCHORS];

// Helper functions for anchor management
void initializeAnchors()
{
    for (int i = 0; i < NUM_ANCHORS; i++)
    {
        anchors[i].anchor_id = FIRST_ANCHOR_ID + i;
        // Initialize all other fields to zero (default constructor handles this)
    }
}

AnchorData *getCurrentAnchor()
{
    return &anchors[current_anchor_index];
}

int getCurrentAnchorId()
{
    return anchors[current_anchor_index].anchor_id;
}

void switchToNextAnchor()
{
    current_anchor_index = (current_anchor_index + 1) % NUM_ANCHORS;
}

bool allAnchorsHaveValidData()
{
    for (int i = 0; i < NUM_ANCHORS; i++)
    {
        if (anchors[i].filtered_distance <= 0)
        {
            return false;
        }
    }
    return true;
}

class DWM3000Class
{
public:
    // Chip Setup
    static void spiSelect(uint8_t cs);
    static void begin();
    static void init();
    static void writeSysConfig();
    static void configureAsTX();
    static void setupGPIO();

    // Double-Sided Ranging
    static void ds_sendFrame(int stage);
    static void ds_sendRTInfo(int t_roundB, int t_replyB);
    static int ds_processRTInfo(int t_roundA, int t_replyA, int t_roundB, int t_replyB, int clock_offset);
    static int ds_getStage();
    static bool ds_isErrorFrame();
    static void ds_sendErrorFrame();

    // Radio Settings
    static void setChannel(uint8_t data);
    static void setPreambleLength(uint8_t data);
    static void setPreambleCode(uint8_t data);
    static void setPACSize(uint8_t data);
    static void setDatarate(uint8_t data);
    static void setPHRMode(uint8_t data);
    static void setPHRRate(uint8_t data);

    // Protocol Settings
    static void setMode(int mode);
    static void setTXFrame(unsigned long long frame_data);
    static void setFrameLength(int frame_len);
    static void setTXAntennaDelay(int delay);
    static void setSenderID(int senderID);
    static void setDestinationID(int destID);

    // Status Checks
    static int receivedFrameSucc();
    static int sentFrameSucc();
    static int getSenderID();
    static int getDestinationID();
    static bool checkForIDLE();
    static bool checkSPI();

    // Radio Analytics
    static double getSignalStrength();
    static double getFirstPathSignalStrength();
    static int getTXAntennaDelay();
    static long double getClockOffset();
    static long double getClockOffset(int32_t ext_clock_offset);
    static int getRawClockOffset();
    static float getTempInC();
    static unsigned long long readRXTimestamp();
    static unsigned long long readTXTimestamp();

    // Chip Interaction
    static uint32_t write(int base, int sub, uint32_t data, int data_len);
    static uint32_t write(int base, int sub, uint32_t data);
    static uint32_t read(int base, int sub);
    static uint8_t read8bit(int base, int sub);
    static uint32_t readOTP(uint8_t addr);

    // Delayed Sending Settings
    static void writeTXDelay(uint32_t delay);
    static void prepareDelayedTX();

    // Radio Stage Settings / Transfer and Receive Modes
    static void delayedTXThenRX();
    static void delayedTX();
    static void standardTX();
    static void standardRX();
    static void TXInstantRX();

    // DWM3000 Firmware Interaction
    static void softReset();
    static void hardReset();
    static void clearSystemStatus();

    // Hardware Status Information
    static void pullLEDHigh(int led);
    static void pullLEDLow(int led);

    // Calculation and Conversion
    static double convertToCM(int DWM3000_ps_units);
    static void calculateTXRXdiff();

    // Printing
    static void printRoundTripInformation();
    static void printDouble(double val, unsigned int precision, bool linebreak);

private:
    // Single Bit Settings
    static void setBit(int reg_addr, int sub_addr, int shift, bool b);
    static void setBitLow(int reg_addr, int sub_addr, int shift);
    static void setBitHigh(int reg_addr, int sub_addr, int shift);

    // Fast Commands
    static void writeFastCommand(int cmd);

    // SPI Interaction
    static uint32_t readOrWriteFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t data_len, uint32_t readWriteBit);
    static uint32_t sendBytes(int b[], int lenB, int recLen);

    // Soft Reset Helper Method
    static void clearAONConfig();

    // Other Helper Methods
    static unsigned int countBits(unsigned int number);
    static int checkForDevID();
};

DWM3000Class DWM3000;

// WiFi Functions
void connectToWiFi()
{
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20)
    {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        wifiConnected = true;
        Serial.println("\nWiFi connected");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println("\nFailed to connect to WiFi");
    }
}

void sendDataOverWiFi()
{
    if (!wifiConnected)
    {
        connectToWiFi();
        if (!wifiConnected)
            return;
    }

    if (!client.connected())
    {
        if (!client.connect(host, port))
        {
            Serial.println("Connection to host failed");
            wifiConnected = false;
            return;
        }
    }

    // Create JSON structure dynamically based on number of anchors
    String data = "{\"tag_id\":" + String(TAG_ID) + ",\"anchors\":{";

    for (int i = 0; i < NUM_ANCHORS; i++)
    {
        data += "\"A" + String(anchors[i].anchor_id) + "\":{";
        data += "\"distance\":" + String(anchors[i].filtered_distance, 2) + ",";
        data += "\"raw\":" + String(anchors[i].distance, 2) + ",";
        data += "\"rssi\":" + String(anchors[i].signal_strength, 2) + ",";
        data += "\"fp_rssi\":" + String(anchors[i].fp_signal_strength, 2) + ",";
        data += "\"round_time\":" + String(anchors[i].t_roundA) + ",";
        data += "\"reply_time\":" + String(anchors[i].t_replyA) + ",";
        data += "\"clock_offset\":" + String((double)DWM3000.getClockOffset(anchors[i].clock_offset), 6);
        data += "}";

        // Add comma if not the last anchor
        if (i < NUM_ANCHORS - 1)
        {
            data += ",";
        }
    }

    data += "}}\n";

    client.print(data);

    // For debugging, print the JSON to serial
    Serial.println("Sent JSON data:");
    Serial.println(data);
}

// Helper function to validate distance
bool isValidDistance(float distance)
{
    return (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE);
}

float calculateMedian(float arr[], int size)
{
    float temp[size];
    for (int i = 0; i < size; i++)
    {
        temp[i] = arr[i];
    }

    for (int i = 0; i < size - 1; i++)
    {
        for (int j = i + 1; j < size; j++)
        {
            if (temp[j] < temp[i])
            {
                float t = temp[i];
                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }

    if (size % 2 == 0)
    {
        return (temp[size / 2 - 1] + temp[size / 2]) / 2.0;
    }
    else
    {
        return temp[size / 2];
    }
}

// updateFilteredDistance function
void updateFilteredDistance(AnchorData &data)
{
    data.distance_history[data.history_index] = data.distance;
    data.history_index = (data.history_index + 1) % FILTER_SIZE;

    float valid_distances[FILTER_SIZE];
    int valid_count = 0;

    for (int i = 0; i < FILTER_SIZE; i++)
    {
        if (isValidDistance(data.distance_history[i]))
        {
            valid_distances[valid_count++] = data.distance_history[i];
        }
    }

    if (valid_count > 0)
    {
        data.filtered_distance = calculateMedian(valid_distances, valid_count);
    }
    else
    {
        data.filtered_distance = 0;
    }
}

// Debug print function
void printDebugInfo(int anchor, long long rx, long long tx, int t_round, int t_reply, int clock_offset)
{
    Serial.print("Anchor ");
    Serial.print(anchor);
    Serial.println(" Debug Info:");
    Serial.print("RX timestamp: ");
    Serial.println(rx);
    Serial.print("TX timestamp: ");
    Serial.println(tx);
    Serial.print("t_round: ");
    Serial.println(t_round);
    Serial.print("t_reply: ");
    Serial.println(t_reply);
    Serial.print("Clock offset: ");
    Serial.println(clock_offset);

    int ranging_time = DWM3000.ds_processRTInfo(t_round, t_reply,
                                                DWM3000.read(0x12, 0x04), DWM3000.read(0x12, 0x08), clock_offset);
    Serial.print("Calculated distance: ");
    Serial.println(DWM3000.convertToCM(ranging_time));
}

void printAllDistances()
{
    Serial.print("Distances - ");
    for (int i = 0; i < NUM_ANCHORS; i++)
    {
        Serial.print("A");
        Serial.print(anchors[i].anchor_id);
        Serial.print(": ");
        if (anchors[i].filtered_distance > 0)
        {
            DWM3000.printDouble(anchors[i].filtered_distance, 100, false);
            Serial.print(" cm");
        }
        else
        {
            Serial.print("INVALID");
        }

        if (i < NUM_ANCHORS - 1)
        {
            Serial.print(" | ");
        }
    }
    Serial.println();
}

// Implementation of DWM3000Class methods
void DWM3000Class::spiSelect(uint8_t cs)
{
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);
    delay(5);
}

void DWM3000Class::begin()
{
    delay(5);
    pinMode(CHIP_SELECT_PIN, OUTPUT);
    SPI.begin();
    delay(5);
    spiSelect(CHIP_SELECT_PIN);
    Serial.println("[INFO] SPI ready");
}

void DWM3000Class::init()
{

    if (!checkForDevID())
    {
        Serial.println("[ERROR] Dev ID is wrong! Aborting!");
        return;
    }

    setBitHigh(GEN_CFG_AES_LOW_REG, 0x10, 4);

    while (!checkForIDLE())
    {
        Serial.println("[WARNING] IDLE FAILED (stage 1)");
        delay(100);
    }

    softReset();
    delay(200);

    while (!checkForIDLE())
    {
        Serial.println("[WARNING] IDLE FAILED (stage 2)");
        delay(100);
    }

    uint32_t ldo_low = readOTP(0x04);
    uint32_t ldo_high = readOTP(0x05);
    uint32_t bias_tune = readOTP(0xA);
    bias_tune = (bias_tune >> 16) & BIAS_CTRL_BIAS_MASK;

    if (ldo_low != 0 && ldo_high != 0 && bias_tune != 0)
    {
        write(0x11, 0x1F, bias_tune);
        write(0x0B, 0x08, 0x0100);
    }

    int xtrim_value = readOTP(0x1E);
    xtrim_value = xtrim_value == 0 ? 0x2E : xtrim_value;
    write(FS_CTRL_REG, 0x14, xtrim_value);
    if (DEBUG_OUTPUT)
        Serial.print("xtrim: ");
    if (DEBUG_OUTPUT)
        Serial.println(xtrim_value);

    writeSysConfig();
    write(0x00, 0x3C, 0xFFFFFFFF);
    write(0x00, 0x40, 0xFFFF);
    write(0x0A, 0x00, 0x000900, 3);

    write(0x3, 0x1C, 0x10000240);
    write(0x3, 0x20, 0x1B6DA489);
    write(0x3, 0x38, 0x0001C0FD);
    write(0x3, 0x3C, 0x0001C43E);
    write(0x3, 0x40, 0x0001C6BE);
    write(0x3, 0x44, 0x0001C77E);
    write(0x3, 0x48, 0x0001CF36);
    write(0x3, 0x4C, 0x0001CFB5);
    write(0x3, 0x50, 0x0001CFF5);
    write(0x3, 0x18, 0xE5E5);
    int f = read(0x4, 0x20);
    write(0x6, 0x0, 0x81101C);
    write(0x07, 0x34, 0x4);
    write(0x07, 0x48, 0x14);
    write(0x07, 0x1A, 0x0E);
    write(0x07, 0x1C, 0x1C071134);
    write(0x09, 0x00, 0x1F3C);
    write(0x09, 0x80, 0x81);
    write(0x11, 0x04, 0xB40200);
    write(0x11, 0x08, 0x80030738);
    Serial.println("[INFO] Initialization finished.\n");
}

void DWM3000Class::writeSysConfig()
{
    int usr_cfg = (STDRD_SYS_CONFIG & 0xFFF) | (config[5] << 3) | (config[6] << 4);
    write(GEN_CFG_AES_LOW_REG, 0x10, usr_cfg);

    if (config[2] > 24)
    {
        Serial.println("[ERROR] SCP ERROR! TX & RX Preamble Code higher than 24!");
    }

    int otp_write = 0x1400;
    if (config[1] >= 256)
    {
        otp_write |= 0x04;
    }

    write(OTP_IF_REG, 0x08, otp_write);
    write(DRX_REG, 0x00, 0x00, 1);
    write(DRX_REG, 0x0, config[3]);
    write(STS_CFG_REG, 0x0, 64 / 8 - 1);
    write(GEN_CFG_AES_LOW_REG, 0x29, 0x00, 1);
    write(DRX_REG, 0x0C, 0xAF5F584C);

    int chan_ctrl_val = read(GEN_CFG_AES_HIGH_REG, 0x14);
    chan_ctrl_val &= (~0x1FFF);
    chan_ctrl_val |= config[0];
    chan_ctrl_val |= 0x1F00 & (config[2] << 8);
    chan_ctrl_val |= 0xF8 & (config[2] << 3);
    chan_ctrl_val |= 0x06 & (0x01 << 1);
    write(GEN_CFG_AES_HIGH_REG, 0x14, chan_ctrl_val);

    int tx_fctrl_val = read(GEN_CFG_AES_LOW_REG, 0x24);
    tx_fctrl_val |= (config[1] << 12);
    tx_fctrl_val |= (config[4] << 10);
    write(GEN_CFG_AES_LOW_REG, 0x24, tx_fctrl_val);
    write(DRX_REG, 0x02, 0x81);

    int rf_tx_ctrl_2 = 0x1C071134;
    int pll_conf = 0x0F3C;
    if (config[0])
    {
        rf_tx_ctrl_2 &= ~0x00FFFF;
        rf_tx_ctrl_2 |= 0x000001;
        pll_conf &= 0x00FF;
        pll_conf |= 0x001F;
    }

    write(RF_CONF_REG, 0x1C, rf_tx_ctrl_2);
    write(FS_CTRL_REG, 0x00, pll_conf);
    write(RF_CONF_REG, 0x51, 0x14);
    write(RF_CONF_REG, 0x1A, 0x0E);
    write(FS_CTRL_REG, 0x08, 0x81);
    write(GEN_CFG_AES_LOW_REG, 0x44, 0x02);
    write(PMSC_REG, 0x04, 0x300200);
    write(PMSC_REG, 0x08, 0x0138);

    int success = 0;
    for (int i = 0; i < 100; i++)
    {
        if (read(GEN_CFG_AES_LOW_REG, 0x0) & 0x2)
        {
            success = 1;
            break;
        }
    }

    if (!success)
    {
        Serial.println("[ERROR] Couldn't lock PLL Clock!");
    }
    else
    {
        Serial.println("[INFO] PLL is now locked.");
    }

    int otp_val = read(OTP_IF_REG, 0x08);
    otp_val |= 0x40;
    if (config[0])
        otp_val |= 0x2000;
    write(OTP_IF_REG, 0x08, otp_val);
    write(RX_TUNE_REG, 0x19, 0xF0);

    int ldo_ctrl_val = read(RF_CONF_REG, 0x48);
    int tmp_ldo = (0x105 | 0x100 | 0x4 | 0x1);
    write(RF_CONF_REG, 0x48, tmp_ldo);
    write(EXT_SYNC_REG, 0x0C, 0x020000);
    int l = read(0x04, 0x0C);
    delay(20);
    write(EXT_SYNC_REG, 0x0C, 0x11);

    int succ = 0;
    for (int i = 0; i < 100; i++)
    {
        if (read(EXT_SYNC_REG, 0x20))
        {
            succ = 1;
            break;
        }
        delay(10);
    }

    if (succ)
    {
        Serial.println("[INFO] PGF calibration complete.");
    }
    else
    {
        Serial.println("[ERROR] PGF calibration failed!");
    }

    write(EXT_SYNC_REG, 0x0C, 0x00);
    write(EXT_SYNC_REG, 0x20, 0x01);

    int rx_cal_res = read(EXT_SYNC_REG, 0x14);
    if (rx_cal_res == 0x1fffffff)
    {
        Serial.println("[ERROR] PGF_CAL failed in stage I!");
    }
    rx_cal_res = read(EXT_SYNC_REG, 0x1C);
    if (rx_cal_res == 0x1fffffff)
    {
        Serial.println("[ERROR] PGF_CAL failed in stage Q!");
    }

    write(RF_CONF_REG, 0x48, ldo_ctrl_val);
    write(0x0E, 0x02, 0x01);
    setTXAntennaDelay(ANTENNA_DELAY);
}

void DWM3000Class::configureAsTX()
{
    write(RF_CONF_REG, 0x1C, 0x34);
    write(GEN_CFG_AES_HIGH_REG, 0x0C, 0xFDFDFDFD);
}

void DWM3000Class::setupGPIO()
{
    write(0x05, 0x08, 0xF0);
}

void DWM3000Class::ds_sendFrame(int stage)
{
    setMode(1);
    write(0x14, 0x01, sender & 0xFF);
    write(0x14, 0x02, destination & 0xFF);
    write(0x14, 0x03, stage & 0x7);
    setFrameLength(4);
    TXInstantRX();

    bool error = true;
    for (int i = 0; i < 50; i++)
    {
        if (sentFrameSucc())
        {
            error = false;
            break;
        }
    };
    if (error)
    {
        Serial.println("[ERROR] Could not send frame successfully!");
    }
}

void DWM3000Class::ds_sendRTInfo(int t_roundB, int t_replyB)
{
    setMode(1);
    write(0x14, 0x01, destination & 0xFF);
    write(0x14, 0x02, sender & 0xFF);
    write(0x14, 0x03, 4);
    write(0x14, 0x04, t_roundB);
    write(0x14, 0x08, t_replyB);
    setFrameLength(12);
    TXInstantRX();
}

int DWM3000Class::ds_processRTInfo(int t_roundA, int t_replyA, int t_roundB, int t_replyB, int clk_offset)
{
    if (DEBUG_OUTPUT)
    {
        Serial.println("\nProcessing Information:");
        Serial.print("t_roundA: ");
        Serial.println(t_roundA);
        Serial.print("t_replyA: ");
        Serial.println(t_replyA);
        Serial.print("t_roundB: ");
        Serial.println(t_roundB);
        Serial.print("t_replyB: ");
        Serial.println(t_replyB);
    }

    int reply_diff = t_replyA - t_replyB;
    long double clock_offset = t_replyA > t_replyB ? 1.0 + getClockOffset(clk_offset) : 1.0 - getClockOffset(clk_offset);
    int first_rt = t_roundA - t_replyB;
    int second_rt = t_roundB - t_replyA;
    int combined_rt = (first_rt + second_rt - (reply_diff - (reply_diff * clock_offset))) / 2;
    int combined_rt_raw = (first_rt + second_rt) / 2;
    return combined_rt / 2;
}

int DWM3000Class::ds_getStage()
{
    return read(0x12, 0x03) & 0b111;
}

bool DWM3000Class::ds_isErrorFrame()
{
    return ((read(0x12, 0x00) & 0x7) == 7);
}

void DWM3000Class::ds_sendErrorFrame()
{
    Serial.println("[WARNING] Error Frame sent. Reverting back to stage 0.");
    setMode(7);
    setFrameLength(3);
    standardTX();
}

void DWM3000Class::setChannel(uint8_t data)
{
    if (data == CHANNEL_5 || data == CHANNEL_9)
        config[0] = data;
}

void DWM3000Class::setPreambleLength(uint8_t data)
{
    if (data == PREAMBLE_32 || data == PREAMBLE_64 || data == PREAMBLE_1024 || data == PREAMBLE_256 || data == PREAMBLE_512 || data == PREAMBLE_1024 || data == PREAMBLE_1536 || data == PREAMBLE_2048 || data == PREAMBLE_4096)
        config[1] = data;
}

void DWM3000Class::setPreambleCode(uint8_t data)
{
    if (data <= 12 && data >= 9)
        config[2] = data;
}

void DWM3000Class::setPACSize(uint8_t data)
{
    if (data == PAC4 || data == PAC8 || data == PAC16 || data == PAC32)
        config[3] = data;
}

void DWM3000Class::setDatarate(uint8_t data)
{
    if (data == DATARATE_6_8MB || data == DATARATE_850KB)
        config[4] = data;
}

void DWM3000Class::setPHRMode(uint8_t data)
{
    if (data == PHR_MODE_STANDARD || data == PHR_MODE_LONG)
        config[5] = data;
}

void DWM3000Class::setPHRRate(uint8_t data)
{
    if (data == PHR_RATE_6_8MB || data == PHR_RATE_850KB)
        config[6] = data;
}

void DWM3000Class::setMode(int mode)
{
    write(0x14, 0x00, mode & 0x7);
}

void DWM3000Class::setTXFrame(unsigned long long frame_data)
{
    if (frame_data > ((pow(2, 8 * 8) - FCS_LEN)))
    {
        Serial.println("[ERROR] Frame is too long (> 1023 Bytes - FCS_LEN)!");
        return;
    }
    write(TX_BUFFER_REG, 0x00, frame_data);
}

void DWM3000Class::setFrameLength(int frameLen)
{
    frameLen = frameLen + FCS_LEN;
    int curr_cfg = read(0x00, 0x24);
    if (frameLen > 1023)
    {
        Serial.println("[ERROR] Frame length + FCS_LEN (2) is longer than 1023. Aborting!");
        return;
    }
    int tmp_cfg = (curr_cfg & 0xFFFFFC00) | frameLen;
    write(GEN_CFG_AES_LOW_REG, 0x24, tmp_cfg);
}

void DWM3000Class::setTXAntennaDelay(int delay)
{
    ANTENNA_DELAY = delay;
    write(0x01, 0x04, delay);
}

void DWM3000Class::setSenderID(int senderID)
{
    sender = senderID;
}

void DWM3000Class::setDestinationID(int destID)
{
    destination = destID;
}

int DWM3000Class::receivedFrameSucc()
{
    int sys_stat = read(GEN_CFG_AES_LOW_REG, 0x44);
    if ((sys_stat & SYS_STATUS_FRAME_RX_SUCC) > 0)
    {
        return 1;
    }
    else if ((sys_stat & SYS_STATUS_RX_ERR) > 0)
    {
        return 2;
    }
    return 0;
}

int DWM3000Class::sentFrameSucc()
{
    int sys_stat = read(GEN_CFG_AES_LOW_REG, 0x44);
    if ((sys_stat & SYS_STATUS_FRAME_TX_SUCC) == SYS_STATUS_FRAME_TX_SUCC)
    {
        return 1;
    }
    return 0;
}

int DWM3000Class::getSenderID()
{
    return read(0x12, 0x01) & 0xFF;
}

int DWM3000Class::getDestinationID()
{
    return read(0x12, 0x02) & 0xFF;
}

bool DWM3000Class::checkForIDLE()
{
    return (read(0x0F, 0x30) >> 16 & PMSC_STATE_IDLE) == PMSC_STATE_IDLE || (read(0x00, 0x44) >> 16 & (SPIRDY_MASK | RCINIT_MASK)) == (SPIRDY_MASK | RCINIT_MASK) ? 1 : 0;
}

bool DWM3000Class::checkSPI()
{
    return checkForDevID();
}

double DWM3000Class::getSignalStrength()
{
    int CIRpower = read(0x0C, 0x2C) & 0x1FF;
    int PAC_val = read(0x0C, 0x58) & 0xFFF;
    unsigned int DGC_decision = (read(0x03, 0x60) >> 28) & 0x7;
    double PRF_const = 121.7;
    return 10 * log10((CIRpower * (1 << 21)) / pow(PAC_val, 2)) + (6 * DGC_decision) - PRF_const;
}

double DWM3000Class::getFirstPathSignalStrength()
{
    float f1 = (read(0x0C, 0x30) & 0x3FFFFF) >> 2;
    float f2 = (read(0x0C, 0x34) & 0x3FFFFF) >> 2;
    float f3 = (read(0x0C, 0x38) & 0x3FFFFF) >> 2;
    int PAC_val = read(0x0C, 0x58) & 0xFFF;
    unsigned int DGC_decision = (read(0x03, 0x60) >> 28) & 0x7;
    double PRF_const = 121.7;
    return 10 * log10((pow(f1, 2) + pow(f2, 2) + pow(f3, 2)) / pow(PAC_val, 2)) + (6 * DGC_decision) - PRF_const;
}

int DWM3000Class::getTXAntennaDelay()
{
    int delay = read(0x01, 0x04) & 0xFFFF;
    return delay;
}

long double DWM3000Class::getClockOffset()
{
    if (config[0] == CHANNEL_5)
    {
        return getRawClockOffset() * CLOCK_OFFSET_CHAN_5_CONSTANT / 1000000;
    }
    else
    {
        return getRawClockOffset() * CLOCK_OFFSET_CHAN_9_CONSTANT / 1000000;
    }
}

long double DWM3000Class::getClockOffset(int32_t sec_clock_offset)
{
    if (config[0] == CHANNEL_5)
    {
        return sec_clock_offset * CLOCK_OFFSET_CHAN_5_CONSTANT / 1000000;
    }
    else
    {
        return sec_clock_offset * CLOCK_OFFSET_CHAN_9_CONSTANT / 1000000;
    }
}

int DWM3000Class::getRawClockOffset()
{
    int raw_offset = read(0x06, 0x29) & 0x1FFFFF;
    if (raw_offset & (1 << 20))
    {
        raw_offset |= ~((1 << 21) - 1);
    }
    if (DEBUG_OUTPUT)
    {
        Serial.print("Raw offset: ");
        Serial.println(raw_offset);
    }
    return raw_offset;
}

float DWM3000Class::getTempInC()
{
    write(0x07, 0x34, 0x04);
    write(0x08, 0x00, 0x01);
    while (!(read(0x08, 0x04) & 0x01))
    {
    };
    int res = read(0x08, 0x08);
    res = (res & 0xFF00) >> 8;
    int otp_temp = readOTP(0x09) & 0xFF;
    float tmp = (float)((res - otp_temp) * 1.05f) + 22.0f;
    write(0x08, 0x00, 0x00, 1);
    return tmp;
}

unsigned long long DWM3000Class::readRXTimestamp()
{
    uint32_t ts_low = read(0x0C, 0x00);
    unsigned long long ts_high = read(0x0C, 0x04) & 0xFF;
    unsigned long long rx_timestamp = (ts_high << 32) | ts_low;
    return rx_timestamp;
}

unsigned long long DWM3000Class::readTXTimestamp()
{
    unsigned long long ts_low = read(0x00, 0x74);
    unsigned long long ts_high = read(0x00, 0x78) & 0xFF;
    unsigned long long tx_timestamp = (ts_high << 32) + ts_low;
    return tx_timestamp;
}

uint32_t DWM3000Class::write(int base, int sub, uint32_t data, int dataLen)
{
    return readOrWriteFullAddress(base, sub, data, dataLen, 1);
}

uint32_t DWM3000Class::write(int base, int sub, uint32_t data)
{
    return readOrWriteFullAddress(base, sub, data, 0, 1);
}

uint32_t DWM3000Class::read(int base, int sub)
{
    uint32_t tmp;
    tmp = readOrWriteFullAddress(base, sub, 0, 0, 0);
    if (DEBUG_OUTPUT)
        Serial.println("");
    return tmp;
}

uint8_t DWM3000Class::read8bit(int base, int sub)
{
    return (uint8_t)(read(base, sub) >> 24);
}

uint32_t DWM3000Class::readOTP(uint8_t addr)
{
    write(OTP_IF_REG, 0x04, addr);
    write(OTP_IF_REG, 0x08, 0x02);
    return read(OTP_IF_REG, 0x10);
}

void DWM3000Class::writeTXDelay(uint32_t delay)
{
    write(0x00, 0x2C, delay);
}

void DWM3000Class::prepareDelayedTX()
{
    long long rx_ts = readRXTimestamp();
    uint32_t exact_tx_timestamp = (long long)(rx_ts + TRANSMIT_DELAY) >> 8;
    long long calc_tx_timestamp = ((rx_ts + TRANSMIT_DELAY) & ~TRANSMIT_DIFF) + ANTENNA_DELAY;
    uint32_t reply_delay = calc_tx_timestamp - rx_ts;
    write(0x14, 0x01, sender & 0xFF);
    write(0x14, 0x02, destination & 0xFF);
    write(0x14, 0x03, reply_delay);
    setFrameLength(7);
    writeTXDelay(exact_tx_timestamp);
}

void DWM3000Class::delayedTXThenRX()
{
    writeFastCommand(0x0F);
}

void DWM3000Class::delayedTX()
{
    writeFastCommand(0x3);
}

void DWM3000Class::standardTX()
{
    writeFastCommand(0x01);
}

void DWM3000Class::standardRX()
{
    writeFastCommand(0x02);
}

void DWM3000Class::TXInstantRX()
{
    writeFastCommand(0x0C);
}

void DWM3000Class::softReset()
{
    clearAONConfig();
    write(PMSC_REG, 0x04, 0x1);
    write(PMSC_REG, 0x00, 0x00, 2);
    delay(100);
    write(PMSC_REG, 0x00, 0xFFFF);
    write(PMSC_REG, 0x04, 0x00, 1);
}

void DWM3000Class::hardReset()
{
    pinMode(RST_PIN, OUTPUT);
    digitalWrite(RST_PIN, LOW);
    delay(10);
    pinMode(RST_PIN, INPUT);
}

void DWM3000Class::clearSystemStatus()
{
    write(GEN_CFG_AES_LOW_REG, 0x44, 0x3F7FFFFF);
}

void DWM3000Class::pullLEDHigh(int led)
{
    if (led > 2)
        return;
    led_status = led_status | (1 << led);
    write(0x05, 0x0C, led_status);
}

void DWM3000Class::pullLEDLow(int led)
{
    if (led > 2)
        return;
    led_status = led_status & ~((int)1 << led);
    write(0x05, 0x0C, led_status);
}

double DWM3000Class::convertToCM(int DWM3000_ps_units)
{
    return (double)DWM3000_ps_units * PS_UNIT * SPEED_OF_LIGHT;
}

void DWM3000Class::calculateTXRXdiff()
{
    unsigned long long ping_tx = readTXTimestamp();
    unsigned long long ping_rx = readRXTimestamp();
    long double clk_offset = getClockOffset();
    long double clock_offset = 1.0 + clk_offset;
    long long t_reply = read(RX_BUFFER_0_REG, 0x03);

    if (t_reply == 0)
    {
        return;
    }

    long long t_round = ping_rx - ping_tx;
    long long t_prop = lround((t_round - lround(t_reply * clock_offset)) / 2);
    long double t_prop_ps = t_prop * PS_UNIT;
    long double t_prop_cm = t_prop_ps * SPEED_OF_LIGHT;
    if (t_prop_cm >= 0)
    {
        printDouble(t_prop_cm, 100, false);
        Serial.println("cm");
    }
}

void DWM3000Class::printRoundTripInformation()
{
    Serial.println("\nRound Trip Information:");
    long long tx_ts = readTXTimestamp();
    long long rx_ts = readRXTimestamp();
    Serial.print("TX Timestamp: ");
    Serial.println(tx_ts);
    Serial.print("RX Timestamp: ");
    Serial.println(rx_ts);
}

void DWM3000Class::printDouble(double val, unsigned int precision, bool linebreak)
{
    Serial.print(int(val));
    Serial.print(".");
    unsigned int frac;
    if (val >= 0)
    {
        frac = (val - int(val)) * precision;
    }
    else
    {
        frac = (int(val) - val) * precision;
    }
    if (linebreak)
    {
        Serial.println(frac, DEC);
    }
    else
    {
        Serial.print(frac, DEC);
    }
}

void DWM3000Class::setBit(int reg_addr, int sub_addr, int shift, bool b)
{
    uint8_t tmpByte = read8bit(reg_addr, sub_addr);
    if (b)
    {
        bitSet(tmpByte, shift);
    }
    else
    {
        bitClear(tmpByte, shift);
    }
    write(reg_addr, sub_addr, tmpByte);
}

void DWM3000Class::setBitLow(int reg_addr, int sub_addr, int shift)
{
    setBit(reg_addr, sub_addr, shift, 0);
}

void DWM3000Class::setBitHigh(int reg_addr, int sub_addr, int shift)
{
    setBit(reg_addr, sub_addr, shift, 1);
}

void DWM3000Class::writeFastCommand(int cmd)
{
    if (DEBUG_OUTPUT)
        Serial.print("[INFO] Executing short command: ");
    int header = 0;
    header = header | 0x1;
    header = header | (cmd & 0x1F) << 1;
    header = header | 0x80;
    if (DEBUG_OUTPUT)
        Serial.println(header, BIN);
    int header_arr[] = {header};
    sendBytes(header_arr, 1, 0);
}

uint32_t DWM3000Class::readOrWriteFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t dataLen, uint32_t readWriteBit)
{
    uint32_t header = 0x00;
    if (readWriteBit)
        header = header | 0x80;
    header = header | ((base & 0x1F) << 1);
    if (sub > 0)
    {
        header = header | 0x40;
        header = header << 8;
        header = header | ((sub & 0x7F) << 2);
    }
    uint32_t header_size = header > 0xFF ? 2 : 1;
    uint32_t res = 0;
    if (!readWriteBit)
    {
        int headerArr[header_size];
        if (header_size == 1)
        {
            headerArr[0] = header;
        }
        else
        {
            headerArr[0] = (header & 0xFF00) >> 8;
            headerArr[1] = header & 0xFF;
        }
        res = (uint32_t)sendBytes(headerArr, header_size, 4);
        return res;
    }
    else
    {
        uint32_t payload_bytes = 0;
        if (dataLen == 0)
        {
            if (data > 0)
            {
                uint32_t payload_bits = countBits(data);
                payload_bytes = (payload_bits - (payload_bits % 8)) / 8;
                if ((payload_bits % 8) > 0)
                {
                    payload_bytes++;
                }
            }
            else
            {
                payload_bytes = 1;
            }
        }
        else
        {
            payload_bytes = dataLen;
        }
        int payload[header_size + payload_bytes];
        if (header_size == 1)
        {
            payload[0] = header;
        }
        else
        {
            payload[0] = (header & 0xFF00) >> 8;
            payload[1] = header & 0xFF;
        }
        for (int i = 0; i < payload_bytes; i++)
        {
            payload[header_size + i] = (data >> i * 8) & 0xFF;
        }
        res = (uint32_t)sendBytes(payload, 2 + payload_bytes, 0);
        return res;
    }
}

uint32_t DWM3000Class::sendBytes(int b[], int lenB, int recLen)
{
    digitalWrite(CHIP_SELECT_PIN, LOW);
    for (int i = 0; i < lenB; i++)
    {
        SPI.transfer(b[i]);
    }
    int rec;
    uint32_t val, tmp;
    if (recLen > 0)
    {
        for (int i = 0; i < recLen; i++)
        {
            tmp = SPI.transfer(0x00);
            if (i == 0)
            {
                val = tmp;
            }
            else
            {
                val |= (uint32_t)tmp << 8 * i;
            }
        }
    }
    else
    {
        val = 0;
    }
    digitalWrite(CHIP_SELECT_PIN, HIGH);
    return val;
}

void DWM3000Class::clearAONConfig()
{
    write(AON_REG, NO_OFFSET, 0x00, 2);
    write(AON_REG, 0x14, 0x00, 1);
    write(AON_REG, 0x04, 0x00, 1);
    write(AON_REG, 0x04, 0x02);
    delay(1);
}

unsigned int DWM3000Class::countBits(unsigned int number)
{
    return (int)log2(number) + 1;
}

int DWM3000Class::checkForDevID()
{
    int res = read(GEN_CFG_AES_LOW_REG, NO_OFFSET);
    if (res != 0xDECA0302 && res != 0xDECA0312)
    {
        Serial.println("[ERROR] DEV_ID IS WRONG!");
        return 0;
    }
    return 1;
}

void setup()
{
    Serial.begin(115200);
    // Initialize anchor array
    initializeAnchors();

    Serial.print("Initialized ");
    Serial.print(NUM_ANCHORS);
    Serial.println(" anchors:");
    for (int i = 0; i < NUM_ANCHORS; i++)
    {
        Serial.print("  Anchor ");
        Serial.print(i);
        Serial.print(" - ID: ");
        Serial.println(anchors[i].anchor_id);
    }

    // Connect to WiFi first
    connectToWiFi();

    // Initialize UWB
    DWM3000.begin();
    DWM3000.hardReset();
    delay(200);

    if (!DWM3000.checkSPI())
    {
        Serial.println("[ERROR] Could not establish SPI Connection to DWM3000!");
        while (1)
            ;
    }

    while (!DWM3000.checkForIDLE())
    {
        Serial.println("[ERROR] IDLE1 FAILED\r");
        delay(1000);
    }

    DWM3000.softReset();
    delay(200);

    if (!DWM3000.checkForIDLE())
    {
        Serial.println("[ERROR] IDLE2 FAILED\r");
        while (1)
            ;
    }

    DWM3000.init();
    DWM3000.setupGPIO();
    DWM3000.setTXAntennaDelay(16350);
    DWM3000.setSenderID(TAG_ID);

    Serial.println("> TAG - Three Anchor Ranging System <");
    Serial.println("> With WiFi Communication <\n");
    Serial.println("[INFO] Setup is finished.");
    Serial.print("Antenna delay set to: ");
    Serial.println(DWM3000.getTXAntennaDelay());

    DWM3000.configureAsTX();
    DWM3000.clearSystemStatus();
}

void loop()
{
    AnchorData *currentAnchor = getCurrentAnchor();
    int currentAnchorId = getCurrentAnchorId();

    switch (curr_stage)
    {
    case 0: // Start ranging with current target
        // Reset timing measurements for current anchor
        currentAnchor->t_roundA = 0;
        currentAnchor->t_replyA = 0;

        DWM3000.setDestinationID(currentAnchorId);
        DWM3000.ds_sendFrame(1);
        currentAnchor->tx = DWM3000.readTXTimestamp();
        curr_stage = 1;
        break;

    case 1: // Await first response
        if (rx_status = DWM3000.receivedFrameSucc())
        {
            DWM3000.clearSystemStatus();
            if (rx_status == 1)
            {
                if (DWM3000.ds_isErrorFrame())
                {
                    Serial.print("[WARNING] Error frame from Anchor ");
                    Serial.print(currentAnchorId);
                    Serial.print("! Signal strength: ");
                    Serial.print(DWM3000.getSignalStrength());
                    Serial.println(" dBm");
                    curr_stage = 0;
                }
                else if (DWM3000.ds_getStage() != 2)
                {
                    Serial.print("[WARNING] Unexpected stage from Anchor ");
                    Serial.print(currentAnchorId);
                    Serial.print(": ");
                    Serial.println(DWM3000.ds_getStage());
                    DWM3000.ds_sendErrorFrame();
                    curr_stage = 0;
                }
                else
                {
                    curr_stage = 2;
                }
            }
            else
            {
                Serial.print("[ERROR] Receiver Error from Anchor ");
                Serial.println(currentAnchorId);
                DWM3000.clearSystemStatus();
            }
        }
        break;

    case 2: // Response received. Send second ranging
        currentAnchor->rx = DWM3000.readRXTimestamp();
        DWM3000.ds_sendFrame(3);

        currentAnchor->t_roundA = currentAnchor->rx - currentAnchor->tx;
        currentAnchor->tx = DWM3000.readTXTimestamp();
        currentAnchor->t_replyA = currentAnchor->tx - currentAnchor->rx;

        curr_stage = 3;
        break;

    case 3: // Await second response
        if (rx_status = DWM3000.receivedFrameSucc())
        {
            DWM3000.clearSystemStatus();
            if (rx_status == 1)
            {
                if (DWM3000.ds_isErrorFrame())
                {
                    Serial.print("[WARNING] Error frame from Anchor ");
                    Serial.println(currentAnchorId);
                    curr_stage = 0;
                }
                else
                {
                    currentAnchor->clock_offset = DWM3000.getRawClockOffset();
                    curr_stage = 4;
                }
            }
            else
            {
                Serial.print("[ERROR] Receiver Error from Anchor ");
                Serial.println(currentAnchorId);
                DWM3000.clearSystemStatus();
            }
        }
        break;

    case 4: // Response received. Calculating results
    {
        int ranging_time = DWM3000.ds_processRTInfo(
            currentAnchor->t_roundA,
            currentAnchor->t_replyA,
            DWM3000.read(0x12, 0x04),
            DWM3000.read(0x12, 0x08),
            currentAnchor->clock_offset);

        currentAnchor->distance = DWM3000.convertToCM(ranging_time);
        currentAnchor->signal_strength = DWM3000.getSignalStrength();
        currentAnchor->fp_signal_strength = DWM3000.getFirstPathSignalStrength();
        updateFilteredDistance(*currentAnchor);
    }

        // Print current distances
        printAllDistances();

        // Send data over WiFi if all anchors have valid data
        if (allAnchorsHaveValidData())
        {
            sendDataOverWiFi();
        }

        // Switch to next anchor
        switchToNextAnchor();
        curr_stage = 0;
        break;

    default:
        Serial.print("Entered stage (");
        Serial.print(curr_stage);
        Serial.println("). Reverting back to stage 0");
        curr_stage = 0;
        break;
    }
}