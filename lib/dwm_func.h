#ifndef DWM_FUNC_H
#define DWM_FUNC_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <time.h>
#include <stdbool.h>
#include <pthread.h>


// Struttura per la posizione
typedef struct {
	int32_t x;
	int32_t y;
	int32_t z;
	uint8_t qf;
} dwm_pos_t;

/* Le seguenti strutture fino all'asterisco sono prese dall'API ufficiale */
/**
 * @brief Position measurement modes
 */
typedef enum {
	DWM_MEAS_MODE_TWR = 0,//!< DWM_MEAS_MODE_TWR
	DWM_MEAS_MODE_TDOA = 1//!< DWM_MEAS_MODE_TDOA
} dwm_meas_mode_t;

/**
 * @brief Device modes
 */
typedef enum {
	DWM_MODE_TAG = 0,  //!< DWM_MODE_TAG
	DWM_MODE_ANCHOR = 1//!< DWM_MODE_ANCHOR
} dwm_mode_t;

typedef enum {
	DWM_UWB_MODE_OFF = 0,
	DWM_UWB_MODE_PASSIVE = 1,
	DWM_UWB_MODE_ACTIVE = 2
}dwm_uwb_mode_t;

typedef enum {
	DWM_UWB_BH_ROUTING_OFF = 0,
	DWM_UWB_BH_ROUTING_ON = 1,
	DWM_UWB_BH_ROUTING_AUTO = 2,
} dwm_uwb_bh_routing_t;

typedef struct dwm_cfg_common {
	dwm_uwb_mode_t uwb_mode;
	bool fw_update_en;
	bool ble_en;
	bool led_en;
	bool enc_en;
} dwm_cfg_common_t;

typedef struct dwm_cfg_anchor {
	dwm_cfg_common_t common;
	bool bridge;
	bool initiator;
	dwm_uwb_bh_routing_t uwb_bh_routing;
} dwm_cfg_anchor_t;

typedef struct dwm_cfg_tag {
	dwm_cfg_common_t common;
	bool loc_engine_en;
	bool low_power_en;
	bool stnry_en;
	dwm_meas_mode_t meas_mode;
} dwm_cfg_tag_t;

typedef struct dwm_cfg {
	dwm_cfg_common_t common;
	bool loc_engine_en;
	bool low_power_en;
	bool stnry_en;
	dwm_meas_mode_t meas_mode;
	dwm_uwb_bh_routing_t uwb_bh_routing;
	bool bridge;
	bool initiator;
	dwm_mode_t mode;
} dwm_cfg_t;

typedef struct {
	uint8_t pg_delay;
	uint32_t tx_power;
	struct {
		uint8_t pg_delay;
		uint32_t tx_power;
	} compensated;
} dwm_uwb_cfg_t;

/* * */

// File descriptor SPI globale
extern int spi_fd;

// Configurazione SPI
static const char *device = "/dev/spidev0.0"; // Bus 0, Device 0
static uint32_t speed = 1000000; // 1 MHz
static uint8_t mode = 0; // CPOL=0, CPHA=0
static uint8_t bits = 8; // 8 bits per word

// Funzioni
int spi_init();
int spi_transfer(uint8_t *tx_buffer, uint8_t *rx_buffer, int length);
void spi_close();
void ensure_device_in_idle_state();
int dwm_spi_transaction(uint8_t *tx_data, int tx_len, uint8_t *rx_data, int max_rx_len);
int read_status();
int dwm_panid_set(uint16_t panid);
int dwm_panid_get();
int dwm_pos_set(dwm_pos_t *pos);
int dwm_pos_get(dwm_pos_t *pos);
int dwm_cfg_tag_set(dwm_cfg_tag_t* p_cfg);
int dwm_cfg_get(dwm_cfg_t *cfg);
int dwm_cfg_anchor_set(dwm_cfg_anchor_t *p_cfg);
int dwm_reset();
int dwm_loc_get(uint32_t **distances);
int dwm_uwb_cfg_set(uint8_t pg_delay, uint32_t tx_power);
int dwm_uwb_cfg_get(dwm_uwb_cfg_t *cfg);
int dwm_nodeid_get();
int dwm_upd_rate_set(uint16_t ur, uint16_t urs); 

#endif