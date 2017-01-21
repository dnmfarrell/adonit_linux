/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2011  Nokia Corporation
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <assert.h>
#include <glib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <getopt.h>

#include "lib/uuid.h"
#include <btio/btio.h>
#include "att.h"
#include "gattrib.h"
#include "gatt.h"
#include "gatttool.h"

#include "log.h"
#include "uinput.h"

static GIOChannel *iochannel = NULL;
static GAttrib *attrib = NULL;
static GMainLoop *event_loop;

static gchar *opt_src = NULL;
static gchar *opt_dst = NULL;
static gchar *opt_dst_type = NULL;
static gchar *opt_sec_level = NULL;
static int opt_mtu = 0;
static int start;
static int end;

static enum state {
    STATE_DISCONNECTED=0,
    STATE_CONNECTING=1,
    STATE_CONNECTED=2
} conn_state;

static const char
*tag_RESPONSE  = "rsp",
    *tag_ERRCODE   = "code",
    *tag_HANDLE    = "hnd",
    *tag_UUID      = "uuid",
    *tag_DATA      = "d",
    *tag_CONNSTATE = "state",
    *tag_SEC_LEVEL = "sec",
    *tag_MTU       = "mtu",
    *tag_DEVICE    = "dst",
    *tag_RANGE_START = "hstart",
    *tag_RANGE_END = "hend",
    *tag_PROPERTIES= "props",
    *tag_VALUE_HANDLE = "vhnd";

static const char
*rsp_ERROR     = "err",
    *rsp_STATUS    = "stat",
    *rsp_NOTIFY    = "ntfy",
    *rsp_IND       = "ind",
    *rsp_DISCOVERY = "find",
    *rsp_DESCRIPTORS = "desc",
    *rsp_READ      = "rd",
    *rsp_WRITE     = "wr";

static const char
*err_CONN_FAIL = "connfail",
    *err_COMM_ERR  = "comerr",
    *err_PROTO_ERR = "protoerr",
    *err_NOT_FOUND = "notfound",
    *err_BAD_CMD   = "badcmd",
    *err_BAD_PARAM = "badparam",
    *err_BAD_STATE = "badstate";

static const char
*st_DISCONNECTED = "disc",
    *st_CONNECTING   = "tryconn",
    *st_CONNECTED    = "conn";

struct uinput_info uinfo;

static int fd;
struct input_event event;

int prev_btn_0;
int prev_btn_1;

int verbose = 0;
char* touchscreen_device = NULL;
char* pen_address = NULL;

void write_event(uint16_t type, uint16_t code, int32_t value) {
    struct input_event ev;

    gettimeofday(&ev.time, 0);

    ev.type = type;
    ev.code = code;
    ev.value = value;

    uinput_write_event(&uinfo, &ev);
}

void pen_update(uint16_t p)
{
    int btn_0 = (p & 0x1);
    int btn_1 = (p & 0x2) >> 1;

    if (btn_0 != prev_btn_0) {
        write_event(EV_KEY, BTN_0, btn_0);
    }
    prev_btn_0 = btn_0;

    if (btn_1 != prev_btn_1) {
        write_event(EV_KEY, BTN_1, btn_1);
    }
    prev_btn_1 = btn_1;

    write_event(EV_ABS, ABS_PRESSURE, (p >> 5) & 0x7ff);

    write_event(EV_SYN, SYN_REPORT, 0);
}

gboolean touchscreen_update(GIOChannel *chan, GIOCondition cond, gpointer user_data) {
    struct input_event *ev;
    gsize size;
    gchar buffer[24];
    GError *error = NULL;

    if (error) {
        LOG("%s", error->message);
    }

    while(G_IO_STATUS_NORMAL == g_io_channel_read_chars(chan, buffer, sizeof(struct input_event), &size, &error)) {
        ev = (struct input_event *)buffer;

        if (ev->type == EV_SYN) {
            //printf("PACK %i %i\n", ev->code, ev->value);
        } else if (ev->type == EV_KEY) {
            //if (ev->code == BTN_TOUCH) {
            //    ev->code = BTN_TOOL_PEN;
            //}

            uinput_write_event(&uinfo, ev);
        } else if (ev->type == EV_ABS) {
            if (ev->code == ABS_X || ev->code == ABS_Y) {
                uinput_write_event(&uinfo, ev);
            }
        }
    }

    write_event(EV_SYN, SYN_REPORT, 0);

    return TRUE;
}

static void resp_begin(const char *rsptype)
{
    printf("%s=$%s", tag_RESPONSE, rsptype);
}

static void send_sym(const char *tag, const char *val)
{
    printf(" %s=$%s", tag, val);
}
static void resp_end()
{
    printf("\n");
    fflush(stdout);
}

static void resp_error(const char *errcode)
{
    resp_begin(rsp_ERROR);
    send_sym(tag_ERRCODE, errcode);
    resp_end();
}

static void set_state(enum state st)
{
    conn_state = st;
}

static void events_handler(const uint8_t *pdu, uint16_t len, gpointer user_data)
{
    uint8_t *opdu;
    uint8_t evt;
    uint16_t handle, i, olen;
    size_t plen;

    evt = pdu[0];

    if ( evt != ATT_OP_HANDLE_NOTIFY && evt != ATT_OP_HANDLE_IND )
    {
        printf("#Invalid opcode %02X in event handler??\n", evt);
        return;
    }

    assert( len >= 3 );
    handle = att_get_u16(&pdu[1]);

    uint16_t value = pdu[3] << 8 | pdu[4];

    pen_update(value);

    if (evt == ATT_OP_HANDLE_NOTIFY)
        return;

    opdu = g_attrib_get_buffer(attrib, &plen);
    olen = enc_confirmation(opdu, plen);

    if (olen > 0)
        g_attrib_send(attrib, 0, opdu, olen, NULL, NULL, NULL);
}

static void connect_cb(GIOChannel *io, GError *err, gpointer user_data)
{
    printf("Connected.");
    if (err) {
        set_state(STATE_DISCONNECTED);
        resp_error(err_CONN_FAIL);
        printf("# Connect error: %s\n", err->message);
        return;
    }


    attrib = g_attrib_new(iochannel);
    g_attrib_register(attrib, ATT_OP_HANDLE_NOTIFY, GATTRIB_ALL_HANDLES,
            events_handler, attrib, NULL);
    g_attrib_register(attrib, ATT_OP_HANDLE_IND, GATTRIB_ALL_HANDLES,
            events_handler, attrib, NULL);
    set_state(STATE_CONNECTED);

    uint8_t value = 0x1;
    gatt_write_char(attrib, 0x000c, &value, 1, NULL, NULL);
}

static void disconnect_io()
{
    if (conn_state == STATE_DISCONNECTED)
        return;

    uint8_t value = 0x0;
    gatt_write_char(attrib, 0x000c, &value, 1, NULL, NULL);

    g_attrib_unref(attrib);
    attrib = NULL;
    opt_mtu = 0;

    g_io_channel_shutdown(iochannel, FALSE, NULL);
    g_io_channel_unref(iochannel);
    iochannel = NULL;

    set_state(STATE_DISCONNECTED);
}
static gboolean channel_watcher(GIOChannel *chan, GIOCondition cond,
        gpointer user_data)
{
    disconnect_io();

    return FALSE;
}

int init_touchscreen() {
    if (touchscreen_device == NULL) {
        LOG("No touchscreen device specified. Please use the --touchscreen option to specify one.\n");
        exit(0);
    }

    int fd = open(touchscreen_device, O_RDONLY | O_NONBLOCK);

    int grab = 1;
    ioctl(fd, EVIOCGRAB, &grab);

    return fd;
}

int main(int argc, char *argv[])
{
    int c;

    while (1) {
        int this_option_optind = optind ? optind : 1;
        int option_index = 0;

        static struct option long_options[] = {
            {"verbose",	    no_argument,       0,  	'v' },
            {"touchscreen", required_argument,  0,  't' },
            {"pen",         required_argument,  0,  'p' },
            {0,             0,                  0,  0   }
        };

        c = getopt_long(argc, argv, "v",
                long_options, &option_index);

        if (c == -1) {
            break;
        }

        switch (c) {
            case 'v':
                verbose = 1;
                break;
            case 'p':
                pen_address = optarg;
                break;
            case 't':
                touchscreen_device = optarg;
                break;
            case '?':
                break;
        }
    }

    GIOChannel *touchscreen;
    //gint events;

    opt_sec_level = g_strdup("low");

    opt_dst = g_strdup(pen_address);
    opt_dst_type = g_strdup("public");

    LOG("# " __FILE__ " built at " __TIME__ " on " __DATE__ "\n\n");

    if (opt_dst == NULL) {
        LOG("No pen device address specified. Please use the --pen option to specify one.\n");
        return 1;
    }

    event_loop = g_main_loop_new(NULL, FALSE);

    uinfo.create_mode = WDAEMON_CREATE;

    LOG("Opening uinput device...\n");
    if (uinput_create(&uinfo)) {
        return 1;
    } else {
        LOG("uinput device created successfully.\n");
    }

    touchscreen = g_io_channel_unix_new(init_touchscreen());

    g_io_channel_set_encoding(touchscreen, NULL, NULL);

    g_io_add_watch(touchscreen, G_IO_IN | G_IO_HUP | G_IO_ERR, touchscreen_update, NULL);

    iochannel = gatt_connect(NULL, opt_dst, opt_dst_type, opt_sec_level,
            0, 0, connect_cb);

    if (iochannel == NULL) {
        LOG("Couldn't connect to Bluetooth device.\n");
        return 1;
    } else {
        g_io_add_watch(iochannel, G_IO_HUP, channel_watcher, NULL);
    }

    VLOG("Starting loop.");
    g_main_loop_run(event_loop);

    disconnect_io();
    fflush(stdout);
    g_main_loop_unref(event_loop);

    g_io_channel_shutdown(touchscreen, FALSE, NULL);
    g_io_channel_unref(touchscreen);
    touchscreen = NULL;

    g_free(opt_src);
    g_free(opt_dst);
    g_free(opt_sec_level);

    ioctl(fd, EVIOCGRAB, NULL);
    close(fd);

    return EXIT_SUCCESS;
}
