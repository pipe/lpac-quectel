#include "at.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdbool.h>
#include <termios.h>

#include <euicc/interface.h>
#include <euicc/hexutil.h>

static int logic_channel = 0;
static int uart_fd =0;
static int at_debug =0;
static char buffer[10240];


static int at_expect(char **response, const char *expected)
{
    int bytesAv = 0;
    char * ip;
    char *ex;
    char *eol;
    int done = false;
    int ret =0;
    int red =0;
    if (response)
        *response = NULL;
    ip = buffer;
    while (!done)
    {
        if ( ioctl (uart_fd,FIONREAD,&bytesAv) < 0 )
        {
            ret = -1;
            done = true;
        }
        else if (bytesAv == 0)
        {
            usleep(100000);
        }
        else
        {
            if ((sizeof(buffer)-(ip-buffer)) > bytesAv)
            {
                red=read(uart_fd,ip,bytesAv);
                if (red <1)
                {
                    ret = -1;
                    done = true;
                }
                else
                {
                    if (at_debug == 1)
                    {
                        write(1,"<",1);
                        write(1,ip,red);
                    }
                    ip += red;
                    *(ip+1) = '\0';
                }
                if (strstr(buffer, "ERROR") != NULL)
                {
                    ret = -1;
                    done = true;
                }
                else if(strstr(buffer, "OK") != NULL)
                {
                    ret = 0;
                    done = true;
                    if ((expected != NULL)&& ((ex = strstr(buffer, expected)) != NULL))
                    {
                        if (response)
                        {
                            eol = strstr(ex,"\r\n");
                            *eol = '\0';
                            *response = strdup(ex + strlen(expected));
                            if (at_debug ==1){
                                write(1,"=",1);
                                write(1,*response,strlen(*response));
                            }
                        }
                    }
                }
            }
            else
            {
                ret = -1;
                done = true;
            }
        }
    }
    return ret;
}

static int writeString(char * st){
    int len;
    int w;
    char *cp;
    char c;
    len = strlen(st);
    cp = st;

    if (at_debug >0)
    {
        write(1,">",1);
        write(1,st,strlen(st));
    }
    while (len > 0)
    {
	c = 0x7f & *cp;
        if (at_debug >0)
    	{
            write(1,&c,1);
        }
        w = write(uart_fd,&c,1);
	if (w < 0){
	   return -1;
        }
	len -= w;
	cp += w;
	usleep(10000);
    }
}
static int apdu_interface_connect(struct euicc_ctx *ctx)
{
    const char *device;
    struct termios tty;
    int mcs;
     
    logic_channel = 0;
    
    if (getenv("AT_DEBUG"))
        at_debug = 1;
    
    if (!(device = getenv("AT_DEVICE")))
    {
        device = "/dev/ttyUSB2";
    }
    uart_fd = open(device, O_RDWR);
    if (uart_fd < 0)
    {
        fprintf(stderr, "Failed to open device: %s\n", device);
        return -1;
    }
    
    tcgetattr(uart_fd,&tty);
    cfsetospeed(&tty,B115200);
    cfsetispeed(&tty,B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag = IGNBRK;
    tty.c_lflag =0;
    tty.c_oflag =0;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cc[VMIN]=1;
    tty.c_cc[VTIME]=5;
    tty.c_iflag &= ~(IXON|IXOFF|IXANY);
    tty.c_cflag &=~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tcsetattr(uart_fd,TCSANOW,&tty);
    /* ioctl(uart_fd,TIOCMODG,&mcs);
    mcs |= TIOCM_RTS;
    ioctl(uart_fd,TIOCMODS,&mcs);
    */



    writeString("AT+CMEE=2\r\n");
    if (at_expect(NULL, NULL))
    {
        fprintf(stderr, "Device refused extended error codes\n");
        return -1;
    }
    writeString("AT+CCHO=?\r\n");
    if (at_expect(NULL, NULL))
    {
        fprintf(stderr, "Device missing AT+CCHO support\n");
        return -1;
    }
    writeString("AT+CCHC=?\r\n");
    if (at_expect(NULL, NULL))
    {
        fprintf(stderr, "Device missing AT+CCHC support\n");
        return -1;
    }
    writeString("AT+CGLA=?\r\n");
    if (at_expect(NULL, NULL))
    {
        fprintf(stderr, "Device missing AT+CGLA support\n");
        return -1;
    }
    
    return 0;
}

static void apdu_interface_disconnect(struct euicc_ctx *ctx)
{
    close(uart_fd);
    logic_channel = 0;
}

static int apdu_interface_transmit(struct euicc_ctx *ctx, uint8_t **rx, uint32_t *rx_len, const uint8_t *tx, uint32_t tx_len)
{
    int fret = 0;
    int ret;
    char *response = NULL;
    char *hexstr = NULL;
    int mxal =0;
    char *op;
    char *wout;
    char *wp;
    
    
    *rx = NULL;
    *rx_len = 0;
    
    if (!logic_channel)
    {
        return -1;
    }
    
    mxal = 15+(2*tx_len)+2+10;
    wout = calloc((mxal+1)/2,2);
    wp = wout;
    
    wp += snprintf(wp, mxal,"AT+CGLA=%d,%u,\"", logic_channel, tx_len * 2);
    for (uint32_t i = 0; i < tx_len; i++)
    {
        wp+=snprintf(wp,mxal-(wp-wout), "%02X", (uint8_t)(tx[i] & 0xFF));
    }
    wp+=snprintf(wp,mxal-(wp-wout), "\"\r\n");
    writeString(wout);
    usleep(10000);
    if (at_expect(&response, "+CGLA: "))
    {
        goto err;
    }
    if (response == NULL)
    {
        goto err;
    }
    
    strtok(response, ",");
    hexstr = strtok(NULL, ",");
    if (!hexstr)
    {
        goto err;
    }
    if (hexstr[0] == '"')
    {
        hexstr++;
    }
    hexstr[strcspn(hexstr, "\"")] = '\0';
    
    *rx_len = strlen(hexstr) / 2;
    *rx = malloc(*rx_len);
    if (!*rx)
    {
        goto err;
    }
    
    ret = euicc_hexutil_hex2bin_r(*rx, *rx_len, hexstr, strlen(hexstr));
    if (ret < 0)
    {
        goto err;
    }
    *rx_len = ret;
    
    goto exit;
    
err:
    fret = -1;
    free(*rx);
    *rx = NULL;
    *rx_len = 0;
exit:
    free(wout);
    free(response);
    return fret;
}

static int apdu_interface_logic_channel_open(struct euicc_ctx *ctx, const uint8_t *aid, uint8_t aid_len)
{
    char *response;
    char obuf[1000];
    int tail =0;
    
    if (logic_channel)
    {
        return logic_channel;
    }
    
    for (int i = 1; i <= 4; i++)
    {
        tail = snprintf(obuf,999, "AT+CCHC=%d\r\n", i);
        obuf[tail] = '\0';
        writeString(obuf);
        at_expect(NULL, NULL);
    }
    tail = snprintf(obuf,999, "AT+CCHO=\"");
    for (int i = 0; i < aid_len; i++)
    {
        tail+=snprintf(obuf+tail,999-tail, "%02X", (uint8_t)(aid[i] & 0xFF));
    }
    tail+=snprintf(obuf+tail,999-tail, "\"\r\n");
    obuf[tail] = '\0';
    writeString(obuf);
    if (at_expect(&response, "+CCHO: "))
    {
        return -1;
    }
    if (response == NULL)
    {
        return -1;
    }
    logic_channel = atoi(response);
    
    return logic_channel;
}

static void apdu_interface_logic_channel_close(struct euicc_ctx *ctx, uint8_t channel)
{
    char obuf[1000];
    int tail =0;
    
    if (!logic_channel)
    {
        return;
    }
    tail = snprintf(obuf,999, "AT+CCHC=%d\r\n", logic_channel);
    obuf[tail] = '\0';
    writeString(obuf);
    at_expect(NULL, NULL);
}

static int libapduinterface_init(struct euicc_apdu_interface *ifstruct)
{
    memset(ifstruct, 0, sizeof(struct euicc_apdu_interface));
    
    ifstruct->connect = apdu_interface_connect;
    ifstruct->disconnect = apdu_interface_disconnect;
    ifstruct->logic_channel_open = apdu_interface_logic_channel_open;
    ifstruct->logic_channel_close = apdu_interface_logic_channel_close;
    ifstruct->transmit = apdu_interface_transmit;
    
    return 0;
}

static int libapduinterface_main(int argc, char **argv)
{
    return 0;
}

static void libapduinterface_fini(void)
{
}

const struct euicc_driver driver_apdu_at = {
    .type = DRIVER_APDU,
    .name = "at",
    .init = (int (*)(void *))libapduinterface_init,
    .main = libapduinterface_main,
    .fini = libapduinterface_fini,
};
