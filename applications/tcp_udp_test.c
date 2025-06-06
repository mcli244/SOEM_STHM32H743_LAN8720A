#include <rtthread.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <netdev.h>

#define TCP_SERVER_PORT 5000
#define UDP_SERVER_PORT 6000
#define BUF_SIZE (1500)
static int cnt = 0;

char *get_ip_addr(char *netif_name)
{
    static char *ip_addr[16] = {0};
    struct netdev *netif;

    /* 通常默认网卡名是 "e0"、"w0"、"ap0" 之类 */
    netif = netdev_get_by_name(netif_name);
    if (netif)
    {
        rt_strncpy(ip_addr, inet_ntoa(netif->ip_addr), sizeof(ip_addr));

        rt_kprintf("IP Address : %s\n", inet_ntoa(netif->ip_addr));
        rt_kprintf("Gateway    : %s\n", inet_ntoa(netif->gw));
        rt_kprintf("Netmask    : %s\n", inet_ntoa(netif->netmask));
    }
    else
    {
        rt_kprintf("No network interface found!\n");
    }
    return ip_addr;
}


void do_tcp_server_test(void)
{
    int server_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len;
    int ret;
    int cnt = 0;

    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0)
    {
        rt_kprintf("Socket create failed\n");
        return;
    }

    // 加这几行允许端口复用
    int opt = 1;
    setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(TCP_SERVER_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        rt_kprintf("Bind failed\n");
        closesocket(server_sock);
        return;
    }

    if (listen(server_sock, 5) < 0)
    {
        rt_kprintf("Listen failed\n");
        closesocket(server_sock);
        return;
    }

    rt_kprintf("TCP server is listening on port %d\n", TCP_SERVER_PORT);
    addr_len = sizeof(client_addr);
    client_sock = accept(server_sock, (struct sockaddr *)&client_addr, &addr_len);
    if (client_sock < 0)
    {
        rt_kprintf("Accept failed\n");
        closesocket(server_sock);
        return;
    }

    rt_kprintf("Accepted connection from %s:%d\n",
               inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

    char *buf = rt_malloc(BUF_SIZE);
    if (buf == RT_NULL)
    {
        rt_kprintf("Memory allocation failed\n");
        closesocket(client_sock);
        closesocket(server_sock);
        return;
    }

    while (1)
    {
        ret = recv(client_sock, buf, BUF_SIZE - 1, 0);
        if (ret > 0)
        {
            buf[ret] = '\0';
            send(client_sock, buf, ret, 0); // 回显

            if (strcmp(buf, "exit") == 0)
            {
                rt_kprintf("Client requested to close connection.\n");
                break;
            }
        }
        else if (ret == 0)
        {
            rt_kprintf("Client closed the connection.\n");
            break;
        }
        else
        {
            rt_kprintf("recv error: %d\n", errno);
            break;
        }

        cnt++;
    }

    closesocket(client_sock);
    closesocket(server_sock);
    rt_free(buf);
    rt_kprintf("TCP server test finished, total received messages: %d\n", cnt);
}

int tcp_server_test(void)
{
  rt_thread_t tid;
  tid = rt_thread_create("tcp_server_test",
                         do_tcp_server_test,
                         RT_NULL,
                         1024 * 1.5,
                         25,
                         2);
  if (tid != RT_NULL)
  {
    rt_thread_startup(tid);
  }
  else
  {
    rt_kprintf("state = -RT_ERROR\n");
  }
  return 0;
}
MSH_CMD_EXPORT(tcp_server_test, TCP server test);

extern int dm_irq_cnt, dm_pkg_max;
int tcp_server_show(void)
{
    rt_kprintf("TCP server test running, received %d messages. dm_irq_cnt:%d dm_pkg_max:%d\n", cnt, dm_irq_cnt, dm_pkg_max);
    return 0;
}
MSH_CMD_EXPORT(tcp_server_show, TCP server test);

void udp_server_test(void)
{
    int sockfd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len;
    int ret;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        rt_kprintf("Socket create failed\n");
        return;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(UDP_SERVER_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        rt_kprintf("Bind failed\n");
        closesocket(sockfd);
        return;
    }

    char *buf = rt_malloc(BUF_SIZE);
    if(buf == RT_NULL)
    {
        rt_kprintf("Memory allocation failed\n");
        closesocket(sockfd);
        return;
    }

    rt_kprintf("UDP server listening on port %d\n", UDP_SERVER_PORT);


    while (1)
    {
        addr_len = sizeof(client_addr);
        ret = recvfrom(sockfd, buf, BUF_SIZE-1, 0, (struct sockaddr *)&client_addr, &addr_len);
        if (ret > 0)
        {
            buf[ret] = '\0';
            // rt_kprintf("Received: %s\n", buf);
            sendto(sockfd, buf, ret, 0, (struct sockaddr *)&client_addr, addr_len); // Echo
            if(strcmp(buf, "exit") == 0)
            {
                rt_kprintf("Client requested to close connection.\n");
                break; // Exit the loop if client sends "exit"
            }
        }
    }
    closesocket(sockfd);
    rt_free(buf);
}
MSH_CMD_EXPORT(udp_server_test, UDP server test);

