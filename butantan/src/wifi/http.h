#include <stdio.h>
#include <stdlib.h>
#include <zephyr/net/socket.h>

void nslookup(const char * hostname, struct zsock_addrinfo **results);
void print_addrinfo_results(struct zsock_addrinfo **results);
void http_get(int sock, char * hostname, char * url);
void http_post(int sock, char * hostname, char * url, uint8_t * data, int data_length);
int connect_socket(struct zsock_addrinfo **results, uint16_t port);
