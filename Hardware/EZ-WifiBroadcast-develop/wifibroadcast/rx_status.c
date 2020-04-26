#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <resolv.h>
#include <string.h>
#include <utime.h>
#include <unistd.h>
#include <getopt.h>
#include <pcap.h>
#include <endian.h>
#include <fcntl.h>
#include <sys/mman.h>

#include "lib.h"

wifibroadcast_rx_status_t *status_memory_open(char* shm_file, char* line) {
	
	int fd;

	for(;;) {
		fd = shm_open(shm_file, O_RDWR, S_IRUSR | S_IWUSR);
		if(fd > 0) {
			break;
		}
		// Goto line/row 1/1
		printf("\033[%s;1H",line);
		printf("Waiting for rx to be started ...\n");
		usleep(1e5);
	}

	if (ftruncate(fd, sizeof(wifibroadcast_rx_status_t)) == -1) {
		perror("ftruncate");
		exit(1);
	}

	void *retval = mmap(NULL, sizeof(wifibroadcast_rx_status_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (retval == MAP_FAILED) {
		perror("mmap");
		exit(1);
	}
	
	
	return (wifibroadcast_rx_status_t*)retval;

}


int main(int argc, char *argv[]) {

//	clear screen
//	printf("\033[2J");

	usleep(1e6);

	wifibroadcast_rx_status_t *t = status_memory_open(argv[1],argv[2]);
	

	int best_dbm = 0;
	int cardcounter = 0;
	int number_cards = t->wifi_adapter_cnt;
	int restarts = 0;

	for(;;) {
        	best_dbm = -1000;
	        for(cardcounter=0; cardcounter<number_cards; ++cardcounter) {
            	    if (best_dbm < t->adapter[cardcounter].current_signal_dbm) best_dbm = t->adapter[cardcounter].current_signal_dbm;
		}

		// Goto line/row 1/1
//		printf("\033[1;1H");
		printf("\033[%s;1H", argv[2]);
		printf("%ddBm(%d)(%d), Rx2: %ddBm(%d)(%d), Rx3: %ddBm(%d)(%d), Rx4: %ddBm(%d)(%d). Total: %ddBm (%d/%d bad) lost: %d                               \n",
		     t->adapter[0].current_signal_dbm, t->adapter[0].received_packet_cnt,t->adapter[0].type,
		     t->adapter[1].current_signal_dbm, t->adapter[1].received_packet_cnt,t->adapter[1].type,
		     t->adapter[2].current_signal_dbm, t->adapter[2].received_packet_cnt,t->adapter[2].type,
		     t->adapter[3].current_signal_dbm, t->adapter[3].received_packet_cnt,t->adapter[3].type,
		     best_dbm, t->damaged_block_cnt, t->received_block_cnt, t->lost_packet_cnt);

		// t->tx_restart_cnt
		//restarts1 = t->tx_restart_cnt;

		if (t->tx_restart_cnt > restarts) {
		    restarts++;
//		    printf("\033[1;1H");
		    printf("\033[%s;1H", argv[2]);
		    printf("!!!!!! TX re-start detected !!!!!!");
		    usleep (1e7);
		}

		usleep(1e5);
	}

	return 0;
}
