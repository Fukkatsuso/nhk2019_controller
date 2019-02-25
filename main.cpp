#include "init2.h"
#include "mbed.h"


int main(){
	wait_ms(300); //全ての基板の電源が入るまで待つ
	pc.baud(921600);


	while(1){
		AdjustCycle(5000);
	}
}

