#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void my_task(){

    while(1){
        puts("Hello world\n");
    }
}

int main(){
    
    enableFlushAfterPrintf();

    xTaskCreate(my_task, (signed char*)"my_task", 1024, NULL, 1, NULL);

    vTaskStartScheduler();

    return 0;
}