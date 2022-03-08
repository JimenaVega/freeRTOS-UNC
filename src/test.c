#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/stat.h>
#include <string.h>
#include <inttypes.h>

#define DIM(x) (sizeof(x)/sizeof(*(x)))
#define ENDPOINT "api/servers/get_goes"
#define PORT 8081
#define TAM 512
#define DATE_ENTRY 3

int months[] = {31,28,31,30,31,30,31,31,30,31,30,31};
// static const char     *sizes[]   = {"TB", "GB", "MB", "KB", "B" };
// static const uint64_t  exbibytes = 1024ULL * 1024ULL * 1024ULL * 1024ULL;

// char* conversion(uint64_t size)
// {   
//     char     *result = (char *) malloc(sizeof(char) * 20);
//     uint64_t  multiplier = exbibytes;
//     int i;

//     for (i = 0; i < DIM(sizes); i++, multiplier /= 1024)
//     {   
//         if (size < multiplier){
//             continue;
//         }
//         else{
//             sprintf(result, "%.1f %s", (float) size / multiplier, sizes[i]);
//         }
//         return result;
//     }
//     strcpy(result, "0");
//     return result;
// }

// int get_file_size(const char* filename){

//     char path[1024];

//     sprintf(path, "../data/%s",filename);

//     struct stat st; /*declare stat variable*/

//     if(stat(path, &st) == 0)
//         return ((int)st.st_size);
//     else
//         return -1;
// }

int is_leap_year(int year){

    if(((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0)){
        return 1;
    }

    return 0;
}

void download_file(char data[TAM], int year, int acum_day){

    
    FILE *file;

    file = popen(data, "r");

    if(file == NULL){
        perror("Failed to run command \n");
        exit(EXIT_FAILURE);
    }
    printf("BUFFER = |%s|", data);

    bzero(data, TAM);

    if(fgets(data, TAM, file) == NULL){
        perror("Failed fgets\n");
        exit(EXIT_FAILURE);
    }
    pclose(file);
  
    //printf("BUFFER = %s", buffer);

    char* nc_file_name;
    nc_file_name = strtok(data, " ");

    for(int i=0; i<4; i++){
        nc_file_name = strtok(NULL, " ");
    }

    char command[TAM];
    
    nc_file_name[strlen(nc_file_name)-1]='\0'; 
    printf("token = |%s|\n", nc_file_name);

    sprintf(command, "cd ../data && aws s3 cp s3://noaa-goes16/ABI-L2-RRQPEF/%d/%03d/03/%s . --no-sign-request &", year, acum_day, nc_file_name);
    printf("COMMAND = %s\n", command);
    system(command);
    
}

int is_valid(int date[DATE_ENTRY]){

    for(int i=0; i<DATE_ENTRY; i++){

        if(date[i] <= 0){
            return 0;
        }
    }

    if(date[1] > 12){
        return 0;
    }

    if(date[2] > months[date[1] - 1]){
        return 0;
    }

    return 1;
}

int main(){
/*              "file_id": 1
              "link": "http://{{server}}/data/OR_ABI-L2-MCMIPF-M6_G16_s20210661636116_e20210661638489_c20210661638589.nc",
              "filesize": "345 kb"  
*/
   /* DIR *d;
    struct dirent *dir;
    d = opendir("../data");
    char buffer [512];
    int file_id = 0;

    if(d){
        while ((dir = readdir(d)) != NULL){
            if(strcmp(dir->d_name, ".") && strcmp(dir->d_name, "..")){

                printf("file_id = %d\n", file_id);
                
                sprintf(buffer,"http://localhost:%d/%s/data/%s", PORT, ENDPOINT, dir->d_name);
                printf("%s\n", buffer);
                printf("file size = %s\n", conversion(get_file_size(dir->d_name)));
                file_id++;
            }
        }
        closedir(d);
    }*/

   //get_file_size("../data/OR_ABI-L2-RRQPEF-M6_G16_s20193371340201_e20193371349509_c20193371350041.nc");

    //"Y%m%d%h"
    char datetime[] = "2020-12-30";
    char *token;
    int date[DATE_ENTRY];//year/month/day
    //int year = 0, month = 0, day = 0;

    printf("datatime = %s\n", datetime);
    token = strtok(datetime, "-");
    date[0] = (int)strtol(token, NULL, 10);
    
    for(int i=1; i<DATE_ENTRY; i++){
        token = strtok(NULL, "-");
        date[i] = (int)strtol(token, NULL, 10);
    }

    printf("year = %d, month = %d, day = %d\n", date[0], date[1], date[2]);

    if(is_valid(date)){

        if(is_leap_year(date[0])){
            months[1]++;
        }
    
        int acum_day = 0;

        for(int i = 0; i<(date[1]-1); i++){
            acum_day += months[i];
            printf("acum_day = %d\n", acum_day);
        }

        acum_day += date[2];
        printf("\nacum_day = %d\n", acum_day);

        printf("año=%d dayacum = %d, dia = %d\n", date[0], acum_day, date[2]);
        months[1]--;

        char data[TAM];
        sprintf(data, " aws s3 ls noaa-goes16/ABI-L2-RRQPEF/%d/%03d/03/ --human-readable --no-sign-request 2>&1", date[0], acum_day);
        printf("DATA = %s\n", data);

        download_file(data, date[0], acum_day);

    }
    // token = strtok(NULL, "-");
    // month = (int)strtol(token, NULL, 10);

    // token = strtok(NULL, "-");
    // day = (int)strtol(token, NULL, 10);

    

 
   
    return(0);
}