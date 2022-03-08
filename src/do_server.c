/**
 * @file do_server.c
 * @author Jimena Vega Cuevas
 * @brief 
 * Servicio de descargas. Para poder consultar los productos disponibles se puede ingresar a:
 * https://noaa-goes16.s3.amazonaws.com/index.html#ABI-L2-RRQPEF/. Aqui se veran productos de la forma:
 * https://noaa-goes16.s3.amazonaws.com/ABI-L2-RRQPEF/2019/362/20/OR_ABI-L2-RRQPEF-M6_G16_s20193622000221_e20193622009529_c20193622010039.nc 
 * 
 * Para interactuar con este servicio se puede utilizar por ejemplo:
 * curl --request POST --url http://192.168.100.6/SoTp3Downloads/api/servers/get_goes  -u USER:SECRET --header 'accept: application/json' --header 'content-type: application/json' --data '{"product": "ABI-L2-RRQPEF", "datetime": "2020-12-30"}'
 * curl --request GET --url http://192.168.100.6/SoTp3Downloads/api/servers/get_goes -u USER:SECRET --header 'accept: application/json' --header 'content-type: application/json'
 *
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <stdio.h>
#include <ulfius.h>
#include <sys/stat.h>
#include <string.h>
#include <inttypes.h>
#include <dirent.h>
#include <jansson.h>
#include <errno.h>

#define PORT 8081
#define DIM(x) (sizeof(x)/sizeof(*(x)))
//#define ENDPOINT "api/servers/get_goes"
#define ENDPOINT "downloads/data"
#define SERVER "192.168.100.6"
#define PATH_LOG "/home/paprika/Documents/Sistemas_operativos_2/practico/soii-2021-sistemas-embebidos-JimenaVega/src/log/downloads.log"
#define PATH_DATA "/home/paprika/Documents/Sistemas_operativos_2/practico/soii-2021-sistemas-embebidos-JimenaVega/data"
#define TAM 512
#define DATE_ENTRY 3
#define UNUSED(x) (void)(x)

int files_reg = 0;
int new_files_reg = 0;

static const char     *sizes[]   = {"TB", "GB", "MB", "KB", "B" };
static const uint64_t  exbibytes = 1024ULL * 1024ULL * 1024ULL * 1024ULL;
static int months[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

char* conversion(uint64_t size){ 

    char     *result = (char *) malloc(sizeof(char) * 20);
    uint64_t  multiplier = exbibytes;

    for (size_t i = 0; i < DIM(sizes); i++, multiplier /= 1024){   
        if (size < multiplier){
            continue;
        }
        else{
            sprintf(result, "%.1f %s", ((float) size / ((float) multiplier)), sizes[i]);
        }
        return result;
    }
    strcpy(result, "0");

    return result;
}

int get_file_size(const char* filename){

    char path[1024];

    sprintf(path, "%s/%s", PATH_DATA, filename);
    struct stat st;

    if(stat(path, &st) == 0)
        return ((int)st.st_size);
    else
        return -1;
}


int get_goes_list (const struct _u_request * request, struct _u_response * response, void * user_data) {

    UNUSED(user_data);
    UNUSED(request);
	
    DIR *d;
    struct dirent *dir;
    d = opendir(PATH_DATA);
	json_t* files_array = json_array();
    char link [TAM];
    int file_id = 0;
    uint64_t file_size = 0;

    if(d){
        while ((dir = readdir(d)) != NULL){
            if(strcmp(dir->d_name, ".") && strcmp(dir->d_name, "..")){

				sprintf(link,"http://%s/%s/%s", SERVER, ENDPOINT, dir->d_name);

                file_size = (uint64_t) get_file_size(dir->d_name);

				json_t* aux_file = json_pack("{s:i,s:s,s:s}",
								"file_id", file_id,
								"link", link,
								"filesize",conversion(file_size)
								);
				json_array_append(files_array, aux_file);

                file_id++;
            }
        }
		ulfius_set_json_body_response(response, 200, files_array);
        closedir(d);
    }
	else{

        char* err = strdup(strerror(errno));
		ulfius_set_string_body_response(response, 404, err);
	}

    //Mensaje en log:
    new_files_reg = 0;
    files_reg = file_id;
    y_log_message(Y_LOG_LEVEL_INFO, "Files in server : %d", files_reg);
	
	return U_CALLBACK_CONTINUE;
}

char* get_file_name(char data[TAM]){

	FILE *file;

    file = popen(data, "r");

    if(file == NULL){
        return "";
    }

    bzero(data, TAM);

    if(fgets(data, TAM, file) == NULL){
        return "";
    }
    pclose(file);
  

    char* nc_file_name;
    nc_file_name = strtok(data, " ");

    for(int i=0; i<4; i++){
        nc_file_name = strtok(NULL, " ");
    }

	nc_file_name[strlen(nc_file_name)-1]='\0'; 

	return nc_file_name;

}


int is_leap_year(int year){

    if(((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0)){
        return 1;
    }

    return 0;
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

json_t* get_json_msg(char* filename){

    int file_id = 0;
    char link[TAM];

    uint64_t file_size = (uint64_t) get_file_size(filename);

    sprintf(link, "http://%s/%s/%s", SERVER, ENDPOINT, filename);

    json_t* msg = json_pack("{s:i,s:s,s:s}",
								"file_id", file_id,
								"link", link,
								"filesize",conversion(file_size)
								);

    return msg;
}

char* check_saved_files(int year, int day_acum){

    DIR *d;
    d = opendir(PATH_DATA);
	struct dirent *dir;
	char user_req_date[TAM];

	sprintf(user_req_date, "%d%d03", year, day_acum);

    if(d){
        while ((dir = readdir(d)) != NULL){
            if(strcmp(dir->d_name, ".") && strcmp(dir->d_name, "..")){

				if(strstr(dir->d_name, user_req_date) != NULL){
					
					return dir->d_name;
				}
            }
        }
        closedir(d);
    }

	return "";
}

int download_goes (const struct _u_request * request, struct _u_response * response, void * user_data) {

    UNUSED(user_data);

	json_t* body = ulfius_get_json_body_request(request, NULL);

	if(json_is_null(body)){
		ulfius_set_string_body_response(response, 200, "NULL: create user!");
		return U_CALLBACK_COMPLETE;
	}

	char* product = strdup(json_string_value(json_object_get(body, "product")));
	char* datetime = strdup(json_string_value(json_object_get(body, "datetime")));

	//parse datetime
    char *token;
    int date[DATE_ENTRY];                     // date[0] = year, date[1] = month date[2] = day
    
     if((token = strtok(datetime, "-")) == NULL){

        ulfius_set_string_body_response(response, 404, "Error. Invalid input.");
        return U_CALLBACK_CONTINUE;
    }

    date[0] = (int)strtol(token, NULL, 10);
    
    for(int i=1; i<DATE_ENTRY; i++){
        token = strtok(NULL, "-");

         if(token == NULL){
            ulfius_set_string_body_response(response, 404, "Error. Invalid input.");
            return U_CALLBACK_CONTINUE;
         }

        date[i] = (int)strtol(token, NULL, 10);
    }

    //Validacion y obtencion de link de descarga
    if(is_valid(date)){

        if(is_leap_year(date[0])){
            months[1]++;
        }
    
        int acum_day = 0;

        for(int i = 0; i<(date[1]-1); i++){
            acum_day += months[i];
        }

        acum_day += date[2];
        months[1]--;

		char* nc_file = strdup(check_saved_files(date[0], acum_day));
        json_t *json_to_user;

		if(strlen(nc_file) != 0){
		
            json_to_user = get_json_msg(nc_file);
            ulfius_set_json_body_response(response, 200, json_to_user);
		}
		else{

			//Se enlistan todos los archivos que se encuentran en la carpeta noaa-goes16/RRQPEF/year/acum_day/03
			char data[TAM];
			sprintf(data, " aws s3 ls noaa-goes16/%s/%d/%03d/03/ --human-readable --no-sign-request 2>&1", product, date[0], acum_day);
			char* nc_file_to_dwn = strdup(get_file_name(data));

			if(strlen(nc_file_to_dwn) == 0){

				ulfius_set_string_body_response(response, 404, "Error. File not found");
				return U_CALLBACK_CONTINUE;
			}

            char command[TAM];
    
            sprintf(command, "cd %s && aws s3 cp s3://noaa-goes16/%s/%d/%03d/03/%s . --no-sign-request &", 
                            PATH_DATA, product, date[0], acum_day, nc_file_to_dwn);

            if(system(command) == -1){

                ulfius_set_string_body_response(response, 404, "Error downloading file.");
            }
            ulfius_set_string_body_response(response, 200, "File is not in server. Downloading file...");
            new_files_reg++;

            free(nc_file_to_dwn);
		}

        free(nc_file);

    }
    else{
        ulfius_set_string_body_response(response, 404, "Error. Invalid date.");
    }

	y_log_message(Y_LOG_LEVEL_INFO, "Amount of new downloaded files : %d", new_files_reg);
	y_log_message(Y_LOG_LEVEL_INFO, "Amount of preexisting files : %d", files_reg);


	return U_CALLBACK_CONTINUE;
}

int main(void) {
	struct _u_instance instance;

	// Inicializacion de la instancia con el puerto
	if (ulfius_init_instance(&instance, PORT, NULL, NULL) != U_OK) {
		fprintf(stderr, "Error ulfius_init_instance, abort\n");
		return(1);
	}

	// Declaracion de endpoints
	ulfius_add_endpoint_by_val(&instance, "GET", "/api/servers/get_goes", NULL, 0, &get_goes_list, NULL);
	ulfius_add_endpoint_by_val(&instance, "POST", "/api/servers/get_goes", NULL, 0, &download_goes, NULL);


	if (ulfius_start_framework(&instance) == U_OK) {
		printf("Start framework on port %d\n", instance.port);

        //Creacion de log
		y_init_logs("Download service",
					Y_LOG_MODE_FILE,
					Y_LOG_LEVEL_INFO,
					PATH_LOG, "Initializing download services log...");
	
		pause();
	}
	else {
		fprintf(stderr, "Error starting framework\n");
	}
	y_close_logs();
	printf("End framework\n");

	ulfius_stop_framework(&instance);
	ulfius_clean_instance(&instance);

	return 0;
}