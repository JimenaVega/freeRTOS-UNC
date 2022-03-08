/**
 * @file us_server.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-07-05
 * 
 * To test:
 * 
 * curl --request POST --url http://192.168.100.6/SoTp3Users/api/users -u USER:SECRET --header 'accept: application/json' --header 'content-type: application/json' --data '{"username":"user1", "password": "user1"}'
 * curl --request GET --url http://192.168.100.6/SoTp3Users/api/users -u USER:SECRET --header 'accept: application/json' --header 'content-type: application/json'
 * 
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <pwd.h>
#include <time.h>
#include <yder.h>
#include <ulfius.h>
#include <jansson.h>

#define PORT 8082
#define TAM 256
#define PATH_LOG "/home/paprika/Documents/Sistemas_operativos_2/practico/soii-2021-sistemas-embebidos-JimenaVega/src/log/users.log"
#define UNUSED(x) (void)(x)
/*curl --request POST --url http://localhost:8082/api/users -u USER:SECRET --header 'accept: application/json' --header 'content-type: application/json' --data '{"username":"HuaHua", "password": "hoho"}'*/
/*curl --request GET --url http://localhost:8082/api/users -u USER:SECRET --header 'accept: application/json' --header 'content-type: application/json'*/
//systemD
//sudo systemctl daemon-reload


int is_repeated_user(const char* username){

	struct passwd* user = getpwent();

	while(user != NULL){
		if(strcmp(user->pw_name, username) == 0){
			endpwent();
			return 1;
		}

		user = getpwent();
	}

	endpwent();
	return 0;
}

struct passwd* get_new_user(const char* username){

	struct passwd* user = getpwent();

	while(user != NULL){
		if(strcmp(user->pw_name, username) == 0){
			return user;
		}
		user = getpwent();
	}

	return NULL;
}
/**
 * Callback function for the web application on /helloworld url call
 */
int create_user(const struct _u_request * request, struct _u_response * response, void * user_data) {

	UNUSED(user_data);
	
	char command [TAM];
	int ctrl_sys;
	printf("Me ejecute hasta aca\n");

	//El <Mensaje> para el log será: Usuario <Id> creado

	json_t* body = ulfius_get_json_body_request(request, NULL);


	if(json_is_null(body)){
		ulfius_set_string_body_response(response, 200, "NULL: create user!");
		return U_CALLBACK_COMPLETE;
	}

	const char* username = strdup(json_string_value(json_object_get(body, "username")));
	char *password = strdup(json_string_value(json_object_get(body, "password")));

	
	if(is_repeated_user(username)){
		printf("Error. User already exists.");
		ulfius_set_string_body_response(response, 400, "Error. User already exists.");
		return U_CALLBACK_CONTINUE;
	}
	
	sprintf(command, "sudo useradd -p $(openssl passwd -1 %s) %s", password, username);
	ctrl_sys = system(command);

	if(ctrl_sys < 0){
		perror("Error in system().");
		exit(EXIT_FAILURE);
	}


	char date[64];
	time_t t = time(NULL);
	struct tm* tm = localtime(&t);
    strftime(date, sizeof(date), "%F %T", tm);
	struct passwd* new_user = get_new_user(username);
	endpwent();

	body = json_pack("{s:i,s:s,s:s}",
					 "id", new_user->pw_uid,
					 "username", new_user->pw_name,
					 "created_at", date
					 );

	ulfius_set_json_body_response(response, 200, body);

	y_log_message(Y_LOG_LEVEL_INFO, "User <%d> created", new_user->pw_uid);

	return U_CALLBACK_CONTINUE;
}


int get_all_users(const struct _u_request * request, struct _u_response * response, void * user_data){

	json_t* users_array = json_array();
	struct passwd* user = getpwent();

	UNUSED(request);
	UNUSED(user_data);

	//El <Mensaje> para el log será: Usuario listados: <cantidad de usuario del SO>
	while(user != NULL){

		json_t* aux_user = json_pack("{s:i,s:s}",
									"user_id", user->pw_uid,
									"username", user->pw_name
									);
		
		json_array_append(users_array, aux_user);
		user = getpwent();
	}
	endpwent();
	ulfius_set_json_body_response(response, 200, users_array);
	printf("Amount of users = %ld\n",json_array_size(users_array));
	
	y_log_message(Y_LOG_LEVEL_INFO, "Listed users: <%d>", json_array_size(users_array));

	return U_CALLBACK_CONTINUE;
}

/**
 * main function
 */
int main(void) {
	struct _u_instance instance;

	// Initialize instance with the port number
	if (ulfius_init_instance(&instance, PORT, NULL, NULL) != U_OK) {
		fprintf(stderr, "Error ulfius_init_instance, abort\n");
		return(EXIT_FAILURE);
	}

	// Endpoint list declaration

	ulfius_add_endpoint_by_val(&instance, "POST", "/api/users", NULL, 0, &create_user, NULL);
	printf("Me ejecute POST hasta aca\n");
	ulfius_add_endpoint_by_val(&instance, "GET", "/api/users", NULL, 0, &get_all_users, NULL);
	printf("Me ejecute GET hasta aca\n");

	// Start the framework
	if (ulfius_start_framework(&instance) == U_OK) {

		printf("Start framework on port %d\n", instance.port);

		y_init_logs("User service",
					Y_LOG_MODE_FILE,
					Y_LOG_LEVEL_INFO,
					PATH_LOG, "Initializing user services log...");
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