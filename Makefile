
CC = gcc
FLAG = -g -O3 -Wall -Werror -pedantic -Wextra -Wconversion -std=gnu11
LIB  = -lulfius -ljansson -lyder


all: bin install


bin: clean create
	$(CC) $(FLAG) -o bin/us_serv src/us_server.c $(LIB)
	$(CC) $(FLAG) -o bin/us_down src/do_server.c $(LIB)
clean: 
	rm -f bin/*
	rm -rf src/log
create:
	mkdir -p src/log
	touch ./src/log/downloads.log
	touch ./src/log/users.log
install:
	#!/bin/bash

	sudo cp -f ./rsc/SoTp3Users.service /etc/systemd/system/SoTp3Users.service
	sudo cp -f ./rsc/SoTp3Download.service /etc/systemd/system/SoTp3Download.service

	sudo systemctl daemon-reload
	sudo systemctl start SoTp3Users
	sudo systemctl enable SoTp3Users
	
	sudo systemctl start SoTp3Download
	sudo systemctl enable SoTp3Download