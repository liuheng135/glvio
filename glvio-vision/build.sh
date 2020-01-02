#!/bin/bash


#export PILOT_BOARD


function select_board()
{
	local cnt=0
	local choice
	local board_list

	printf "All available boards:\n"
	for boarddir in `ls -F configs/ |grep "/$"` ; do
		boards[$cnt]=`basename $boarddir`
		printf "%4d. %s\n" $cnt ${boards[$cnt]}
		((cnt+=1))
	done

	while true ; do
		read -p "Choice: " choice
		if [ -z "${choice}" ] ; then
		continue
	fi

	if [ -z "${choice//[0-9]/}" ] ; then
		if [ $choice -ge 0 -a $choice -lt $cnt ] ; then
			export PILOT_BOARD="${boards[$choice]}"
			echo "export PILOT_BOARD=${boards[$choice]}" >> .buildconfig
			break
		fi
	fi
	printf "Invalid input ...\n"
	done
}

function select_platform()
{
    local cnt=0
    local choice
    local board_list

    printf "All available platform:\n"
    for platformdir in `ls -F platform/ |grep "/$"` ; do
        platforms[$cnt]=`basename $platformdir`
        printf "%4d. %s\n" $cnt ${platforms[$cnt]}
        ((cnt+=1))
    done
    
    while true ; do
        read -p "Choice: " choice
        if [ -z "${choice}" ] ; then
            continue
        fi  

        if [ -z "${choice//[0-9]/}" ] ; then
            if [ $choice -ge 0 -a $choice -lt $cnt ] ; then
                export PILOT_PLATFORM="${platforms[$choice]}"
		echo "export PILOT_PLATFORM=${platforms[$choice]}" >> .buildconfig
                break
            fi  
        fi  
        printf "Invalid input ...\n"
    done
}

if [ "x$1" = "xconfig" ] ;then
	rm -f .buildconfig
	select_board $?
	select_platform $?
elif [ "x$1" = "xclean" ] ;then
	rm -f ./build/*
	make clean
elif [ "x$1" = "xdistclean" ] ;then
	rm -f ./bsp/config/*
	make clean
	rm -f .buildconfig

elif [ $# -eq 0 ] ; then
	source .buildconfig
	if [ ! -n "$PILOT_BOARD" ] ;then
		echo "run ./build.sh config first please!"
		exit 1
	else
		echo "build for board:$PILOT_BOARD"
	fi
	
	if [ ! -n "PILOT_PLATFORM" ] ;then
		echo "run ./build.sh config first please!"
		exit 1
	else
		echo "build for platform:$PILOT_PLATFORM"
	fi

	rm -rf ./build
	mkdir -p build
	cp -r ./applications/Makefile ./build/
	cp -r ./configs/${PILOT_BOARD}/* ./build/
	cp -r ./platform/${PILOT_PLATFORM}/*.c ./platform/${PILOT_PLATFORM}/*.h ./build/
    make
    exit $?
fi
