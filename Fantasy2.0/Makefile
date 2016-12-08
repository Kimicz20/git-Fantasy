# 循环编译 两个子程序的 make文件
# 需要make的文件目录
SUBDIRS = Controller\
	PBQL3.0

all:
	@echo "make compile	:编译文件"
	@echo "make exec 	:执行文件"
	@echo "make clean 	:清除文件"

compile_all:$(SUBDIRS)

$(SUBDIRS): CHO
	@make -C $@ compile

CHO:
	@echo "############		开始循环编译cpp文件		############\n\t编译Controller文件"

compile_c:
	@make -C Controller compile

compile_p:
	@make -C PBQL3.0 compile

exec:ECHO
	@make -C Controller exec
ECHO:
	@echo "############		执行控制程序		############"

clean:
	@echo "############		清理文件 	############"
	@make -C Controller clean
	@make -C PBQL3.0 clean
	@make -C PBQL3.0 fileDelete
	@echo "\t清理完毕"
