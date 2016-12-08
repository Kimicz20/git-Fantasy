# 各个文件夹路径
DIR_OBJ = ./obj
DIR_INC = ./include
DIR_BIN = ./bin
DIR_SRC = ./src

# 执行程序，名称和路径
TARGET 		= Controller
BIN_TARGET  = ${DIR_BIN}/${TARGET}

# 执行参数： 调试信息显示,不生成任何警告信息，头文件位置自定义
CFLAGS  = -std=c++11 -g -w -I${DIR_INC}
LDFLAGS = -lpthread
CC      = g++

# 所有的文件目录
SUBDIRS=\
	Main\
	Socket\
	SupportClass\
	TestCase\
	TinyXML2


# 找到所有cpp文件
SRCS = $(foreach n,$(SUBDIRS),$(wildcard $(join ${DIR_SRC}/,$(n))/*.cpp))
# 编译后将文件名 改为xxx.cpp到xxxx.o，并将路径改成 Moadel/xxx.o,例如Main/test.o
OBJ = $(foreach n,$(SRCS),$(subst src/,,$(patsubst %.cpp,%.o,$(n))))

all:
	@echo "make compile	:编译文件"
	@echo "make exec 	:执行文件"
	@echo "make clean 	:清除文件"
	@echo "make execp 	:运行子程序"

compile:$(BIN_TARGET)
	@echo "\tController 文件编译完毕...\n\n\t编译项目文件"
# 编译所有的cpp文件成中间文件,加@可以不显示 语句，%表示OBJ中除.o的部分
# 例如 g++ -c -std=c++11 -g -w -I./include src/Main/test.cpp -o ./obj/Main/test.o
$(OBJ):%.o: $(DIR_SRC)/%.cpp
	@$(CC) -c $(CFLAGS) $< -o $(join ${DIR_OBJ}/,$(notdir $@))

# 链接所有的中间文件
$(BIN_TARGET):$(OBJ)
	@$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(foreach n,$^,$(join ${DIR_OBJ}/,$(notdir $(n))))

# 运行主程序
exec:
	@./$(BIN_TARGET)
# 运行子程序
execp:
	@make -C ../PBQL3.0 exec

# 清理 文件
clean:
	@rm -rf $(DIR_OBJ)/*.o $(BIN_TARGET)
fileDelete:
	@rm -rf test.xml

# 统计文件夹下所有文件
test:
	@echo $(DIR_EXE)
