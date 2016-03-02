# MEMO
# $(SRCDIR) 以下にある、全ての .cpp ファイルを検索してコンパイルする makefile
# $(OBJDIR) 以下に中間生成ファイルを出力する

# 出力する実行ファイル名
TARGET    = $(shell basename `pwd`)
# ソースコードの入っているディレクトリ
SRCDIR    = source
# 中間生成ファイルの出力先
OBJDIR    = obj

INCLUDE   =
LDLIBS    =
FRAMEWORK =

SRCS      = $(shell basename -a `find $(SRCDIR) -name "*.cpp"`)
OBJS      = $(addprefix $(OBJDIR)/, $(SRCS:.cpp=.o))
HEADS     = $(SRCS:.cpp=.h)
DEPENDS   = $(OBJS:.o=.d)
VPATH     = $(shell find $(SRCDIR) -type d)
CXXFLAGS  = -MMD -MP -O3 -std=c++14

OS = $(shell uname)

ifeq ($(OS),Darwin)
LDLIBS += $(FRAMEWORK)
endif


$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDLIBS)

all: clean $(TARGET)

test: $(TARGET)
	./$(TARGET)
	
library: $(OBJS)
	ar -r lib$(TARGET).a $(OBJS)

clean:
	rm -rf $(TARGET) lib$(TARGET).a $(OBJS) $(DEPENDS)

allclean:
	rm -rf $(OBJDIR)

$(OBJDIR)/%.o: %.cpp
	@[ -d $(OBJDIR) ] || mkdir -p $(OBJDIR)
	$(CXX) $(CXXFLAGS) -o $@ -c $< $(INCLUDE)

-include $(DEPENDS)
