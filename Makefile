CC = g++
TARGET ?= simulator
SRC_DIRS ?= ./src

SRCS := $(wildcard $(SRC_DIRS)/*.cpp)
OBJS := $(patsubst $(SRC_DIRS)/%.cpp,$(SRC_DIRS)/%.o,$(SRCS))

INC_DIRS := $(shell find $(SRC_DIRS) -type d)
INC_FLAGS := $(addprefix -I,$(INC_DIRS))

CPPFLAGS ?= $(INC_FLAGS) -std=c++11 -MMD -MP -Wall -g
OPENGLFLAGS := -lglut -lGL -lGLU#-lGLEW -lGLU

OTHERS := out.tga

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) $(OPENGLFLAGS) -o $@ $(LOADLIBES) $(LDLIBS)

.PHONY: clean
clean:
	$(RM) $(TARGET) $(OBJS) $(DEPS) $(OTHERS)

-include $(DEPS)
