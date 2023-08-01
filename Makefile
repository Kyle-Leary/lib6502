# Variables
CC := gcc
# have a front-facing api for the library through the src/api headerfile directory.
# we're using headers both internally and externally with this.
CFLAGS += -Isrc -Isrc/api -g -Wall

BACKEND_DIR := linux

# Directories
SRC_DIR := src 
OBJ_DIR := build

TARGET := lib6502.a

# just compile everything in SRC_DIR.
SRCS := $(shell find $(SRC_DIR) -type f -name "*.c") 
OBJS = $(SRCS:.c=.o)

# set this flag only when we run the "make test" rule.
ifdef TESTING
CFLAGS += "-DTESTING=1"
endif

all: $(TARGET)
	@echo  Successfully created the library.

# then archive them into a .a file.
$(TARGET): $(OBJS) 
	ar rcs $(TARGET) $(OBJS)

# compile all the objects
.c.o:
	$(CC) $(CFLAGS) -c $<  -o $@


clean:
	rm -f $(shell find . -name "*.o") $(TARGET)
