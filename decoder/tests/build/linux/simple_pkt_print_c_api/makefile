########################################################
# Copyright 2015 ARM Limited. All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without modification, 
# are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, 
# this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice, 
# this list of conditions and the following disclaimer in the documentation 
# and/or other materials provided with the distribution. 
# 
# 3. Neither the name of the copyright holder nor the names of its contributors 
# may be used to endorse or promote products derived from this software without 
# specific prior written permission. 
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS' AND 
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND 
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
# 
#################################################################################

########  
# RCTDL - test makefile for simple c api packet list test.
#

CC := $(MASTER_CC)
CC_FLAGS := $(MASTER_CC_FLAGS) -Wno-switch
LINKER := $(MASTER_LINKER)	
LINKER_FLAGS := $(MASTER_LINKER_FLAGS) 

PROG = simple_pkt_print_c_api

BUILD_DIR=./$(PLAT_DIR)

VPATH	=	 $(OCSD_TESTS)/source 

CC_INCLUDES	=	\
			-I$(OCSD_TESTS)/source \
			-I$(OCSD_INCLUDE) 

OBJECTS		=	$(BUILD_DIR)/simple_pkt_c_api.o 

LIBS		=	-L$(LIB_TARGET_DIR) -l$(LIB_BASE_NAME) -l$(LIB_CAPI_NAME) 

all:  build_dir test_app copy_libs

test_app: 	$(OBJECTS) $(BIN_TEST_TARGET_DIR)/$(PROG)


 $(BIN_TEST_TARGET_DIR)/$(PROG):
			mkdir -p  $(BIN_TEST_TARGET_DIR)
			$(LINKER) $(LINKER_FLAGS) $(OBJECTS) -Wl,--start-group $(LIBS) -Wl,--end-group -o $(BIN_TEST_TARGET_DIR)/$(PROG)
			cp $(LIB_TARGET_DIR)/*.so .

build_dir:
	mkdir -p $(BUILD_DIR)

.PHONY: copy_libs

copy_libs:
	cp $(LIB_TARGET_DIR)/*.so $(BIN_TEST_TARGET_DIR)/.


#### build rules
$(BUILD_DIR)/%.o : %.c
			$(CC) $(CC_FLAGS) $(CC_INCLUDES) $< -o $@ 

#### clean
.PHONY: clean
clean :
			rm -f  $(BIN_TEST_TARGET_DIR)/$(PROG) $(OBJECTS)

# end of file makefile
