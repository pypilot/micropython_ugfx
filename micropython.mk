TEST_MOD_DIR := $(USERMOD_DIR)

# Add all C files to SRC_USERMOD.
SRC_USERMOD += $(TEST_MOD_DIR)/modugfx.c
SRC_USERMOD += $(TEST_MOD_DIR)/ilidriver.c
SRC_USERMOD += $(TEST_MOD_DIR)/upy_wrap.cpp
SRC_USERMOD += $(TEST_MOD_DIR)/ugfx.cpp

# We can add our module folder to include paths if needed
# This is not actually needed in this test.
CFLAGS_USERMOD += -I$(TEST_MOD_DIR)
