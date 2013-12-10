SUBDIRS := hello_world Glumanda Timer SPI Audio i2c move_to_audio onewire_temperatursensor display

SELF_DIR := $(dir $(lastword $(MAKEFILE_LIST)))
include $(abspath $(addprefix $(SELF_DIR),$(addsuffix /target.mak,$(SUBDIRS))))
SELF_DIR := $(abspath $(SELF_DIR)/..)/
